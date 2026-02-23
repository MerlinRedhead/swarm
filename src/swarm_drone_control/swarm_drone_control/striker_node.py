#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import math
import time
import subprocess
import os
import signal
import asyncio
from cv_bridge import CvBridge

# ROS Messages
from sensor_msgs.msg import NavSatFix, Image
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from swarm_interfaces.msg import AgentStatus, SwarmTask, GuiOverlay
from rclpy.qos import qos_profile_sensor_data

# --- КОНСТАНТЫ БОЕВОЙ ЧАСТИ ---
SAFE_ALTITUDE = 30.0  # Высота подлета
HANDOVER_DIST = 60.0  # Дистанция начала визуального поиска
ATTACK_SPEED = 22.0  # Скорость атаки (м/с)
PN_GAIN = 3.5  # Коэффициент наведения


class StrikerNode(Node):
    def __init__(self):
        super().__init__('striker_node')

        # --- НАСТРОЙКИ ---
        self.declare_parameter('agent_id', 'striker_1')
        self.declare_parameter('server_ip', '192.168.1.100')
        self.declare_parameter('srt_port', 8890)
        self.declare_parameter('video_device', '/dev/video0')

        self.agent_id = self.get_parameter('agent_id').value
        self.server_ip = self.get_parameter('server_ip').value
        self.srt_port = self.get_parameter('srt_port').value
        self.device = self.get_parameter('video_device').value

        # --- СОСТОЯНИЕ ---
        self.state = "IDLE"  # IDLE, APPROACH, TERMINAL
        self.mavros_state = State()
        self.current_gps = None
        self.target_gps = None
        self.target_uuid = ""

        # --- ТРЕКИНГ ---
        self.bridge = CvBridge()
        self.tracker = None
        self.tracking_active = False
        self.server_hint_bbox = None  # Подсказка от сервера
        self.last_hint_time = 0
        self.stream_process = None

        # Переменные PN
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.last_loop_time = self.get_clock().now()

        # --- ПОДПИСКИ ---
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(State, 'mavros/state', self.state_cb, qos_profile_sensor_data)
        self.create_subscription(SwarmTask, '/swarm/task_assignment', self.task_cb, 10)

        # Камера (для трекера) и Оверлей (для подсказки)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10) # Картинки лучше оставлять как есть или использовать специальный QoS
        self.create_subscription(GuiOverlay, '/swarm/gui_overlay', self.overlay_cb, qos_profile_sensor_data)

        # --- ПУБЛИКАЦИИ ---
        self.status_pub = self.create_publisher(AgentStatus, '/swarm/agent_status', qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', qos_profile_sensor_data)

        # Сервисы
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')

        self.create_timer(0.05, self.control_loop)  # 20 Hz

        # ЗАПУСК ВИДЕО (ЧТОБЫ СЕРВЕР НАС ВИДЕЛ)
        self.start_video_stream()

        self.get_logger().info(f"STRIKER [{self.agent_id}] READY. Stream Active.")

    # --- CALLBACKS ---
    def gps_cb(self, msg):
        self.current_gps = msg

    def state_cb(self, msg):
        self.mavros_state = msg

    def task_cb(self, msg):
        if msg.assigned_agent_id != self.agent_id: return
        if msg.task_type == "STRIKE":
            self.target_gps = (msg.lat, msg.lon)
            self.target_uuid = msg.target_uuid
            if self.state == "IDLE":
                self.get_logger().info(f"ENGAGING TARGET: {self.target_uuid}")
                # Асинхронно устанавливаем режим
                asyncio.create_task(self.set_mode("GUIDED"))
                self.state = "APPROACH"

    def overlay_cb(self, msg):
        """
        ПОЛУЧЕНИЕ ПОДСКАЗКИ ОТ СЕРВЕРА
        Сервер смотрит наше видео и говорит: "Цель в точке X, Y"
        """
        # Проверяем, что подсказка именно для нас (source_agent_id == striker_1)
        if msg.source_agent_id == self.agent_id:
            # Сохраняем подсказку.
            self.server_hint_bbox = (msg.x, msg.y, msg.w, msg.h)
            self.last_hint_time = time.time()

    def image_cb(self, msg):
        if self.state == "IDLE": return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        h, w = cv_image.shape[:2]

        # ЛОГИКА ЗАХВАТА (HANDOVER)
        # Если мы летим по GPS, но дистанция < 60м и есть свежая подсказка
        if self.state == "APPROACH" and not self.tracking_active:
            if self.is_close_to_target() and self.server_hint_bbox:
                if (time.time() - self.last_hint_time < 0.5):  # Подсказка не старше 0.5 сек

                    nx, ny, nw, nh = self.server_hint_bbox
                    bbox = (int(nx * w), int(ny * h), int(nw * w), int(nh * h))

                    self.tracker = cv2.TrackerKCF_create()
                    self.tracker.init(cv_image, bbox)
                    self.tracking_active = True

                    self.state = "TERMINAL"
                    self.get_logger().warn(">>> VISUAL LOCK CONFIRMED! <<<")

        # ЛОГИКА ВЕДЕНИЯ (TERMINAL)
        if self.state == "TERMINAL" and self.tracking_active:
            success, bbox = self.tracker.update(cv_image)
            if success:
                self.execute_pn_guidance(bbox, w, h)
            else:
                self.get_logger().error("TRACKER LOST! Reverting to GPS.")
                self.tracking_active = False
                self.state = "APPROACH"

    def control_loop(self):
        self.publish_status()
        if not self.current_gps or not self.target_gps: return

        # Если мы еще не захватили цель глазами, летим по GPS
        if self.state == "APPROACH":
            self.fly_to_gps(self.target_gps, SAFE_ALTITUDE)

    def execute_pn_guidance(self, bbox, w, h):
        """Пропорциональная навигация на основе визуального трекера"""
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        if dt <= 0: dt = 0.05

        cx, cy = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
        error_x = (cx / w) - 0.5
        error_y = (cy / h) - 0.5

        los_rate_x = (error_x - self.prev_error_x) / dt
        los_rate_y = (error_y - self.prev_error_y) / dt

        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        # CRITICAL FIX: To command Vel X, Vel Z, and Yaw Rate, use mask: 0x05C7 (0b0000010111000111)
        # Ensure ArduPilot interprets this specifically as a velocity+yawrate body command.
        msg.type_mask = 0b0000010111000111
        msg.velocity.x = ATTACK_SPEED

        # Рулим (Yaw) и пикируем (Vel Z)
        msg.yaw_rate = -1.0 * PN_GAIN * (los_rate_x + 1.0 * error_x)
        
        # CRITICAL MATH FIX: Downward velocity (Z) in NED frame.
        # Reduced aggressive PD gain (error_y * 10.0 -> error_y * 4.0) to prevent jerking.
        proposed_z_vel = (error_y * 4.0 + los_rate_y * PN_GAIN * 2.0)
        
        # Clamped to safe multirotor dive limits: Max 5 m/s down, 2 m/s up. 
        # (NED frame: Positive Z is DOWN)
        msg.velocity.z = max(min(proposed_z_vel, 5.0), -2.0)

        self.vel_pub.publish(msg)
        self.prev_error_x, self.prev_error_y = error_x, error_y
        self.last_loop_time = now

    def fly_to_gps(self, target, alt):
        """Подлет по координатам"""
        msg = PositionTarget()
        # CRITICAL FIX: Change from FRAME_GLOBAL_INT to FRAME_GLOBAL_REL_ALT
        msg.coordinate_frame = PositionTarget.FRAME_GLOBAL_REL_ALT
        # CRITICAL FIX: Change from 0xFF8 (ignores yaw) to 0xDF8 (uses yaw)
        msg.type_mask = 0b0000110111111000
        msg.lat_int = int(target[0] * 1e7)
        msg.lon_int = int(target[1] * 1e7)
        msg.alt = float(alt)

        # Поворачиваем нос к цели
        d_lat = target[0] - self.current_gps.latitude
        d_lon = target[1] - self.current_gps.longitude
        msg.yaw = math.atan2(d_lon, d_lat)

        self.vel_pub.publish(msg)

    def is_close_to_target(self):
        if not self.current_gps or not self.target_gps: return False
        dlat = (self.target_gps[0] - self.current_gps.latitude) * 111319
        dlon = (self.target_gps[1] - self.current_gps.longitude) * 111319 * math.cos(math.radians(self.target_gps[0]))
        dist = math.sqrt(dlat ** 2 + dlon ** 2)
        return dist < HANDOVER_DIST

    def publish_status(self):
        msg = AgentStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.agent_id = self.agent_id
        msg.role = "STRIKER"
        msg.state = self.state
        if self.current_gps:
            msg.lat, msg.lon, msg.alt = self.current_gps.latitude, self.current_gps.longitude, self.current_gps.altitude
        self.status_pub.publish(msg)

    def start_video_stream(self):
        """Запуск видеопотока на сервер"""
        uri = f"srt://{self.server_ip}:{self.srt_port}?streamid=publish:{self.agent_id}"

        if "aarch64" in os.uname().machine:
            # Orange Pi 5
            cmd = ['gst-launch-1.0', '-v', 'v4l2src', f'device={self.device}', '!',
                   'video/x-raw,width=1280,height=720,framerate=30/1', '!',
                   'mpph264enc', 'gop=30', 'bps=2000000', '!', 'mpegtsmux', '!',
                   'srtserversink', f'uri={uri}', 'latency=20']
        else:
            # PC
            cmd = ['gst-launch-1.0', 'v4l2src', f'device={self.device}', '!',
                   'videoconvert', '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast', '!',
                   'mpegtsmux', '!', 'srtserversink', f'uri={uri}']

        self.stream_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    async def set_mode(self, mode):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request(custom_mode=mode)
            try:
                response = await self.set_mode_client.call_async(req)
                return response.mode_sent
            except Exception as e:
                self.get_logger().error(f"SetMode exception: {e}")
                return False
        return False

    async def arm(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request(value=True)
            try:
                response = await self.arming_client.call_async(req)
                return response.success
            except Exception as e:
                self.get_logger().error(f"Arming exception: {e}")
                return False
        return False

    def destroy_node(self):
        if self.stream_process: os.kill(self.stream_process.pid, signal.SIGKILL)
        super().destroy_node()


def main_loop(args=None):
    rclpy.init(args=args)
    node = StrikerNode()
    
    # Запускаем asyncio loop для поддержки асинхронных MAVROS вызовов
    loop = asyncio.get_event_loop()
    
    # Обертка для запуска ROS 2 spin в фоне
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_loop()