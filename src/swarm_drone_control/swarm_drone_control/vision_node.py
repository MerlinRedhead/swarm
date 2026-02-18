#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import os
import signal
import math
import time
import sys
import tf_transformations

# ROS Messages
from sensor_msgs.msg import NavSatFix, BatteryState, Imu
from mavros_msgs.msg import State, PositionTarget, MountControl
from swarm_interfaces.msg import AgentStatus, SwarmTask, GimbalState
from std_msgs.msg import Header


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- КОНФИГУРАЦИЯ ---
        self.declare_parameter('agent_id', 'vision_1')
        self.declare_parameter('server_ip', '192.168.1.100')
        self.declare_parameter('srt_port', 8890)
        self.declare_parameter('video_device', '/dev/video0')

        self.agent_id = self.get_parameter('agent_id').value
        self.server_ip = self.get_parameter('server_ip').value
        self.srt_port = self.get_parameter('srt_port').value
        self.device = self.get_parameter('video_device').value

        # --- СОСТОЯНИЕ ---
        self.current_gps = None
        self.current_alt = 0.0
        self.orientation = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw корпуса
        self.mavros_state = State()
        self.battery = 0.0
        self.stream_process = None

        # Целевые точки
        self.formation_goal = None  # (Lat, Lon, Alt) - Где стоять
        self.look_at_target = None  # (Lat, Lon, Alt) - Куда смотреть

        # Текущее состояние подвеса (для отчета)
        self.gimbal_rpy = [0.0, 0.0, 0.0]

        # --- ПОДПИСКИ ---
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.gps_cb, 10)
        self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.create_subscription(BatteryState, 'mavros/battery', self.batt_cb, 10)
        self.create_subscription(Imu, 'mavros/imu/data', self.imu_cb, 10)
        self.create_subscription(SwarmTask, '/swarm/task_assignment', self.task_cb, 10)

        # --- ПУБЛИКАЦИИ ---
        self.status_pub = self.create_publisher(AgentStatus, '/swarm/agent_status', 10)
        # Публикуем состояние подвеса (ты скидывал GimbalState.msg)
        self.gimbal_pub = self.create_publisher(GimbalState, '/swarm/gimbal_state', 10)

        self.pos_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_position/global', 10)
        self.mount_pub = self.create_publisher(MountControl, 'mavros/mount_control', 10)

        # Таймеры
        self.create_timer(0.1, self.publish_status)  # 10 Гц телеметрия
        self.create_timer(0.1, self.control_loop)  # 10 Гц управление (плавность)

        self.start_video_stream()
        self.get_logger().info(f"SCOUT [{self.agent_id}] ONLINE. BODY LOCKED, GIMBAL TRACKING.")

    # --- CALLBACKS ---
    def gps_cb(self, msg):
        self.current_gps = msg
        self.current_alt = msg.altitude

    def state_cb(self, msg):
        self.mavros_state = msg

    def batt_cb(self, msg):
        self.battery = msg.voltage

    def imu_cb(self, msg):
        try:
            q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            (r, p, y) = tf_transformations.euler_from_quaternion(q)
            self.orientation = [r, p, y]
        except:
            pass

    def task_cb(self, msg):
        if msg.assigned_agent_id != self.agent_id and msg.assigned_agent_id != "all":
            return

        # 1. СТАТЬ В СТРОЙ
        if msg.task_type == "SCOUT_POINT" and msg.target_uuid == "":
            self.formation_goal = (msg.lat, msg.lon, msg.alt)
            # Если нет цели слежения, сбрасываем взгляд
            if not self.look_at_target:
                self.get_logger().info("Holding Formation Position.")

        # 2. СЛЕЖЕНИЕ (ТОЛЬКО КАМЕРОЙ)
        elif msg.task_type == "TRACK" or (msg.task_type == "SCOUT_POINT" and msg.target_uuid != ""):
            self.look_at_target = (msg.lat, msg.lon, 0.0)
            self.get_logger().info(f"Gimbal Tracking Target: {msg.target_uuid}")

    def control_loop(self):
        """
        Раздельное управление:
        - Дрон: летит в точку строя, Yaw=0 (Север)
        - Подвес: крутится на цель
        """
        if not self.current_gps or not self.formation_goal:
            return

        # А. ПОЛЕТ ДРОНА (BODY)
        target_lat, target_lon, target_alt = self.formation_goal

        # Дрон всегда смотрит на Север (0.0) для стабильности строя
        # Или можно сохранять текущий Yaw (self.orientation[2])
        body_yaw = 0.0

        pos_msg = PositionTarget()
        pos_msg.coordinate_frame = PositionTarget.FRAME_GLOBAL_INT
        pos_msg.type_mask = 0b0000100111111000  # Pos + Yaw

        pos_msg.lat_int = int(target_lat * 1e7)
        pos_msg.lon_int = int(target_lon * 1e7)
        pos_msg.alt = float(target_alt)
        pos_msg.yaw = body_yaw  # КОРПУС ЗАФИКСИРОВАН

        self.pos_pub.publish(pos_msg)

        # Б. УПРАВЛЕНИЕ ПОДВЕСОМ (GIMBAL)
        gimbal_pitch = 0.0
        gimbal_yaw = 0.0
        gimbal_roll = 0.0

        if self.look_at_target:
            look_lat, look_lon, _ = self.look_at_target

            # 1. Азимут на цель (Абсолютный, относительно Севера)
            bearing = self.calc_bearing(self.current_gps.latitude, self.current_gps.longitude, look_lat, look_lon)

            # 2. Угол места
            dist = self.calc_distance(look_lat, look_lon)
            if dist > 1.0:
                gimbal_pitch = -math.atan2(self.current_alt, dist)  # Вниз
            else:
                gimbal_pitch = -1.57  # Вертикально вниз

            # 3. Назначение углов
            # MAV_MOUNT_MODE_MAVLINK_TARGETING обычно работает в Earth Frame (абсолютные углы)
            # Если подвес в Body Frame, нужно вычесть body_yaw:
            # gimbal_yaw = bearing - self.orientation[2]

            # Предполагаем, что умный подвес (SIYI/Gremsy) понимает абсолютные углы
            gimbal_yaw = bearing

        else:
            # Если цели нет - смотрим прямо по курсу дрона (0.0) и в горизонт
            gimbal_pitch = 0.0
            gimbal_yaw = 0.0  # North

        # Публикация в MAVROS
        mount_msg = MountControl()
        mount_msg.header.stamp = self.get_clock().now().to_msg()
        mount_msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING

        # Конвертация в градусы
        mount_msg.pitch = math.degrees(gimbal_pitch)
        mount_msg.roll = 0.0  # Стабилизирован в горизонт
        mount_msg.yaw = math.degrees(gimbal_yaw)

        self.mount_pub.publish(mount_msg)

        # Сохраняем для статуса
        self.gimbal_rpy = [0.0, gimbal_pitch, gimbal_yaw]

    def publish_status(self):
        # 1. Основной статус
        msg = AgentStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.agent_id = self.agent_id
        msg.role = "SCOUT"
        msg.state = "ARMED" if self.mavros_state.armed else "DISARMED"
        msg.battery_volts = self.battery
        if self.current_gps:
            msg.lat, msg.lon, msg.alt = self.current_gps.latitude, self.current_gps.longitude, self.current_gps.altitude

        # ВАЖНО: Передаем ориентацию ПОДВЕСА, а не дрона!
        # YOLO должен знать, куда смотрит камера, а не куда летит дрон.
        # Если мы передадим углы дрона, расчет 3D будет ошибочным.
        # Поэтому мы передаем углы Гимбала.
        msg.roll = 0.0  # Камера всегда в горизонте
        msg.pitch = self.gimbal_rpy[1]
        msg.yaw = self.gimbal_rpy[2]

        self.status_pub.publish(msg)

        # 2. Дополнительный статус подвеса (для отладки)
        g_msg = GimbalState()
        g_msg.agent_id = self.agent_id
        g_msg.pitch = self.gimbal_rpy[1]
        g_msg.yaw = self.gimbal_rpy[2]
        g_msg.roll = 0.0
        g_msg.is_ready = True
        self.gimbal_pub.publish(g_msg)

    # --- MATH & UTIL ---
    def calc_bearing(self, lat1, lon1, lat2, lon2):
        d_lon = math.radians(lon2 - lon1)
        y = math.sin(d_lon) * math.cos(math.radians(lat2))
        x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
            math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(d_lon)
        return math.atan2(y, x)

    def calc_distance(self, lat2, lon2):
        R = 6378137.0
        d_lat = math.radians(lat2 - self.current_gps.latitude)
        d_lon = math.radians(lon2 - self.current_gps.longitude)
        a = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(self.current_gps.latitude)) * math.cos(
            math.radians(lat2)) * math.sin(d_lon / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def start_video_stream(self):
        uri = f"srt://{self.server_ip}:{self.srt_port}?streamid=publish:{self.agent_id}"
        if "aarch64" in os.uname().machine:
            cmd = ['gst-launch-1.0', 'v4l2src', f'device={self.device}', '!',
                   'video/x-raw,width=1280,height=720,framerate=30/1', '!',
                   'mpph264enc', 'gop=30', 'bps=2000000', '!', 'mpegtsmux', '!',
                   'srtserversink', f'uri={uri}', 'latency=20']
        else:
            cmd = ['gst-launch-1.0', 'v4l2src', f'device={self.device}', '!',
                   'videoconvert', '!', 'x264enc', 'tune=zerolatency', '!',
                   'mpegtsmux', '!', 'srtserversink', f'uri={uri}']
        self.stream_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def destroy_node(self):
        if self.stream_process: os.kill(self.stream_process.pid, signal.SIGKILL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisionNode())
    rclpy.shutdown()