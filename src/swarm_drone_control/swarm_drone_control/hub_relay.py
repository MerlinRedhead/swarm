#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import os
import yaml
import asyncio
from ament_index_python.packages import get_package_share_directory

# ROS Messages
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from swarm_interfaces.msg import SwarmTask, AgentStatus, TargetRefined, SwarmCommand

# --- КОНСТАНТЫ ---
EARTH_RADIUS = 6378137.0
SWARM_ALTITUDE_DEFAULT = 30.0  # Высота полета по умолчанию


class HubRelayNode(Node):
    def __init__(self):
        super().__init__('hub_relay')

        # --- ПАРАМЕТРЫ ЗАПУСКА ---
        self.declare_parameter('agent_id', 'hub')
        self.agent_id = self.get_parameter('agent_id').value

        # Загрузка конфигурации строя
        self.formation_config = self.load_config()

        # --- СОСТОЯНИЕ ---
        self.agents = {}  # {agent_id: AgentStatus}
        self.targets = {}  # {target_uuid: TargetRefined}
        self.current_gps = None
        self.mavros_state = State()
        self.takeoff_in_progress = False

        # --- ПОДПИСКИ (ВХОДЯЩИЕ ДАННЫЕ) ---
        # 1. Своя телеметрия (Хаб сам летает!)
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.gps_cb, 10)
        self.create_subscription(State, 'mavros/state', self.state_cb, 10)

        # 2. Статусы подчиненных дронов
        self.create_subscription(AgentStatus, '/swarm/agent_status', self.agent_status_cb, 10)

        # 3. Цели от YOLO (Сервер)
        self.create_subscription(TargetRefined, '/swarm/target_global', self.target_cb, 10)

        # 4. Глобальные команды от НСУ (Карта/Оператор)
        self.create_subscription(SwarmCommand, '/swarm/global_command', self.nsu_command_cb, 10)

        # --- ПУБЛИКАЦИИ (ИСХОДЯЩИЕ ДАННЫЕ) ---
        # 1. Раздача задач рою
        self.task_pub = self.create_publisher(SwarmTask, '/swarm/task_assignment', 10)

        # 2. Управление полетом Хаба (MAVROS)
        self.global_pos_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_position/global', 10)

        # 3. Логи для НСУ
        self.log_pub = self.create_publisher(String, '/swarm/sys_log', 10)

        # --- СЕРВИСЫ ---
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')

        self.get_logger().info(f"HUB COMMANDER [{self.agent_id}] ONLINE. Swarm Formation Loaded.")

    def load_config(self):
        """Загрузка матрицы смещений из config.yaml"""
        # Путь: ищем файл config.yaml рядом со скриптом или в share
        # Для отладки предполагаем, что он в той же папке или передан как параметр
        cfg_path = os.path.join(os.getcwd(), 'src/swarm_drone_control/swarm_drone_control/config.yaml')

        # Если запускаешь через colcon, лучше использовать get_package_share_directory
        # cfg_path = os.path.join(get_package_share_directory('swarm_drone_control'), 'config', 'config.yaml')

        if not os.path.exists(cfg_path):
            self.get_logger().warn(f"Config not found at {cfg_path}. Using Defaults.")
            return {}

        try:
            with open(cfg_path, 'r') as f:
                data = yaml.safe_load(f)
                self.get_logger().info(f"Loaded Config: {data}")
                return data.get('formation', {})
        except Exception as e:
            self.get_logger().error(f"Config Load Error: {e}")
            return {}

    # --- CALLBACKS ---
    def gps_cb(self, msg):
        self.current_gps = msg

    def state_cb(self, msg):
        self.mavros_state = msg

    def agent_status_cb(self, msg):
        self.agents[msg.agent_id] = msg

    def target_cb(self, msg):
        """
        YOLO обнаружил/обновил цель.
        Хаб сохраняет её координаты. Атака начнется ТОЛЬКО по приказу НСУ.
        """
        self.targets[msg.target_id] = msg
        self.log(f"Target tracked: {msg.target_id}")

    def nsu_command_cb(self, msg):
        """
        ГЛАВНЫЙ МОЗГ: Обработка команд с карты НСУ.
        """
        cmd_type = msg.command_type  # DEPLOY, ATTACK, TAKEOFF, RTB

        self.log(f"NSU COMMAND: {cmd_type} -> {msg.recipient_id}")

        if cmd_type == "DEPLOY":
            center_lat = msg.param_1
            center_lon = msg.param_2
            self.move_swarm_formation(center_lat, center_lon)

        elif cmd_type == "ATTACK":
            target_uuid = msg.target_uuid
            self.coordinate_attack(target_uuid)

        elif cmd_type == "TAKEOFF":
            if not self.takeoff_in_progress:
                alt = msg.param_3 if msg.param_3 > 0 else SWARM_ALTITUDE_DEFAULT
                # Запускаем асинхронную задачу в фоне, чтобы не блокировать ROS Executor
                asyncio.create_task(self.swarm_takeoff_sequence(alt))
            else:
                self.log("Takeoff already in progress. Ignoring command.")

        elif cmd_type == "RTB":
            self.return_to_base()

    # --- ЛОГИКА СТРОЯ (FORMATION FLIGHT) ---
    def move_swarm_formation(self, center_lat, center_lon):
        """Рассчитывает целевую точку для КАЖДОГО дрона на основе config.yaml"""
        if not self.formation_config:
            self.log("ERROR: Formation config missing!")
            return

        self.log(f"MOVING SWARM TO: {center_lat:.6f}, {center_lon:.6f}")

        for role_id, offsets in self.formation_config.items():
            off_x = offsets.get('x', 0.0)
            off_y = offsets.get('y', 0.0)
            off_z = offsets.get('z', SWARM_ALTITUDE_DEFAULT)

            d_lat = (off_x / EARTH_RADIUS) * (180.0 / math.pi)
            d_lon = (off_y / EARTH_RADIUS) * (180.0 / math.pi) / math.cos(math.radians(center_lat))

            target_lat = center_lat + d_lat
            target_lon = center_lon + d_lon

            if role_id == self.agent_id:
                # Хаб летит сам
                self.fly_me_to(target_lat, target_lon, off_z)
            else:
                # Хаб командует другим
                self.send_task(role_id, "SCOUT_POINT", target_lat, target_lon, off_z)

    # --- ЛОГИКА АТАКИ (HUNTER-KILLER) ---
    def coordinate_attack(self, target_uuid):
        """Связка: Ударник атакует, Разведчик (Споттер) корректирует."""
        if target_uuid not in self.targets:
            self.log(f"ERROR: Target {target_uuid} lost or invalid ID")
            return

        target = self.targets[target_uuid]

        striker = self.find_best_agent("striker", "IDLE", target.lat, target.lon)
        spotter_hint = target_uuid.split('_')[0] + "_" + target_uuid.split('_')[1]
        spotter = spotter_hint if spotter_hint in self.agents else self.find_best_agent("vision", "ACTIVE", target.lat, target.lon)

        if not striker:
            self.log("NO STRIKERS AVAILABLE!")
            return

        self.log(f"ENGAGING {target_uuid}: Striker={striker}, Spotter={spotter}")

        self.send_task(striker, "STRIKE", target.lat, target.lon, 0.0, target_uuid)

        if spotter:
            self.send_task(spotter, "SCOUT_POINT", target.lat, target.lon, SWARM_ALTITUDE_DEFAULT, target_uuid)

    # --- НИЗКОУРОВНЕВОЕ УПРАВЛЕНИЕ ---
    def fly_me_to(self, lat, lon, alt):
        """Полет самого Хаба"""
        # Если находимся на земле - игнорируем, либо заставляем взлететь
        if not self.mavros_state.armed:
           self.log("Cannot fly_to: HUB is not armed!")
           return
           
        if self.mavros_state.mode != "GUIDED":
            asyncio.create_task(self.set_mode("GUIDED"))

        msg = PositionTarget()
        # CRITICAL FIX: FRAME_GLOBAL_REL_ALT (6) instead of FRAME_GLOBAL_INT (5). 
        # 5 expects Altitude Above Sea Level. 6 expects Altitude above Home.
        msg.coordinate_frame = PositionTarget.FRAME_GLOBAL_REL_ALT
        
        # CRITICAL FIX: type_mask 0x0DF8 (ignore velocity/accel/yaw_rate, keeping pos and yaw)
        # Previously ignored yaw, which means drone would fly sideways.
        msg.type_mask = 0b0000110111111000 
        
        msg.lat_int = int(lat * 1e7)
        msg.lon_int = int(lon * 1e7)
        msg.alt = float(alt)
        
        # Opcional: Calculate heading (yaw) towards destination
        if self.current_gps:
             dy = (lon - self.current_gps.longitude) * math.cos(math.radians(self.current_gps.latitude))
             dx = lat - self.current_gps.latitude
             msg.yaw = math.atan2(dy, dx)
        else:
             msg.yaw = 0.0

        self.global_pos_pub.publish(msg)

    async def swarm_takeoff_sequence(self, alt):
        """Безопасная асинхронная процедура взлета (Без time.sleep()!)."""
        self.takeoff_in_progress = True
        self.log(f"SWARM TAKEOFF SEQUENCE. Alt: {alt}m")

        # 1. Установка режима GUIDED для Ардупилота
        success = await self.set_mode("GUIDED")
        if not success:
            self.log("Takeoff Aborted: Failed to set GUIDED mode.")
            self.takeoff_in_progress = False
            return

        # 2. Попытка Arming
        success = await self.arm()
        if not success:
            self.log("Takeoff Aborted: ARMING REJECTED by ArduPilot.")
            self.takeoff_in_progress = False
            return
            
        # Даем Ардупилоту время осознать, что он вооружен (асинхронный слип)
        await asyncio.sleep(2.0)
        
        # Двойная проверка, что моторы крутятся
        if not self.mavros_state.armed:
            self.log("Takeoff Aborted: Drone disarmed automatically.")
            self.takeoff_in_progress = False
            return

        # 3. Команда Takeoff
        if self.takeoff_client.service_is_ready():
            req = CommandTOL.Request()
            req.altitude = float(alt)
            req.latitude = float('nan')
            req.longitude = float('nan')
            
            self.log("Commanding HUB takeoff...")
            try:
                # Асинхронный вызов сервиса ROS2
                response = await self.takeoff_client.call_async(req)
                if response.success:
                    self.log("HUB TAKEOFF SUCCESSFUL")
                else:
                    self.log("HUB TAKEOFF FAILED/REJECTED")
            except Exception as e:
                self.log(f"Takeoff Service Exception: {e}")
        else:
            self.log("Takeoff service not available!")

        # 4. Взлет остальных (раздаем задачи рою)
        if self.current_gps:
            self.move_swarm_formation(self.current_gps.latitude, self.current_gps.longitude)
            
        self.takeoff_in_progress = False

    def return_to_base(self):
        self.log("SWARM RTB INITIATED")
        asyncio.create_task(self.set_mode("RTL"))  # Хаб домой

    # --- ВСПОМОГАТЕЛЬНЫЕ ---
    def send_task(self, agent_id, type_str, lat, lon, alt, target_uuid=""):
        msg = SwarmTask()
        msg.task_id = f"CMD_{int(time.time())}"
        msg.assigned_agent_id = agent_id
        msg.task_type = type_str
        msg.lat = lat
        msg.lon = lon
        msg.alt = alt
        msg.target_uuid = target_uuid
        self.task_pub.publish(msg)

    def find_best_agent(self, role, status_req, lat, lon):
        best = None
        min_dist = float('inf')
        for aid, info in self.agents.items():
            if role in aid and (status_req in info.role or status_req == "ANY"):
                dist = (info.lat - lat) ** 2 + (info.lon - lon) ** 2
                if dist < min_dist:
                    min_dist = dist
                    best = aid
        return best

    async def set_mode(self, mode):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request(custom_mode=mode)
            try:
                response = await self.set_mode_client.call_async(req)
                return response.mode_sent
            except Exception as e:
                self.log(f"SetMode exception: {e}")
                return False
        return False

    async def arm(self):
        if self.arming_client.service_is_ready():
            req = CommandBool.Request(value=True)
            try:
                response = await self.arming_client.call_async(req)
                return response.success
            except Exception as e:
                self.log(f"Arming exception: {e}")
                return False
        return False

    def log(self, text):
        self.get_logger().info(text)
        self.log_pub.publish(String(data=text))


def main_loop(args=None):
    rclpy.init(args=args)
    node = HubRelayNode()
    
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