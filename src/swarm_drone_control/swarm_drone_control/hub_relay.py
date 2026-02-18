#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import os
import yaml
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
        # self.log(f"Target tracked: {msg.target_id}")

    def nsu_command_cb(self, msg):
        """
        ГЛАВНЫЙ МОЗГ: Обработка команд с карты НСУ.
        """
        cmd_type = msg.command_type  # DEPLOY, ATTACK, TAKEOFF, RTB

        self.log(f"NSU COMMAND: {cmd_type} -> {msg.recipient_id}")

        if cmd_type == "DEPLOY":
            # Перелет роя в зону интереса (В СТРОЮ)
            center_lat = msg.param_1
            center_lon = msg.param_2
            self.move_swarm_formation(center_lat, center_lon)

        elif cmd_type == "ATTACK":
            # Атака конкретной цели
            target_uuid = msg.target_uuid
            self.coordinate_attack(target_uuid)

        elif cmd_type == "TAKEOFF":
            # Взлет всего роя
            alt = msg.param_3 if msg.param_3 > 0 else SWARM_ALTITUDE_DEFAULT
            self.swarm_takeoff(alt)

        elif cmd_type == "RTB":
            # Возврат на базу
            self.return_to_base()

    # --- ЛОГИКА СТРОЯ (FORMATION FLIGHT) ---
    def move_swarm_formation(self, center_lat, center_lon):
        """
        Рассчитывает целевую точку для КАЖДОГО дрона на основе config.yaml
        и отправляет их всех (включая себя) в полет.
        """
        if not self.formation_config:
            self.log("ERROR: Formation config missing!")
            return

        self.log(f"MOVING SWARM TO: {center_lat:.6f}, {center_lon:.6f}")

        # Проходим по всем агентам в конфиге (hub, scout_1, striker_1...)
        for role_id, offsets in self.formation_config.items():

            # 1. Расчет смещения GPS
            # Offsets: x (North), y (East) в метрах
            off_x = offsets.get('x', 0.0)
            off_y = offsets.get('y', 0.0)
            off_z = offsets.get('z', SWARM_ALTITUDE_DEFAULT)

            d_lat = (off_x / EARTH_RADIUS) * (180.0 / math.pi)
            d_lon = (off_y / EARTH_RADIUS) * (180.0 / math.pi) / math.cos(math.radians(center_lat))

            target_lat = center_lat + d_lat
            target_lon = center_lon + d_lon

            # 2. Раздача команд
            if role_id == self.agent_id:
                # Хаб летит сам (через MAVROS)
                self.fly_me_to(target_lat, target_lon, off_z)
            else:
                # Хаб командует другим (через SwarmTask)
                # Тип задачи SCOUT_POINT означает "лететь и ждать/наблюдать"
                self.send_task(role_id, "SCOUT_POINT", target_lat, target_lon, off_z)

    # --- ЛОГИКА АТАКИ (HUNTER-KILLER) ---
    def coordinate_attack(self, target_uuid):
        """
        Связка: Ударник атакует, Разведчик (Споттер) корректирует.
        """
        if target_uuid not in self.targets:
            self.log(f"ERROR: Target {target_uuid} lost or invalid ID")
            return

        target = self.targets[target_uuid]

        # 1. Выбор Ударника (ближайший свободный)
        striker = self.find_best_agent("striker", "IDLE", target.lat, target.lon)

        # 2. Выбор Споттера (ближайший, который видит цель)
        # В идеале берем того, кто эту цель и заметил (target_uuid содержит ID дрона)
        spotter_hint = target_uuid.split('_')[0] + "_" + target_uuid.split('_')[1]  # vision_1
        spotter = spotter_hint if spotter_hint in self.agents else self.find_best_agent("vision", "ACTIVE", target.lat,
                                                                                        target.lon)

        if not striker:
            self.log("NO STRIKERS AVAILABLE!")
            return

        self.log(f"ENGAGING {target_uuid}: Striker={striker}, Spotter={spotter}")

        # 3. Приказ Ударнику: STRIKE (Включает PN-наведение)
        # Передаем UUID цели, чтобы ударник знал, какой бокс трекать
        self.send_task(striker, "STRIKE", target.lat, target.lon, 0.0, target_uuid)

        # 4. Приказ Споттеру: TRACK (Повернуть камеру на цель)
        if spotter:
            # Споттер должен висеть на месте (или кружить) и держать цель в фокусе
            # Мы отправляем ему координаты цели, vision_node повернет гимбал
            self.send_task(spotter, "SCOUT_POINT", target.lat, target.lon, SWARM_ALTITUDE_DEFAULT, target_uuid)

    # --- НИЗКОУРОВНЕВОЕ УПРАВЛЕНИЕ ---
    def fly_me_to(self, lat, lon, alt):
        """Полет самого Хаба"""
        if self.mavros_state.mode != "GUIDED":
            self.set_mode("GUIDED")

        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_GLOBAL_INT
        msg.type_mask = 0b0000111111111000  # Ignore Vel/Accel, use Pos
        msg.lat_int = int(lat * 1e7)
        msg.lon_int = int(lon * 1e7)
        msg.alt = float(alt)
        # Устанавливаем Yaw на цель (опционально) или по курсу
        self.global_pos_pub.publish(msg)

    def swarm_takeoff(self, alt):
        """Команда на взлет всем (включая Хаб)"""
        self.log(f"SWARM TAKEOFF SEQUENCE. Alt: {alt}m")

        # 1. Взлет Хаба
        self.set_mode("GUIDED")
        self.arm()
        time.sleep(2)

        # Используем сервис Takeoff для себя
        if self.takeoff_client.service_is_ready():
            req = CommandTOL.Request()
            req.altitude = float(alt)
            req.latitude = float('nan')
            req.longitude = float('nan')
            self.takeoff_client.call_async(req)

        # 2. Взлет остальных (через Task)
        # В данном упрощении мы шлем SCOUT_POINT на текущие координаты + высота
        # В идеале нужен отдельный тип команды TAKEOFF в SwarmTask
        # Но пока используем логику "лети в строй"
        if self.current_gps:
            self.move_swarm_formation(self.current_gps.latitude, self.current_gps.longitude)

    def return_to_base(self):
        self.log("SWARM RTB INITIATED")
        self.set_mode("RTL")  # Хаб домой
        # Остальным тоже командуем RTL (если реализовано в дронах)
        # Или шлем их в точку сбора (Launch Point)
        pass

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
            if role in aid and (status_req in info.role or status_req == "ANY"):  # info.role из AgentStatus
                dist = (info.lat - lat) ** 2 + (info.lon - lon) ** 2
                if dist < min_dist:
                    min_dist = dist
                    best = aid
        return best

    def set_mode(self, mode):
        if self.set_mode_client.service_is_ready():
            self.set_mode_client.call_async(SetMode.Request(custom_mode=mode))

    def arm(self):
        if self.arming_client.service_is_ready():
            self.arming_client.call_async(CommandBool.Request(value=True))

    def log(self, text):
        self.get_logger().info(text)
        self.log_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HubRelayNode())
    rclpy.shutdown()