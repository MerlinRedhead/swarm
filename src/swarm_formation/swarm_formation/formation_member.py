import rclpy
from rclpy.node import Node
from swarm_formation.msg import MissionState
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
# Импортируем нашу математику (убедитесь, что файл gps_utils.py виден python)
from .gps_utils import add_offset_to_gps 

class FormationMember(Node):
    def __init__(self):
        super().__init__('formation_member')
        
        # --- ПАРАМЕТРЫ (Задаются в launch файле для каждого дрона) ---
        self.declare_parameter('offset_x', 0.0) # Метры на Восток
        self.declare_parameter('offset_y', 0.0) # Метры на Север
        self.declare_parameter('my_id', 'drone_generic')
        
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.my_id = self.get_parameter('my_id').value
        
        # --- ПОДПИСКИ ---
        # Слушаем Хаб
        self.create_subscription(MissionState, '/swarm/global_mission', self.mission_cb, 10)
        # Слушаем MAVROS статус
        self.mavros_state = State()
        self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        
        # --- ПАБЛИШЕРЫ ---
        # Отправка точки в автопилот
        self.target_pub = self.create_publisher(GlobalPositionTarget, 'mavros/setpoint_position/global', 10)
        
        # --- КЛИЕНТЫ ---
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')

        self.get_logger().info(f"{self.my_id} READY. Offset: [{self.offset_x}, {self.offset_y}]")

    def state_cb(self, msg):
        self.mavros_state = msg

    def mission_cb(self, msg):
        """Логика реакции на приказ Хаба"""
        if msg.phase == "HOLD":
            return # Просто висим или ждем
            
        if msg.phase == "DEPLOY":
            # 1. Считаем СВОЮ точку назначения (Центр + Моё смещение)
            my_lat, my_lon = add_offset_to_gps(
                msg.center_lat, 
                msg.center_lon, 
                self.offset_x, 
                self.offset_y
            )
            
            # 2. Формируем команду для Pixhawk
            target = GlobalPositionTarget()
            target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            target.type_mask = 0b0000111111111000 
            target.latitude = my_lat
            target.longitude = my_lon
            target.altitude = msg.center_alt
            
            # 3. Отправляем (постоянно обновляем точку)
            self.target_pub.publish(target)
            
            # 4. Проверяем режим полета (Авто-взлет и полет)
            self.ensure_flying_mode()

    def ensure_flying_mode(self):
        """Переводит дрон в GUIDED и Армит, если он еще на земле"""
        if self.mavros_state.mode != "GUIDED":
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.set_mode_client.call_async(req)
        
        if not self.mavros_state.armed:
            self.get_logger().info("DEPLOY COMMAND RECEIVED: TAKING OFF!")
            arm_req = CommandBool.Request()
            arm_req.value = True
            self.arming_client.call_async(arm_req)

def main():
    rclpy.init()
    node = FormationMember()
    rclpy.spin(node)
    rclpy.shutdown()
