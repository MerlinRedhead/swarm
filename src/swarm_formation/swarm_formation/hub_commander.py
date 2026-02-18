import rclpy
from rclpy.node import Node
from swarm_formation.msg import MissionState
from geographic_msgs.msg import GeoPoint

class HubCommander(Node):
    def __init__(self):
        super().__init__('hub_commander')
        
        # 1. Слушаем команду с НСУ (Клик по карте)
        # Топик: /ground/set_deploy_zone
        self.create_subscription(GeoPoint, '/ground/set_deploy_zone', self.ground_cmd_cb, 10)
        
        # 2. Вещаем приказ для роя
        self.swarm_pub = self.create_publisher(MissionState, '/swarm/global_mission', 10)
        
        # Храним текущую цель
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.mission_phase = "HOLD"
        
        # Таймер трансляции (5 Гц) - чтобы новые дроны сразу подхватывали задачу
        self.create_timer(0.2, self.broadcast_mission)
        
        self.get_logger().info("HUB COMMANDER: READY. Waiting for GCS input...")

    def ground_cmd_cb(self, msg):
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.mission_phase = "DEPLOY" # Начинаем движение
        self.get_logger().warn(f"NEW MISSION: Fly to {self.target_lat}, {self.target_lon}")

    def broadcast_mission(self):
        msg = MissionState()
        msg.center_lat = self.target_lat
        msg.center_lon = self.target_lon
        msg.center_alt = 50.0  # Высота эшелона перелета
        msg.phase = self.mission_phase
        msg.speed = 10.0       # 10 м/с
        
        self.swarm_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(HubCommander())
    rclpy.shutdown()
