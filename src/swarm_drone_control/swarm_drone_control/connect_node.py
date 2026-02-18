import rclpy
import time
import math
from rclpy.node import Node
from .gps_utils import add_offset_to_gps
from swarm_formation.msg import MissionState, AgentStatus
from mavros_msgs.msg import GlobalPositionTarget, State
from sensor_msgs.msg import NavSatFix, Imu

class ConnectNode(Node):
    def __init__(self):
        super().__init__('connect_node')
        
        # Вставь это в начало __init__ любой ноды
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('alt_step', 0.0)

        # Теперь значения подтянутся автоматически из YAML
        self.ox = self.get_parameter('offset_x').value
        self.oy = self.get_parameter('offset_y').value
        self.alt_step = self.get_parameter('alt_step').value

        self.get_logger().info(f"LOADED OFFSETS: X={self.ox}, Y={self.oy}, Alt={self.alt_step}")

        self.last_hub_msg = time.time()
        self.state = "IDLE"
        self.curr_gps = None

        self.create_subscription(MissionState, '/swarm/global_mission', self.hub_cb, 10)
        self.create_subscription(Imu, 'mavros/imu/data', self.imu_cb, 10)
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.gps_cb, 10)
        self.status_pub = self.create_publisher(AgentStatus, '/swarm/status_all', 10)
        self.setpoint_pub = self.create_publisher(GlobalPositionTarget, 'mavros/setpoint_position/global', 10)

        self.create_timer(1.0, self.watchdog)

    def gps_cb(self, msg): self.curr_gps = msg

    def hub_cb(self, msg):
        self.last_hub_msg = time.time()
        self.state = "RELAYING"
        
        # Ретрансляторы обычно висят выше (напр. +15 метров к базе)
        t_lat, t_lon = self.add_offset(msg.center_lat, msg.center_lon, self.ox, self.oy)
        self.send_cmd(t_lat, t_lon, msg.center_alt + 15.0)

    def watchdog(self):
        """Логика поднятия выше при потере связи"""
        if time.time() - self.last_hub_msg > 5.0 and self.state != "IDLE":
            self.state = "SIGNAL_LOST"
            self.get_logger().error("LOST HUB! CLIMBING...")
            if self.curr_gps:
                self.send_cmd(self.curr_gps.latitude, self.curr_gps.longitude, self.safe_alt)
        
        # Отправка статуса
        s = AgentStatus(agent_id=self.id, state=self.state)
        self.status_pub.publish(s)

    def send_cmd(self, lat, lon, alt):
        target = GlobalPositionTarget()
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = 0b0000111111111000
        target.latitude, target.longitude, target.altitude = lat, lon, alt
        self.setpoint_pub.publish(target)

    def add_offset(self, lat, lon, dx, dy):
        R = 6378137.0
        n_lat = lat + (dy / R) * (180 / math.pi)
        n_lon = lon + (dx / (R * math.cos(math.pi * lat / 180))) * (180 / math.pi)
        return n_lat, n_lon
