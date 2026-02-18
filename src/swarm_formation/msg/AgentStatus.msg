# Уникальный ID дрона (например: "hub_leader", "vision_1", "striker_3")
string agent_id
string role              # "IDLE", "SCOUT", "ATTACK"
string state             # "ARMED", "DISARMED"
float32 battery_volts
float64 lat              # Текущая широта
float64 lon              # Текущая долгота
float32 alt              # Высота AMSL

# ОРИЕНТАЦИЯ (Радианы) - Критично для 3D проекции координат
float32 roll
float32 pitch
float32 yaw

# СВЯЗЬ (Для логики Relay)
float32 wifi_rssi       # Уровень сигнала (если < -80, реле набирает высоту)

std_msgs/Header header  # Содержит stamp (время) и frame_id

# Текущее состояние: "BOOT", "IDLE", "TAKEOFF", "FORMATION", "ATTACK", "RTL", "LANDED"
string state

# Текущие координаты
float64 latitude
float64 longitude
float64 altitude
float64 yaw
float64 roll
float64 pitch

# Заряд батареи (0.0 - 1.0)
float32 battery_level

# Статус GPS (0 = No GPS, 3 = 3D Fix, 4 = RTK Float, 5 = RTK Fixed)
uint8 gps_status
