# Тип команды: "ROI_LOCK", "ROI_TRACK", "ROI_RESET", "LAND"
string command_type

# ID целевого агента (или "all")
string target_id

# Параметры команды (координаты точки интереса)
float64 lat
float64 lon
float64 alt

string target_uuid      # ID цели ("vision_1_42")
float32 formation_x     # Смещение в строю (если команда FORMATION)
float32 formation_y
