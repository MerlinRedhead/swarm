# ID цели (совпадает с ID в AttackCommand)
string target_id         # Пример: "T72_B3"
string class_name        # "tank", "infantry"
float64 lat              # Координаты цели (фильтрованные)
float64 lon
float64 vel_x            # Вектор скорости цели (м/с)
float64 vel_y
bool is_locked           # True, если цель подтверждена

# ID ударника, который должен получить эту коррекцию
string striker_id

# Уточненные координаты
float64 lat
float64 lon

# Вектор скорости цели (для упреждения), м/с
float32 velocity_north
float32 velocity_east

# Флаг захвата: true = цель надежно удерживается, false = прогноз/потеря
bool is_locked

# ВЕКТОР СКОРОСТИ ЦЕЛИ (м/с) - Для PN-наведения
float32 vel_n           # Скорость на Север
float32 vel_e           # Скорость на Восток

# КАЧЕСТВО ДАННЫХ
float32 confidence      # Вероятность (0.0 - 1.0)
float64 last_seen       # Timestamp (чтобы удалять старые цели)
