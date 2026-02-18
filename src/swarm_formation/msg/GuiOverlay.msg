# От какого дрона пришло видео
string source_agent_id
string class_name
float32 x                # Нормированные координаты (0.0 - 1.0)
float32 y
float32 w
float32 h

# ID распознанного объекта
string object_id

# Класс объекта (например "BMP-2")
string class_name

# Координаты Bounding Box на экране (пиксели)
float32 x                # Нормированные координаты (0.0 - 1.0)
float32 y
float32 w
float32 h

# Расчетные GPS координаты объекта
float64 lat
float64 lon

string track_id         # ID трекера (BoTSORT), например "42"
string global_uuid      # Глобальный ID: "vision_1_42"
bool is_locked          # True = Цель уже захвачена (рисуем красным)