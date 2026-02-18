import math
import numpy as np

EARTH_RADIUS = 6378137.0

def add_offset_to_gps(lat, lon, x_offset, y_offset):
    """Добавляет метры (East, North) к GPS координатам"""
    d_lat = y_offset / EARTH_RADIUS
    new_lat = lat + (d_lat * 180 / math.pi)
    d_lon = x_offset / (EARTH_RADIUS * math.cos(math.pi * lat / 180))
    new_lon = lon + (d_lon * 180 / math.pi)
    return new_lat, new_lon

def pixel_to_gps(cam_lat, cam_lon, cam_alt, pitch, yaw, u, v, img_w, img_h, hfov_deg):
    """
    Проекция пикселя на землю.
    pitch: наклон камеры (радианы, 0 - горизонт, -pi/2 - в пол)
    yaw: курс дрона + поворот подвеса
    u, v: координаты пикселя
    """
    # 1. Расчет углового отклонения пикселя от центра
    hfov = math.radians(hfov_deg)
    vfov = hfov * (img_h / img_w)
    
    alpha_u = ((u / img_w) - 0.5) * hfov
    alpha_v = ((v / img_h) - 0.5) * vfov
    
    # 2. Суммарные углы луча
    total_pitch = pitch + alpha_v
    total_yaw = yaw + alpha_u
    
    # 3. Расстояние до точки на земле (плоская земля)
    if total_pitch >= 0: return None # Луч выше горизонта
    dist_horizontal = cam_alt / math.tan(-total_pitch)
    
    # 4. Смещение в метрах
    dx = dist_horizontal * math.sin(total_yaw)
    dy = dist_horizontal * math.cos(total_yaw)
    
    return add_offset_to_gps(cam_lat, cam_lon, dx, dy)
