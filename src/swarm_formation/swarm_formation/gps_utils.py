import math

EARTH_RADIUS = 6378137.0

def add_offset_to_gps(lat, lon, x_offset, y_offset):
    """Смещение в метрах (x - восток, y - север)"""
    d_lat = y_offset / EARTH_RADIUS
    new_lat = lat + (d_lat * 180 / math.pi)
    d_lon = x_offset / (EARTH_RADIUS * math.cos(math.pi * lat / 180))
    new_lon = lon + (d_lon * 180 / math.pi)
    return new_lat, new_lon

def pixel_to_gps(cam_lat, cam_lon, cam_alt, pitch_rad, yaw_rad, u, v, img_w, img_h, hfov_deg):
    """Проекция пикселя на плоскость земли"""
    hfov = math.radians(hfov_deg)
    vfov = hfov * (img_h / img_w)
    
    # Углы отклонения от центральной оси камеры
    alpha_u = ((u / img_w) - 0.5) * hfov
    alpha_v = ((v / img_h) - 0.5) * vfov
    
    # Итоговый угол луча с учётом тангажа и рыскания дрона
    target_pitch = pitch_rad + alpha_v
    target_yaw = yaw_rad + alpha_u
    
    if target_pitch >= 0: return None # Смотрим выше горизонта
    
    dist_horiz = cam_alt / math.tan(-target_pitch)
    dx = dist_horiz * math.sin(target_yaw)
    dy = dist_horiz * math.cos(target_yaw)
    
    return add_offset_to_gps(cam_lat, cam_lon, dx, dy)
