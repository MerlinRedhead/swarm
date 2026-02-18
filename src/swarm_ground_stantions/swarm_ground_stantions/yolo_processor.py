#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import threading
import collections
import time
import subprocess
import os
from ultralytics import YOLO

# Импорт сообщений
from swarm_interfaces.msg import TargetRefined, GuiOverlay, AgentStatus
from std_msgs.msg import Header

# --- КОНСТАНТЫ ---
EARTH_RADIUS = 6378137.0
# Классы целей (COCO): 0=person, 2=car, 3=motorcycle, 5=bus, 7=truck
TARGET_CLASSES = [0, 1, 2, 3, 5, 7]


class TelemetryBuffer:
    """Буфер для синхронизации времени (хранит историю 2-3 секунды)"""

    def __init__(self, maxlen=100):
        self.buffer = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add(self, msg):
        with self.lock:
            # Время сообщения в секундах
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.buffer.append((t, msg))

    def get_interpolated_state(self, query_time):
        with self.lock:
            if len(self.buffer) < 2: return None

            # Поиск двух ближайших кадров телеметрии
            prev, nxt = None, None
            for t, msg in self.buffer:
                if t <= query_time:
                    prev = (t, msg)
                else:
                    nxt = (t, msg)
                    break

            if prev and nxt:
                t1, m1 = prev
                t2, m2 = nxt
                dt = t2 - t1
                if dt <= 0.0001: return m1

                ratio = (query_time - t1) / dt

                # Интерполяция
                res = AgentStatus()
                res.lat = m1.lat + (m2.lat - m1.lat) * ratio
                res.lon = m1.lon + (m2.lon - m1.lon) * ratio
                res.alt = m1.alt + (m2.alt - m1.alt) * ratio
                res.roll = m1.roll + (m2.roll - m1.roll) * ratio
                res.pitch = m1.pitch + (m2.pitch - m1.pitch) * ratio
                res.yaw = self._interp_angle(m1.yaw, m2.yaw, ratio)
                return res

            # Если точного совпадения нет, возвращаем ближайший (fallback)
            return prev[1] if prev else (nxt[1] if nxt else None)

    def _interp_angle(self, a1, a2, ratio):
        diff = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
        return (a1 + diff * ratio + math.pi) % (2 * math.pi) - math.pi


class StreamProcessor:
    """
    Обработчик видеопотока с трекингом и ретрансляцией.
    """

    def __init__(self, agent_id, model, ros_node, telem_buffer):
        self.agent_id = agent_id
        self.model = model
        self.node = ros_node
        self.telem_buffer = telem_buffer
        self.running = True

        # Настройки потоков
        # ВАЖНО: MediaMTX должен быть запущен на этом же сервере или указать IP
        self.media_server = "127.0.0.1"

        # ВХОД: Сырой поток от дрона
        self.input_url = f"srt://{self.media_server}:8890?streamid=read:{agent_id}"

        # ВЫХОД: Обработанный поток (с квадратиками) для НСУ
        self.output_url = f"srt://{self.media_server}:8890?streamid=publish:{agent_id}_processed"

        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        self.node.get_logger().info(f"[{self.agent_id}] PIPELINE START: {self.input_url}")

        cap = cv2.VideoCapture(self.input_url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Минимальный буфер чтения

        # Ожидание подключения дрона
        retry_count = 0
        while not cap.isOpened() and self.running:
            if retry_count % 5 == 0:
                self.node.get_logger().info(f"[{self.agent_id}] Waiting for stream...")
            time.sleep(1)
            cap.open(self.input_url)
            retry_count += 1

        # --- FFMPEG OUTPUT PIPELINE ---
        # Используем NVENC (NVIDIA) для кодирования на RTX 5080
        # Это критически важно, чтобы не грузить CPU
        ffmpeg_cmd = [
            'ffmpeg',
            '-y',  # Перезаписать файл если есть
            '-f', 'rawvideo',  # Формат входных данных (от OpenCV)
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',  # Формат цвета OpenCV
            '-s', '1280x720',  # Размер кадра (должен совпадать с камерой)
            '-r', '30',  # FPS
            '-i', '-',  # Читаем из stdin (из Python)

            # Настройки кодирования (NVIDIA RTX)
            '-c:v', 'h264_nvenc',  # Аппаратный энкодер
            '-preset', 'p1',  # p1 = Fastest, p7 = Best Quality
            '-tune', 'll',  # Low Latency
            '-rc', 'cbr',  # Constant Bitrate (стабильность для сети)
            '-b:v', '2M',  # Битрейт 2 Мбит/с

            # Выходной формат (SRT MPEG-TS)
            '-f', 'mpegts',
            self.output_url
        ]

        # Fallback для тестов без NVIDIA (если код запустят на ноуте)
        # ffmpeg_cmd = ['ffmpeg', ..., '-c:v', 'libx264', '-preset', 'ultrafast', ...]

        process = None
        try:
            process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
        except Exception as e:
            self.node.get_logger().error(f"FFMPEG START FAILED: {e}")
            return

        while self.running and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                # Потеря видео - ждем
                time.sleep(0.01)
                continue

            # Оценка времени съемки кадра (Latency ~200ms)
            capture_time = self.node.get_clock().now().nanoseconds * 1e-9 - 0.2

            # --- YOLO TRACKING ---
            # persist=True сохраняет ID треков между кадрами
            results = self.model.track(frame, persist=True, verbose=False, conf=0.45, imgsz=1280,
                                       tracker="botsort.yaml")

            # Получаем состояние дрона на момент съемки
            drone_state = self.telem_buffer.get_interpolated_state(capture_time)
            h, w = frame.shape[:2]

            # Обработка результатов
            for r in results:
                if r.boxes and r.boxes.id is not None:

                    boxes_xyxy = r.boxes.xyxy.cpu().numpy()
                    track_ids = r.boxes.id.int().cpu().numpy()
                    cls_ids = r.boxes.cls.int().cpu().numpy()

                    for box, track_id, cls_id in zip(boxes_xyxy, track_ids, cls_ids):
                        cls_name = self.model.names[cls_id]

                        # Координаты
                        x1, y1, x2, y2 = box
                        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

                        # Глобальный ID: vision_1_42
                        global_target_id = f"{self.agent_id}_{track_id}"

                        # 1. ОТПРАВКА GUI OVERLAY (Для Striker Handover)
                        # Это нужно, чтобы Ударник мог подхватить цель
                        overlay_msg = GuiOverlay()
                        overlay_msg.source_agent_id = self.agent_id
                        overlay_msg.track_id = str(track_id)
                        overlay_msg.class_name = cls_name
                        overlay_msg.x = float(cx / w)
                        overlay_msg.y = float(cy / h)
                        overlay_msg.w = float((x2 - x1) / w)
                        overlay_msg.h = float((y2 - y1) / h)
                        self.node.overlay_pub.publish(overlay_msg)

                        # 2. РАСЧЕТ GPS И ОТПРАВКА НА ХАБ
                        gps_info = ""
                        if drone_state:
                            lat, lon = self.calculate_precise_gps(drone_state, cx, cy, w, h)
                            if lat:
                                tgt_msg = TargetRefined()
                                tgt_msg.target_id = global_target_id
                                tgt_msg.class_name = cls_name
                                tgt_msg.lat = lat
                                tgt_msg.lon = lon
                                # is_locked ставится Хабом, когда оператор кликнет
                                self.node.target_pub.publish(tgt_msg)
                                gps_info = f"GPS"

                        # 3. РИСОВАНИЕ ГРАФИКИ (Для оператора)
                        color = (0, 255, 0)  # Зеленый
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

                        label = f"ID:{track_id} {cls_name} {gps_info}"
                        cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # --- ОТПРАВКА ОБРАБОТАННОГО КАДРА В СЕТЬ ---
            try:
                process.stdin.write(frame.tobytes())
            except BrokenPipeError:
                self.node.get_logger().error(f"[{self.agent_id}] FFmpeg Pipe Broken!")
                # Перезапуск FFmpeg можно реализовать здесь, но проще перезапустить ноду
                break

        # Очистка ресурсов
        cap.release()
        if process:
            process.stdin.close()
            process.wait()

    def calculate_precise_gps(self, st, u, v, w, h):
        """Математика проекции 3D -> GPS"""
        if st.alt < 2.0: return None, None

        # FOV (Угол обзора) - Требует калибровки под конкретную камеру!
        HFOV = math.radians(85.0)
        VFOV = HFOV * (h / w)

        alpha_u = ((u / w) - 0.5) * HFOV
        alpha_v = ((v / h) - 0.5) * VFOV

        # Учет наклона подвеса (st.pitch содержит угол подвеса, так как мы берем его из AgentStatus)
        # В vision_node мы договорились писать туда угол гимбала.
        # Если камера смотрит вниз, st.pitch ~ -1.57

        # Угол луча к горизонту
        gamma = math.radians(90.0) + st.pitch - alpha_v

        if gamma <= 0.05: return None, None  # Луч в небо

        dist = st.alt / math.tan(gamma)
        bearing = st.yaw + alpha_u

        dx = dist * math.sin(bearing)
        dy = dist * math.cos(bearing)

        d_lat = (dy / EARTH_RADIUS) * (180.0 / math.pi)
        d_lon = (dx / EARTH_RADIUS) * (180.0 / math.pi) / math.cos(math.radians(st.lat))

        return st.lat + d_lat, st.lon + d_lon

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()


class YoloProcessor(Node):
    def __init__(self):
        super().__init__('yolo_processor')
        self.declare_parameter('model_path', 'yolo11x.pt')
        self.model_path = self.get_parameter('model_path').value

        self.get_logger().info(f"LOADING MODEL: {self.model_path}")
        self.model = YOLO(self.model_path)

        # Словари для мультипоточности
        self.streams = {}  # {agent_id: StreamProcessor}
        self.buffers = {}  # {agent_id: TelemetryBuffer}

        # Подписки
        self.create_subscription(AgentStatus, '/swarm/agent_status', self.agent_status_cb, 10)
        self.target_pub = self.create_publisher(TargetRefined, '/swarm/target_global', 10)
        self.overlay_pub = self.create_publisher(GuiOverlay, '/swarm/gui_overlay', 10)

        self.get_logger().info("YOLO PROCESSOR READY. Waiting for streams...")

    def agent_status_cb(self, msg):
        aid = msg.agent_id

        # 1. Обновляем буфер телеметрии
        if aid not in self.buffers:
            self.buffers[aid] = TelemetryBuffer()
        self.buffers[aid].add(msg)

        # 2. Если это новый РАЗВЕДЧИК или УДАРНИК (который стримит)
        # Проверяем, запущен ли уже обработчик
        if aid not in self.streams and ("vision" in aid or "striker" in aid):
            self.get_logger().info(f"NEW STREAM DETECTED: {aid}")
            processor = StreamProcessor(aid, self.model, self, self.buffers[aid])
            self.streams[aid] = processor

    def destroy_node(self):
        self.get_logger().info("SHUTTING DOWN STREAMS...")
        for s in self.streams.values():
            s.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YoloProcessor())
    rclpy.shutdown()