import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import threading
import multiprocessing
import collections
import time
import subprocess
import os
import ctypes
from ultralytics import YOLO

# Импорт сообщений
from swarm_interfaces.msg import TargetRefined, GuiOverlay, AgentStatus
from std_msgs.msg import Header

# --- КОНСТАНТЫ ---
EARTH_RADIUS = 6378137.0
TARGET_CLASSES = [0, 1, 2, 3, 5, 7]


class TelemetryBuffer:
    def __init__(self, maxlen=100):
        self.buffer = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add(self, msg):
        with self.lock:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.buffer.append((t, msg))

    def get_interpolated_state(self, query_time):
        with self.lock:
            if len(self.buffer) < 2: return None
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
                res = AgentStatus()
                res.lat = m1.lat + (m2.lat - m1.lat) * ratio
                res.lon = m1.lon + (m2.lon - m1.lon) * ratio
                res.alt = m1.alt + (m2.alt - m1.alt) * ratio
                res.roll = m1.roll + (m2.roll - m1.roll) * ratio
                res.pitch = m1.pitch + (m2.pitch - m1.pitch) * ratio
                res.yaw = self._interp_angle(m1.yaw, m2.yaw, ratio)
                return res
            return prev[1] if prev else (nxt[1] if nxt else None)

    def _interp_angle(self, a1, a2, ratio):
        diff = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
        return (a1 + diff * ratio + math.pi) % (2 * math.pi) - math.pi


# --- АСИНХРОННЫЙ ЧИТАТЕЛЬ КАДРОВ ---
class AsyncFrameReader:
    """Читает кадры в отдельном фоновом потоке, всегда хранит только самый свежий. 
       Это предотвращает накопление задержки в буферах FFMPEG/OpenCV."""
    def __init__(self, url):
        self.url = url
        self.cap = cv2.VideoCapture(self.url)
        # Ускоряем открытие и минимизируем внутренние буферы OpenCV, если поддержано
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.ret = False
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    with self.lock:
                        self.ret = ret
                        self.frame = frame
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.1)

    def read(self):
        with self.lock:
            if self.frame is not None:
                # Копируем отдавая, чтобы избежать гонок с записью
                return self.ret, self.frame.copy() 
            return self.ret, None

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()
        
    def isOpened(self):
        return self.cap.isOpened()
        
    def open(self):
        return self.cap.open(self.url)


# --- ПРОЦЕСС ОБРАБОТКИ (Изолирован от GIL) ---
def stream_process_worker(agent_id, model_path, telem_dict, running_flag, 
                          overlay_queue, target_queue, log_queue):
    """
    Эта функция крутится в ОТДЕЛЬНОМ процессе (Process). 
    У нее свой экземпляр YOLO. Она общается с главным ROS-узлом через очереди.
    """
    import logging
    
    def log(msg, level="INFO"):
        try:
            log_queue.put((agent_id, level, msg))
        except:
            pass

    log(f"INIT YOLO ENGINE: {model_path} on separate process...")
    # Инициализация модели ВНУТРИ процесса
    # На RTX 5080 это займет секунду.
    model = YOLO(model_path)
    
    media_server = "127.0.0.1"
    input_url = f"rtmp://{media_server}:8890?streamid=read:{agent_id}"
    output_url = f"rtmp://{media_server}:8890?streamid=publish:{agent_id}_processed"

    log(f"PIPELINE START: {input_url}")
    
    # Асинхронный читатель
    reader = AsyncFrameReader(input_url)

    retry_count = 0
    while not reader.isOpened() and running_flag.value:
        if retry_count % 5 == 0:
            log(f"Waiting for stream...")
        time.sleep(1)
        reader.open()
        retry_count += 1

    # FFMPEG с жесткими настройками ZEROLATENCY
    ffmpeg_cmd = [
        'ffmpeg', '-y', 
        '-f', 'rawvideo', '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24', '-s', '1280x720', '-r', '30', '-i', '-',
        '-c:v', 'h264_nvenc', 
        '-preset', 'p1',             # Fastest preset for NVENC
        '-tune', 'zerolatency',      # CRITICAL: zero latency tuning
        '-rc', 'cbr',
        '-b:v', '2M', 
        '-flags', 'low_delay',       # Disable b-frames
        '-strict', 'experimental',
        '-fflags', 'nobuffer',       # CRITICAL: no buffering inside ffmpeg muxer
        '-f', 'mpegts', output_url
    ]

    process = None
    try:
        process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except Exception as e:
        log(f"FFMPEG START FAILED: {e}", "ERROR")
        reader.release()
        return

    def calculate_precise_gps(st, u, v, w, h):
        if st['alt'] < 2.0: return None, None
        HFOV = math.radians(85.0)
        VFOV = HFOV * (h / w)
        alpha_u = ((u / w) - 0.5) * HFOV
        alpha_v = ((v / h) - 0.5) * VFOV
        gamma = math.radians(90.0) + st['pitch'] - alpha_v
        if gamma <= 0.05: return None, None
        dist = st['alt'] / math.tan(gamma)
        bearing = st['yaw'] + alpha_u
        dx = dist * math.sin(bearing)
        dy = dist * math.cos(bearing)
        d_lat = (dy / EARTH_RADIUS) * (180.0 / math.pi)
        d_lon = (dx / EARTH_RADIUS) * (180.0 / math.pi) / math.cos(math.radians(st['lat']))
        return st['lat'] + d_lat, st['lon'] + d_lon

    processed_frames = 0
    start_time = time.time()

    while running_flag.value:
        ret, frame = reader.read()
        if not ret or frame is None:
            time.sleep(0.005) # Ждем активнее, 5мс
            continue

        h, w = frame.shape[:2]

        # Запускаем YOLO. Отключаем verbose для скорости.
        results = model.track(frame, persist=True, verbose=False, conf=0.45, imgsz=1280, tracker="botsort.yaml")
        
        # Получаем структуру телеметрии из словаря (Manager)
        drone_state_dict = telem_dict.get(agent_id, None)

        for r in results:
            if r.boxes and r.boxes.id is not None:
                boxes_xyxy = r.boxes.xyxy.cpu().numpy()
                track_ids = r.boxes.id.int().cpu().numpy()
                cls_ids = r.boxes.cls.int().cpu().numpy()

                for box, track_id, cls_id in zip(boxes_xyxy, track_ids, cls_ids):
                    cls_name = model.names[cls_id]
                    x1, y1, x2, y2 = box
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    global_target_id = f"{agent_id}_{track_id}"

                    # Шлем данные для отрисовки в ROS
                    try:
                        overlay_queue.put({
                            "source_agent_id": agent_id,
                            "track_id": str(track_id),
                            "class_name": cls_name,
                            "x": float(cx / w), "y": float(cy / h),
                            "w": float((x2 - x1) / w), "h": float((y2 - y1) / h)
                        })
                    except:
                        pass # Очередь переполнена - игнорим графику

                    gps_info = ""
                    if drone_state_dict:
                        lat, lon = calculate_precise_gps(drone_state_dict, cx, cy, w, h)
                        if lat:
                            try:
                                target_queue.put({
                                    "target_id": global_target_id,
                                    "class_name": cls_name,
                                    "lat": lat, "lon": lon
                                })
                                gps_info = "GPS"
                            except:
                                pass

                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    label = f"ID:{track_id} {cls_name} {gps_info}"
                    cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if processed_frames % 100 == 0:
            elapsed = time.time() - start_time
            fps = processed_frames / elapsed if elapsed > 0 else 0
            log(f"Processing FPS: {fps:.1f}")

        try:
            # Запись кадра в FFMPEG pipe. 
            process.stdin.write(frame.tobytes())
            process.stdin.flush() # Принудительно выталкиваем данные
        except BrokenPipeError:
            log(f"FFmpeg Pipe Broken!", "ERROR")
            break
        
        processed_frames += 1

    reader.release()
    if process:
        process.stdin.close()
        process.wait()
    log(f"Worker process termianted.")


# --- ГЛАВНЫЙ ROS УЗЕЛ ---
class YoloProcessorNode(Node):
    def __init__(self):
        super().__init__('yolo_processor')
        self.declare_parameter('model_path', 'yolo11x.pt')
        self.model_path = self.get_parameter('model_path').value
        
        self.get_logger().info(f"YOLO PROCESSOR STARTING (Hardware Accel Mode) Model: {self.model_path}")

        # Мультипроцессинг менеджер (для шаринга телеметрии)
        self.mp_manager = multiprocessing.Manager()
        self.shared_telem = self.mp_manager.dict()
        
        # Очереди для связи Процессов -> Главного узла
        self.overlay_q = multiprocessing.Queue(maxsize=100)
        self.target_q = multiprocessing.Queue(maxsize=100)
        self.log_q = multiprocessing.Queue(maxsize=200)

        self.processes = {}
        self.running_flags = {}
        self.local_buffers = {}

        self.create_subscription(AgentStatus, '/swarm/agent_status', self.agent_status_cb, 10)
        self.target_pub = self.create_publisher(TargetRefined, '/swarm/target_global', 10)
        self.overlay_pub = self.create_publisher(GuiOverlay, '/swarm/gui_overlay', 10)
        
        # Таймер для вычитывания очередей от рабочих процессов (100 Hz)
        self.create_timer(0.01, self.dispatch_queues_cb)

        self.get_logger().info("READY. Waiting for streams...")

    def dispatch_queues_cb(self):
        """Читает результаты из процессов и кидает их в ROS топики"""
        # Логи
        while not self.log_q.empty():
            try:
                aid, lvl, msg = self.log_q.get_nowait()
                txt = f"[{aid}] {msg}"
                if lvl == "ERROR": self.get_logger().error(txt)
                else: self.get_logger().info(txt)
            except: break

        # Оверлеи
        while not self.overlay_q.empty():
            try:
                data = self.overlay_q.get_nowait()
                msg = GuiOverlay()
                msg.source_agent_id = data["source_agent_id"]
                msg.track_id = data["track_id"]
                msg.class_name = data["class_name"]
                msg.x = data["x"]
                msg.y = data["y"]
                msg.w = data["w"]
                msg.h = data["h"]
                self.overlay_pub.publish(msg)
            except: break
            
        # Цели (GPS)
        while not self.target_q.empty():
            try:
                data = self.target_q.get_nowait()
                msg = TargetRefined()
                msg.target_id = data["target_id"]
                msg.class_name = data["class_name"]
                msg.lat = data["lat"]
                msg.lon = data["lon"]
                self.target_pub.publish(msg)
            except: break

    def agent_status_cb(self, msg):
        aid = msg.agent_id
        
        # 1. Локальная буферизация для точной интерполяции
        if aid not in self.local_buffers:
            self.local_buffers[aid] = TelemetryBuffer()
        
        # Оцениваем задержку видео как 150мс (оптимизировано под zero-latency pipeline)
        capture_time = self.get_clock().now().nanoseconds * 1e-9 - 0.150
        
        self.local_buffers[aid].add(msg)
        interp_state = self.local_buffers[aid].get_interpolated_state(capture_time)
        
        # 2. Обновление расшаренного словаря для процессов
        if interp_state:
            self.shared_telem[aid] = {
                'lat': interp_state.lat, 'lon': interp_state.lon, 'alt': interp_state.alt,
                'pitch': interp_state.pitch, 'yaw': interp_state.yaw, 'roll': interp_state.roll
            }

        # 3. Запуск процесса видео-аналитики, если дрон с камерой
        if aid not in self.processes and ("vision" in aid or "striker" in aid):
            self.get_logger().info(f"SPAWNING YOLO PROCESS FOR: {aid}")
            
            flag = multiprocessing.Value(ctypes.c_bool, True)
            self.running_flags[aid] = flag
            
            p = multiprocessing.Process(
                target=stream_process_worker,
                args=(aid, self.model_path, self.shared_telem, flag, 
                      self.overlay_q, self.target_q, self.log_q)
            )
            p.daemon = True
            p.start()
            self.processes[aid] = p

    def destroy_node(self):
        self.get_logger().info("SHUTTING DOWN MULTIPROCESSING WORKERS...")
        for aid, flag in self.running_flags.items():
            flag.value = False
            
        for aid, p in self.processes.items():
            p.join(timeout=2.0)
            if p.is_alive():
                p.terminate()
                
        super().destroy_node()


def main(args=None):
    # Требуется для корректной работы multiprocessing в PyTorch / ROS2
    # multiprocessing.set_start_method('spawn', force=True)
    
    rclpy.init(args=args)
    node = YoloProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # ОБЯЗАТЕЛЬНО для PyTorch в процессах, иначе зависнет CUDA:
    import torch.multiprocessing as mp
    try:
         mp.set_start_method('spawn')
    except RuntimeError:
         pass
         
    main()
