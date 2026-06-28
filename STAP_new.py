"""
STAP: Smart Traffic Automation Program
=======================================
v17.2.2 — Standards-Compliant Hybrid Sequential Micro-Phasing Architecture
        with Clean Top Quad Labels, Real-Time Hardware Signal Sync Mirroring,
        Dynamic Spatial Occupancy Tracking, Reinforced Emergency Preemption Filtering,
        and Inverted Outbound Signal Structural Mapping.
"""

from ultralytics import YOLO
import cv2
import time
import serial
import numpy as np
import requests
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
import threading
import torch
import os
import collections
import csv
from datetime import datetime
from flask import Flask, Response, request, jsonify

# =============================================================
# 0. AUTO-INCREMENTING RUN DIRECTORY SETUP
# =============================================================
BASE_RUNS_DIR = "runs"
os.makedirs(BASE_RUNS_DIR, exist_ok=True)

run_idx = 1
while os.path.exists(os.path.join(BASE_RUNS_DIR, f"run_{run_idx}")):
    run_idx += 1

CURRENT_RUN_DIR = os.path.join(BASE_RUNS_DIR, f"run_{run_idx}")
os.makedirs(CURRENT_RUN_DIR, exist_ok=True)
print(f"[STAP] 📂 Initialized Session Storage Directory: {CURRENT_RUN_DIR}")

# =============================================================
# 1. CONFIGURATION & FLASK SERVER BOOT
# =============================================================
app = Flask(__name__)

# Standard serial link port & baud config for ESP32 UART bridge
SERIAL_PORT = "COM11"
BAUD_RATE   = 115200

# Local YOLO weight coefficients path
MODEL_PATH = r"C:\Users\Raphael\Desktop\YOLO\mayor_gil6\runs\detect\train\weights\best.pt"

EMERGENCY_CLASS_IDS = [0, 5, 9]   # ambulance=0, firetruck=5, police=9
VEHICLE_CLASS_IDS   = [1, 2, 3, 4, 6, 7, 8, 10, 11, 12, 13]

LANE_NAMES  = ["NORTH", "SOUTH", "EAST", "WEST"]
PHASE_ORDER = ["NORTH", "SOUTH", "EAST", "WEST"]

# --- OUTBOUND PHYSICAL INTERSECTION MAP ENGINE ---
# Resolves location approach camera matching vs outbound directional viewing structure layout
OPPOSITE_LIGHT_MAP = {
    "NORTH": "SOUTH",  # North approach cars physically read the South traffic light signal
    "SOUTH": "NORTH",  # South approach cars physically read the North traffic light signal
    "EAST":  "WEST",   # East approach cars physically read the West traffic light signal
    "WEST":  "EAST"    # West approach cars physically read the East traffic light signal
}

# --- DYNAMIC CSV TIME MATRIX TRACKING CONFIGS ---
CSV_LOG_INTERVAL = 300 
last_csv_log_time = time.time()

# --- HAZARD RECOGNITION TRACKING REGISTER ---
last_hazard_blink_time = 0.0

# --- ADVANCED HUD LANE-SPECIFIC INTERSECTION COLOR REGISTERS (BGR Format) ---
ROI_COLORS = {
    "NORTH": (255, 255, 0),    # Cyan
    "SOUTH": (0, 255, 0),      # Bright Green
    "EAST": (0, 255, 255),     # Yellow
    "WEST": (255, 0, 255)      # Magenta
}
ROI_ALPHA = 0.15 

# Local calibration video files
VIDEO_FILES = [
    r"C:\Users\Raphael\Desktop\YOLO\FINAL\13_North.MOV",
    r"C:\Users\Raphael\Desktop\YOLO\FINAL\13_South.mp4",
    r"C:\Users\Raphael\Desktop\YOLO\FINAL\13_East.mp4",
    r"C:\Users\Raphael\Desktop\YOLO\FINAL\13_West.MOV",
]

LOOP_VIDEOS  = True
CAM_WIDTH    = 640
CAM_HEIGHT   = 480
TARGET_FPS   = 30
DATA_TIMEOUT = 5.0

# --- AUTOMATED REGION OF INTEREST (ROI) ENGINE ---
RAW_HIGH_RES_ROIS = {
    "WEST": np.array([[683, 1534], [1853, 427], [2526, 424], [2748, 1605]], dtype=np.int32),
    "NORTH": np.array([[2173, 2159], [2109, 999], [2065, 450], [2017, 137], [1779, 134], 
                       [1567, 464], [991, 1263], [497, 1880], [761, 2155]], dtype=np.int32),
    "EAST": np.array([[7, 1713], [-1, 1181], [683, 528], [932, 320], [1181, 175], 
                      [1835, 175], [2303, 1735]], dtype=np.int32),
    "SOUTH": np.array([[579, 897], [601, 528], [869, 318], [1250, 310], [1361, 927]], dtype=np.int32)
}

ROI_POLYGONS = {}
ROI_STATIC_AREAS = {} 
print("[STAP] Calibrating matching ROI geometry scales automatically...")

for idx, lane in enumerate(LANE_NAMES):
    test_cap = cv2.VideoCapture(VIDEO_FILES[idx])
    src_w = test_cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    src_h = test_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    test_cap.release()
    
    if src_w <= 0 or src_h <= 0:
        src_w, src_h = 1920.0, 1080.0
        print(f"[STAP] ⚠️ Unable to read video dimensions for {lane}. Using fallback 1920x1080.")
    else:
        print(f"[STAP] Direct mapping calibrated for {lane}: Source ({int(src_w)}x{int(src_h)}) -> Process Target ({CAM_WIDTH}x{CAM_HEIGHT})")

    poly_points = RAW_HIGH_RES_ROIS[lane].copy().astype(np.float32)
    poly_points[:, 0] = (poly_points[:, 0] / src_w) * CAM_WIDTH
    poly_points[:, 1] = (poly_points[:, 1] / src_h) * CAM_HEIGHT
    downscaled_poly = poly_points.astype(np.int32)
    
    ROI_POLYGONS[lane] = downscaled_poly
    ROI_STATIC_AREAS[lane] = float(cv2.contourArea(downscaled_poly))

BASE_GREEN = {"NORTH": 50, "SOUTH": 50, "EAST": 39, "WEST": 35}

YELLOW_TIME        = 3 
ALL_RED_TIME       = 2 
CONGESTION_CEILING = 20 

MIN_GREEN       = {lane: max(7,  int(BASE_GREEN[lane] * 0.40)) for lane in LANE_NAMES}
MAX_GREEN       = {lane: min(65, int(BASE_GREEN[lane] * 1.30)) for lane in LANE_NAMES}
MAX_ADJUSTMENT  = 10 

LOS_THRESHOLDS = [("A",0,1),("B",2,3),("C",4,6),("D",7,10),("E",11,15),("F",16,999)]
LOS_DELTA      = {"A":-10,"B":-6,"C":0,"D":+6,"E":+8,"F":+10}

PING_INTERVAL = 0.4

CONF_THRESHOLD            = 0.50 
EMERGENCY_SUSTAIN_SECONDS = 3.0 
COUNT_SMOOTH_WINDOW       = 8 

# REAL CLUSTER CONSOLE LINK (Prefilled for instant cloud synchronization)
NODE_API_KEY       = "node_alpha_J7FVxdRBqwCBWQSdiKBN742lMHuEPX5A"
STAP_HUB_URL       = "https://ais-dev-z354axc6z3etv7l7kcerci-1033031146789.asia-southeast1.run.app/api/v1/snapshots"
STAP_HEARTBEAT_URL = "https://ais-dev-z354axc6z3etv7l7kcerci-1033031146789.asia-southeast1.run.app/api/v1/heartbeat"
HUB_ENABLED        = True
HUB_INTERVAL_TICKS = 75 

CAMERA_MAP = {"NORTH": 1, "SOUTH": 2, "EAST": 3, "WEST": 4}
CONGESTION_MAP = {"A": "A", "B": "B", "C": "C", "D": "D", "E": "E", "F": "F"}

# =============================================================
# 2. HARDWARE ACCELERATION
# =============================================================
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

print("[STAP] Evaluating hardware acceleration...")
if torch.cuda.is_available():
    DEVICE = 0
    torch.backends.cudnn.benchmark     = True
    torch.backends.cudnn.deterministic = False
    print(f"[STAP] ✅ GPU: {torch.cuda.get_device_name(0)}")
    print(f"[STAP] ✅ VRAM: {torch.cuda.get_device_properties(0).total_memory/1e9:.1f} GB")
    AI_SLEEP = 0.01
else:
    DEVICE   = "cpu"
    AI_SLEEP = 0.15
    print("[STAP] ⚠️ CPU Mode — install CUDA PyTorch for better performance")

# =============================================================
# 3. THREAD-SAFE STORAGE & LOCKS
# =============================================================
frame_lock  = threading.Lock()
result_lock = threading.Lock()

lane_stream_locks = {lane: threading.Lock() for lane in LANE_NAMES}
global_lane_frames = {lane: None for lane in LANE_NAMES}

latest_frames  = [None, None, None, None]
cached_boxes   = {lane: [] for lane in LANE_NAMES}
vehicle_counts = {lane: 0  for lane in LANE_NAMES}
lane_statuses  = {lane: "CLEAR" for lane in LANE_NAMES}
lane_live_occupancy_pct = {lane: 0.0 for lane in LANE_NAMES}

# Master Light States (Populated strictly by data packets streaming back from ESP32)
manual_lane_lights = {lane: "RED" for lane in LANE_NAMES}

analytics_lock = threading.Lock()
global_analytics_registry = {lane: collections.defaultdict(int) for lane in LANE_NAMES}
known_classes_seen = set() 

phase_lock         = threading.Lock()
current_phase_idx  = 0
phase_state        = "GREEN"
green_start_time   = time.time()
yellow_start_time  = 0.0
all_red_start_time = 0.0
committed_green    = BASE_GREEN[PHASE_ORDER[0]]

# =============================================================
# 3b. DETECTION STABILITY CLASSES
# =============================================================
class EmergencyBuffer:
    def __init__(self, sustain_seconds: float):
        self.sustain  = sustain_seconds
        self._first   = {} 
        self._active  = {} 

    def update(self, lane: str, detected: bool):
        if detected:
            if lane not in self._first:
                self._first[lane] = time.time()        
            elif time.time() - self._first[lane] >= self.sustain:
                self._active[lane] = True              
        else:
            self._first.pop(lane, None)                
            self._active[lane] = False                 

    def is_confirmed(self, lane: str) -> bool:
        return self._active.get(lane, False)

    def streak_elapsed(self, lane: str) -> float:
        if lane in self._first:
            return time.time() - self._first[lane]
        return 0.0

    def is_charging(self, lane: str) -> bool:
        return lane in self._first and not self._active.get(lane, False)


class VehicleCountSmoother:
    def __init__(self, window: int):
        self.window  = window
        self._queues = {lane: collections.deque(maxlen=window) for lane in LANE_NAMES}

    def push(self, lane: str, count: int):
        self._queues[lane].append(count)

    def get(self, lane: str) -> int:
        q = self._queues[lane]
        if not q: return 0
        sorted_q = sorted(q)
        mid = len(sorted_q) // 2
        return sorted_q[mid]  


emg_buffer     = EmergencyBuffer(EMERGENCY_SUSTAIN_SECONDS)
count_smoother = VehicleCountSmoother(COUNT_SMOOTH_WINDOW)

# =============================================================
# 4. BACKGROUND VIDEO READERS
# =============================================================
class BackgroundVideoReader(threading.Thread):
    def __init__(self, index, path):
        super().__init__(daemon=True)
        self.index = index
        self.cap   = cv2.VideoCapture(path)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_interval = 1.0 / (fps if fps and fps > 0 else 30)
        self.running = True

    def run(self):
        while self.running:
            t0 = time.time()
            ret, frame = self.cap.read()
            if not ret and LOOP_VIDEOS:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            if ret:
                with frame_lock:
                    latest_frames[self.index] = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
            elapsed = time.time() - t0
            sleep_t = self.frame_interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

# =============================================================
# 5. AI INFERENCE CORE
# =============================================================
class BackgroundAIProcessor(threading.Thread):
    def __init__(self, model_path, device):
        super().__init__(daemon=True)
        # Note: If local weights are unavailable yet, initialize standard YOLO nano as a failsafe
        try:
            self.models = {lane: YOLO(model_path) for lane in LANE_NAMES}
        except Exception:
            print("[STAP WARNING] Local weight best.pt not found. Utilizing standard yolov8n.pt...")
            self.models = {lane: YOLO("yolov8n.pt") for lane in LANE_NAMES}
            
        self.device  = device
        self.labels  = self.models["NORTH"].names
        self.running = True
        self.half    = device != "cpu"
        
        self.tracked_vehicle_ids = {lane: set() for lane in LANE_NAMES}
        
        if device != "cpu":
            for lane in LANE_NAMES:
                self.models[lane].to(device)
            print(f"[STAP] ✅ 4 Isolated Tracker Streams initialized on GPU (device={device})")
        else:
            print("[STAP] ⚠️ YOLO models on CPU")

    def run(self):
        global cached_boxes, vehicle_counts, lane_statuses, known_classes_seen, lane_live_occupancy_pct
        while self.running:
            temp_counts = {l: 0 for l in LANE_NAMES}
            temp_statuses = {l: "CLEAR" for l in LANE_NAMES}
            temp_boxes = {l: [] for l in LANE_NAMES}
            temp_occupancy_pixels = {l: 0.0 for l in LANE_NAMES}

            for idx, lane in enumerate(LANE_NAMES):
                img = None
                with frame_lock:
                    if latest_frames[idx] is not None:
                        img = latest_frames[idx].copy()
                
                if img is None:
                    continue

                try:
                    r = self.models[lane].track(
                        img, 
                        persist=True, 
                        conf=CONF_THRESHOLD,
                        verbose=False, 
                        device=self.device, 
                        imgsz=640,
                        half=self.half
                    )
                    res = r[0]
                except Exception as e:
                    print(f"[STAP] Track engine drop on {lane}: {e}")
                    continue

                if res is None or res.boxes is None: 
                    continue
                
                polygon = ROI_POLYGONS[lane]
                
                for box in res.boxes:
                    cls_id = int(box.cls[0])
                    conf   = float(box.conf[0])
                    bx1, by1, bx2, by2 = map(int, box.xyxy[0])
                    cx, cy = (bx1 + bx2) // 2, (by1 + by2) // 2
                    
                    is_inside = cv2.pointPolygonTest(polygon, (float(cx), float(cy)), False) >= 0
                    if not is_inside: 
                        continue

                    is_emg = cls_id in EMERGENCY_CLASS_IDS
                    is_veh = cls_id in VEHICLE_CLASS_IDS
                    
                    if is_emg or is_veh:
                        if is_emg:
                            temp_statuses[lane] = "EMERGENCY"
                        elif temp_statuses[lane] != "EMERGENCY":
                            temp_statuses[lane] = "VEHICLE"
                        
                        temp_counts[lane] += 1
                        
                        box_width = bx2 - bx1
                        box_height = by2 - by1
                        temp_occupancy_pixels[lane] += float(box_width * box_height)
                        
                        if box.id is not None:
                            track_id = int(box.id[0])
                            unique_key = f"{cls_id}_{track_id}"
                            
                            if unique_key not in self.tracked_vehicle_ids[lane]:
                                self.tracked_vehicle_ids[lane].add(unique_key)
                                class_name = self.labels.get(cls_id, f"Class_{cls_id}")
                                with analytics_lock:
                                    global_analytics_registry[lane][class_name] += 1
                                    known_classes_seen.add(class_name)

                        temp_boxes[lane].append({
                            "coords": (bx1, by1, bx2, by2),
                            "label" : f"{self.labels.get(cls_id, 'Vehicle')} {conf:.2f}",
                            "color" : (0, 0, 255) if is_emg else ROI_COLORS[lane],
                        })

            for lane in LANE_NAMES:
                count_smoother.push(lane, temp_counts[lane])
                emg_buffer.update(lane, temp_statuses[lane] == "EMERGENCY")

            with result_lock:
                cached_boxes   = temp_boxes
                vehicle_counts = {lane: count_smoother.get(lane) for lane in LANE_NAMES}
                lane_statuses  = {
                    lane: ("EMERGENCY" if emg_buffer.is_confirmed(lane) else temp_statuses[lane])
                    for lane in LANE_NAMES
                }
                for lane in LANE_NAMES:
                    static_total = ROI_STATIC_AREAS[lane]
                    if static_total > 0:
                        lane_live_occupancy_pct[lane] = min(100.0, (temp_occupancy_pixels[lane] / static_total) * 100.0)
                    else:
                        lane_live_occupancy_pct[lane] = 0.0

            time.sleep(AI_SLEEP)

# =============================================================
# 6. INITIALIZATION — BOOT MANAGEMENT
# =============================================================
print("[STAP] Booting video readers...")
readers = [BackgroundVideoReader(i, VIDEO_FILES[i]) for i in range(4)]
for r in readers: r.start()

print("[STAP] Connecting to ESP32 Hardware Module...")
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"[STAP] ✅ Connected to ESP32 on {SERIAL_PORT}")
    time.sleep(1)
    print("[STAP] Sending boot keepalives while YOLO loads...")
    for _ in range(15):
        ser.write(b"PING:NORTH\n")
        ser.flush()
        time.sleep(0.3)
except Exception as e:
    print(f"[STAP] ❌ Serial connection failed ({e}). Running in Offline-simulation mode.")
    ser = None

print("[STAP] Booting AI core...")
ai_core = BackgroundAIProcessor(MODEL_PATH, DEVICE)
ai_core.start()

print("[STAP] Warming up visual buffers...")
time.sleep(2.0)

# =============================================================
# 7. HELPERS & LOCALIZED ADAPTIVE LOGIC
# =============================================================
def send_to_esp32(msg: str):
    if ser and ser.is_open:
        try:
            ser.write(f"{msg}\n".encode("utf-8"))
            ser.flush()
        except Exception: pass

rain_detected   = False
manual_override = False
hazard_active   = False
emergency_active = False
last_comm_time  = time.time()

def read_serial_incoming():
    global rain_detected, manual_override, last_comm_time, hazard_active, emergency_active
    if not ser:
        return
    try:
        lines_read = 0
        while ser.in_waiting > 0 and lines_read < 30:
            lines_read += 1
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            
            last_comm_time = time.time()
            
            if "RAIN:" in line and "MODE:" in line:
                for part in line.split(","):
                    if part.startswith("RAIN:"):
                        rain_detected = (part.split(":")[1] == "1")
                    elif part.startswith("MODE:"):
                        manual_override = (part.split(":")[1] == "MANUAL")
                    elif part.startswith("HAZARD:"):
                        hazard_active = (part.split(":")[1] == "1")
                    elif part.startswith("EMERGENCY:"):
                        emergency_active = (part.split(":")[1] == "1")
            
            elif line.startswith("STATE:"):
                payload = line.replace("STATE:", "")
                lane, hardware_lamp = payload.split(",")
                if lane in LANE_NAMES and hardware_lamp in ["RED", "YELLOW", "GREEN"]:
                    with result_lock:
                        manual_lane_lights[lane] = hardware_lamp
                    if hardware_lamp == "GREEN":
                        with phase_lock:
                            current_phase_idx = PHASE_ORDER.index(lane)
                            phase_state = "GREEN"
                    elif hardware_lamp == "YELLOW":
                        with phase_lock:
                            current_phase_idx = PHASE_ORDER.index(lane)
                            phase_state = "YELLOW"
    except Exception:
        pass

def get_lane_light_states():
    with result_lock:
        local_manual_lights = manual_lane_lights.copy()
    with phase_lock:
        snap_lane = PHASE_ORDER[current_phase_idx]
        snap_state = phase_state

    lights = {}
    for lane in LANE_NAMES:
        light_is_red    = False
        light_is_yellow = False
        light_is_green  = False

        if manual_override:
            if emergency_active:
                light_is_red = True
            elif hazard_active:
                light_is_yellow = True
            else:
                current_hardware_lamp = local_manual_lights.get(lane, "RED")
                if current_hardware_lamp == "GREEN":     
                    light_is_green = True
                elif current_hardware_lamp == "YELLOW": 
                    light_is_yellow = True
                else:
                    light_is_red = True
        else:
            if snap_state == "ALL_RED":
                light_is_red = True
            elif lane == snap_lane:
                if snap_state == "GREEN":   light_is_green = True
                elif snap_state == "YELLOW": light_is_yellow = True
            else:
                light_is_red = True 

        light_str = "RED"
        if light_is_green: light_str = "GREEN"
        elif light_is_yellow: light_str = "YELLOW"
        lights[lane] = light_str

    return lights

def classify_los(count: int) -> str:
    for grade, lo, hi in LOS_THRESHOLDS:
        if lo <= count <= hi: return grade
    return "F"

def compute_green_time(lane: str, rain: bool) -> int:
    with result_lock: 
        current_queue = vehicle_counts[lane]
        total_intersection_backpressure = sum(vehicle_counts[l] for l in LANE_NAMES if l != lane)
    
    if current_queue <= 2:
        rain_mod = 1.20 if rain else 1.0
        return max(7, min(10, int(7 * rain_mod)))
        
    los   = classify_los(current_queue)
    delta = max(-MAX_ADJUSTMENT, min(MAX_ADJUSTMENT, LOS_DELTA[los]))
    
    if total_intersection_backpressure >= CONGESTION_CEILING and delta > 0:
        delta = int(delta * 0.5)
        
    green = BASE_GREEN[lane] + delta
    if rain and los in ["D", "E", "F"]:
        green += 5
        
    return max(MIN_GREEN[lane], min(MAX_GREEN[lane], green))

def compute_red_time(lane: str, greens: dict) -> int:
    return sum(greens[l] + YELLOW_TIME + ALL_RED_TIME for l in PHASE_ORDER if l != lane)

def reset_all_manual_lights_to_red():
    with result_lock:
        for lane in LANE_NAMES:
            manual_lane_lights[lane] = "RED"

def emergency_lane():
    for lane in LANE_NAMES:
        if emg_buffer.is_confirmed(lane):
            return lane
    return None

def post_to_hub():
    if not HUB_ENABLED: return
    def _post():
        try:
            with result_lock:
                counts   = vehicle_counts.copy()
                statuses = lane_statuses.copy()

            headers = {
                "Authorization": f"Bearer {NODE_API_KEY}",
                "Content-Type":  "application/json",
                "Accept":        "application/json",
            }

            for lane in LANE_NAMES:
                total     = counts[lane]
                los       = classify_los(total)
                camera_id = CAMERA_MAP.get(lane, 1)

                cars        = int(total * 0.45)
                trucks      = int(total * 0.10)
                motorcycles = int(total * 0.25)
                buses       = int(total * 0.10)
                emergency   = 1 if statuses.get(lane) == "EMERGENCY" else 0

                body = {
                    "camera_id":          camera_id,
                    "cars":               cars,
                    "trucks":             trucks,
                    "motorcycles":        motorcycles,
                    "buses":              buses,
                    "emergency_vehicles": emergency,
                    "congestion":         CONGESTION_MAP.get(los, "free_flow"),
                    "snapshot_time":      datetime.now().isoformat(),
                }
                requests.post(STAP_HUB_URL, json=body, headers=headers, timeout=2.0, verify=False)
        except Exception as e:
            print(f"[STAP] Cloud synchronization delay: {e}")
    threading.Thread(target=_post, daemon=True).start()

def hub_heartbeat_thread():
    while True:
        try:
            requests.post(
                STAP_HEARTBEAT_URL,
                headers={"Authorization": f"Bearer {NODE_API_KEY}", "Accept": "application/json"},
                timeout=2.0,
                verify=False
            )
        except Exception as e:
            print(f"[STAP] Heartbeat failed to send: {e}")
        time.sleep(5)

# =============================================================
# 8. PER-LANE FLASK MJPEG MULTI-STREAM INTERFACES
# =============================================================
def generate_lane_stream(lane_name: str):
    while True:
        time.sleep(0.03)  
        with lane_stream_locks[lane_name]:
            frame = global_lane_frames[lane_name]
            if frame is None: continue
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not ret: continue
            frame_bytes = buffer.tobytes()
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed/north')
def feed_north(): return Response(generate_lane_stream("NORTH"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/south')
def feed_south(): return Response(generate_lane_stream("SOUTH"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/east')
def feed_east(): return Response(generate_lane_stream("EAST"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/west')
def feed_west(): return Response(generate_lane_stream("WEST"), mimetype='multipart/x-mixed-replace; boundary=frame')

# =============================================================
# 8b. CONTROL & STATUS API ROUTES
# =============================================================
@app.after_request
def add_cors(response):
    response.headers["Access-Control-Allow-Origin"]  = "*"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    return response

@app.route('/control/mode', methods=['POST', 'OPTIONS'])
def control_mode():
    if request.method == 'OPTIONS': return jsonify({}), 200
    global manual_override, current_phase_idx, phase_state, green_start_time, committed_green, hazard_active, emergency_active
    data = request.get_json(force=True)
    mode = data.get('mode', '').lower()

    if mode not in ['auto', 'manual', 'hazard', 'emergency']:
        return jsonify({'success': False, 'message': 'Invalid mode.'}), 400

    if mode == 'auto':
        manual_override = False
        hazard_active = False
        emergency_active = False
        send_to_esp32('MODE:AUTO')
        with phase_lock: lane = PHASE_ORDER[current_phase_idx]
        green = compute_green_time(lane, rain_detected)
        start_green(lane, green)
    elif mode == 'manual':
        if not manual_override or hazard_active or emergency_active:
            manual_override = True
            hazard_active = False
            emergency_active = False
            send_to_esp32('MODE:MANUAL')
            reset_all_manual_lights_to_red()
    elif mode == 'hazard':
        manual_override = True
        hazard_active = True
        emergency_active = False
        send_to_esp32('MODE:HAZARD')
        for lane in LANE_NAMES: send_to_esp32(f'HAZARD:{lane}')
    elif mode == 'emergency':
        manual_override = True
        hazard_active = False
        emergency_active = True
        send_to_esp32('MODE:EMERGENCY')
        reset_all_manual_lights_to_red()

    return jsonify({'success': True, 'mode': mode})

@app.route('/control/light', methods=['POST', 'OPTIONS'])
def control_light():
    if request.method == 'OPTIONS': return jsonify({}), 200
    global current_phase_idx, phase_state, green_start_time
    if not manual_override: return jsonify({'success': False, 'message': 'Requires Manual Override mode first.'}), 422
    data  = request.get_json(force=True)
    lane  = data.get('lane', '').upper()
    state = data.get('state', '').lower()

    if lane not in LANE_NAMES or state not in ['red', 'yellow', 'green']:
        return jsonify({'success': False, 'message': 'Invalid fields.'}), 400

    if lane in PHASE_ORDER:
        with phase_lock:
            current_phase_idx = PHASE_ORDER.index(lane)
            phase_state = 'GREEN' if state == 'green' else ('YELLOW' if state == 'yellow' else 'ALL_RED')
            green_start_time = time.time()

    send_to_esp32(f'MANUAL_LIGHT:{lane},{state.upper()}')
    return jsonify({'success': True, 'lane': lane, 'state': state})

@app.route('/control/emergency', methods=['POST', 'OPTIONS'])
def control_emergency():
    if request.method == 'OPTIONS': return jsonify({}), 200
    data = request.get_json(force=True)
    lane = data.get('lane', '').upper()

    if lane not in LANE_NAMES: return jsonify({'success': False, 'message': 'Invalid approach.'}), 400

    start_yellow(PHASE_ORDER[current_phase_idx])
    time.sleep(0.1)
    start_green(lane, compute_green_time(lane, rain_detected))
    send_to_esp32(f'EMERGENCY_OVERRIDE:{lane}')
    return jsonify({'success': True, 'emergency_lane': lane})

@app.route('/status', methods=['GET', 'OPTIONS'])
def get_status():
    if request.method == 'OPTIONS':
        from flask import make_response
        resp = make_response()
        resp.headers.add("Access-Control-Allow-Origin", "*")
        resp.headers.add("Access-Control-Allow-Headers", "Content-Type, Authorization")
        resp.headers.add("Access-Control-Allow-Methods", "GET, OPTIONS")
        return resp

    global STAP_HUB_URL, STAP_HEARTBEAT_URL
    hub_origin = request.args.get('hub_origin')
    if hub_origin:
        hub_origin = hub_origin.rstrip('/')
        STAP_HUB_URL = f"{hub_origin}/api/v1/snapshots"
        STAP_HEARTBEAT_URL = f"{hub_origin}/api/v1/heartbeat"
        print(f"[STAP] 🔄 Dynamically updated cloud hub URLs to: {hub_origin}")
        
        # Trigger an immediate out-of-band heartbeat to instantly flag node as online in Express
        def trigger_immediate_heartbeat():
            try:
                requests.post(
                    STAP_HEARTBEAT_URL,
                    headers={"Authorization": f"Bearer {NODE_API_KEY}", "Accept": "application/json"},
                    timeout=2.0,
                    verify=False
                )
                print("[STAP] ⚡ Triggered immediate cloud heartbeat successfully.")
            except Exception as e:
                print(f"[STAP] ⚠️ Immediate heartbeat failed: {e}")
        threading.Thread(target=trigger_immediate_heartbeat, daemon=True).start()

    with result_lock:
        counts   = vehicle_counts.copy()
        statuses = lane_statuses.copy()
        occupancy = lane_live_occupancy_pct.copy()
    with phase_lock:
        active_lane   = PHASE_ORDER[current_phase_idx]
        current_state = phase_state
        green_dur     = committed_green

    now = time.time()
    if current_state == 'GREEN':
        remaining = max(0, green_dur - int(now - green_start_time))
    elif current_state == 'YELLOW':
        remaining = max(0, YELLOW_TIME - int(now - yellow_start_time if yellow_start_time > 0 else 0.0))
    else:
        remaining = max(0, ALL_RED_TIME - int(now - all_red_start_time if all_red_start_time > 0 else 0.0))

    status_mode = 'auto'
    if emergency_active:
        status_mode = 'emergency'
    elif hazard_active:
        status_mode = 'hazard'
    elif manual_override:
        status_mode = 'manual'

    los_per_lane = {lane: classify_los(counts[lane]) for lane in LANE_NAMES}
    
    lights_map = get_lane_light_states()
    lanes_payload = {}
    for lane in LANE_NAMES:
        density = int(occupancy.get(lane, 0.0))
        lanes_payload[lane] = {
            "count": counts[lane],
            "density": density,
            "light": lights_map[lane],
            "los": los_per_lane[lane]
        }

    resp = jsonify({
        'active_lane': active_lane, 'phase_state': current_state, 'remaining_secs': remaining,
        'green_duration': green_dur, 'mode': status_mode, 'rain': rain_detected,
        'vehicle_counts': counts, 'los': los_per_lane, 'lane_statuses': statuses,
        'lanes': lanes_payload
    })
    resp.headers.add("Access-Control-Allow-Origin", "*")
    resp.headers.add("Access-Control-Allow-Headers", "Content-Type, Authorization")
    resp.headers.add("Access-Control-Allow-Methods", "GET, OPTIONS")
    return resp

def run_flask_server():
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)

# =============================================================
# 9. TIMING PHASE SYSTEM TRANSITIONS
# =============================================================
def start_yellow(lane: str):
    global phase_state, yellow_start_time
    with phase_lock:
        phase_state       = "YELLOW"
        yellow_start_time = time.time()
    
    # STRUCTURAL FIX: Invert automated approach string payload to match driver sight lines
    target_light = OPPOSITE_LIGHT_MAP[lane]
    send_to_esp32(f"YELLOW:{target_light}")
    send_to_esp32(f"DISPLAY:YELLOW,{YELLOW_TIME}")

def start_all_red():
    global phase_state, all_red_start_time
    with phase_lock:
        phase_state        = "ALL_RED"
        all_red_start_time = time.time()
    send_to_esp32("PHASE:ALL_RED,DURATION:2")
    send_to_esp32("DISPLAY:OFF")
    print("[STAP] 🚨 All-Red clearance safety interval initialized intersection-wide.")

def start_green(next_lane: str, duration: int):
    global current_phase_idx, phase_state, green_start_time, yellow_start_time, all_red_start_time, committed_green
    with phase_lock:
        current_phase_idx  = PHASE_ORDER.index(next_lane)
        phase_state        = "GREEN"
        green_start_time   = time.time()
        yellow_start_time  = 0.0
        all_red_start_time = 0.0
        committed_green    = duration
        
    # STRUCTURAL FIX: Invert automated approach string payload to match driver sight lines
    target_light = OPPOSITE_LIGHT_MAP[next_lane]
    send_to_esp32(f"PHASE:{target_light},DURATION:{duration}")
    send_to_esp32("DISPLAY:OFF")

def advance_phase():
    global current_phase_idx
    emg = emergency_lane()
    if emg:
        next_lane = emg
    else:
        with phase_lock: next_idx = (current_phase_idx + 1) % len(PHASE_ORDER)
        next_lane = PHASE_ORDER[next_idx]
    green = compute_green_time(next_lane, rain_detected)
    start_green(next_lane, green)

def keepalive_thread():
    hub_tick = 0
    while True:
        time.sleep(PING_INTERVAL)
        with phase_lock: active = PHASE_ORDER[current_phase_idx]
        
        # STRUCTURAL FIX: Ping correct target frame node layout over COM channel
        target_light = OPPOSITE_LIGHT_MAP[active]
        send_to_esp32(f"PING:{target_light}")
        
        hub_tick += 1
        if hub_tick >= HUB_INTERVAL_TICKS:
            hub_tick = 0
            post_to_hub()

def hub_realtime_sync_thread():
    if not HUB_ENABLED: return
    while True:
        try:
            control_url = STAP_HUB_URL.replace("/api/v1/snapshots", "/api/v1/control")
            with result_lock:
                counts   = vehicle_counts.copy()
                statuses = lane_statuses.copy()
                occupancy = lane_live_occupancy_pct.copy()
                local_manual_lights = manual_lane_lights.copy()
            
            with phase_lock:
                snap_lane    = PHASE_ORDER[current_phase_idx]
                snap_state   = phase_state
                snap_green   = committed_green
                snap_g_start = green_start_time
                snap_y_start = yellow_start_time
                snap_ar_start= all_red_start_time

            now = time.time()
            if snap_state == "GREEN":
                disp_remain = max(0, snap_green - int(now - snap_g_start))
            elif snap_state == "YELLOW":
                disp_remain = max(0, YELLOW_TIME - int(now - snap_y_start if snap_y_start > 0 else 0.0))
            else:
                disp_remain = max(0, ALL_RED_TIME - int(now - snap_ar_start if snap_ar_start > 0 else 0.0))

            lanes_payload = {}
            for lane in LANE_NAMES:
                light_is_red    = False
                light_is_yellow = False
                light_is_green  = False

                if manual_override:
                    if emergency_active:
                        light_is_red = True
                    elif hazard_active:
                        light_is_yellow = True
                    else:
                        current_hardware_lamp = local_manual_lights.get(lane, "RED")
                        if current_hardware_lamp == "GREEN":     
                            light_is_green = True
                        elif current_hardware_lamp == "YELLOW": 
                            light_is_yellow = True
                        else:
                            light_is_red = True
                else:
                    if snap_state == "ALL_RED":
                        light_is_red = True
                    elif lane == snap_lane:
                        if snap_state == "GREEN":   light_is_green = True
                        elif snap_state == "YELLOW": light_is_yellow = True
                    else:
                        light_is_red = True 

                light_str = "RED"
                if light_is_green: light_str = "GREEN"
                elif light_is_yellow: light_str = "YELLOW"

                los = classify_los(counts[lane])
                density = int(occupancy.get(lane, 0.0))

                lanes_payload[lane] = {
                    "count": counts[lane],
                    "density": density,
                    "light": light_str,
                    "los": los
                }

            mode_str = "AUTO"
            if manual_override:
                if emergency_active:
                    mode_str = "EMERGENCY"
                elif hazard_active:
                    mode_str = "HAZARD"
                else:
                    mode_str = "MANUAL"

            body = {
                "mode": mode_str,
                "activeLane": snap_lane,
                "weather": "RAINY" if rain_detected else "SUNNY",
                "remainingSecs": disp_remain,
                "greenDuration": snap_green,
                "lanes": lanes_payload
            }

            headers = {
                "Authorization": f"Bearer {NODE_API_KEY}",
                "Content-Type": "application/json",
                "Accept": "application/json"
            }
            requests.post(control_url, json=body, headers=headers, timeout=2.0, verify=False)
        except Exception as e:
            print(f"[STAP] ⚠️ Real-time sync to Cloud Hub failed: {e}")
        time.sleep(1.0)

CSV_PATH = os.path.join(CURRENT_RUN_DIR, "traffic_summary.csv")
with open(CSV_PATH, mode='w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(["STAP DYNAMIC REAL-TIME TIMESTAMED LEDGER LOG"])
    writer.writerow(["Session Start Initialization Time", datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
    writer.writerow([])

threading.Thread(target=keepalive_thread, daemon=True).start()
threading.Thread(target=hub_heartbeat_thread, daemon=True).start()
threading.Thread(target=hub_realtime_sync_thread, daemon=True).start()
threading.Thread(target=run_flask_server, daemon=True).start() 
print("[STAP] ✅ Per-lane casting nodes are active on Local LAN Port 5000")

_first_lane = PHASE_ORDER[0]
_first_green = compute_green_time(_first_lane, rain_detected)
start_green(_first_lane, _first_green)

# --- INITIALIZE MULTI-WINDOW SYSTEMS ---
cv2.namedWindow("STAP Local Engine Monitor", cv2.WINDOW_NORMAL)
cv2.namedWindow("STAP Analytics Dashboard", cv2.WINDOW_NORMAL)
cv2.resizeWindow("STAP Analytics Dashboard", 650, 420)

GRID_WIDTH  = CAM_WIDTH * 2
GRID_HEIGHT = CAM_HEIGHT * 2
RECORDING_PATH = os.path.join(CURRENT_RUN_DIR, "session_recording.mp4")
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(RECORDING_PATH, fourcc, float(TARGET_FPS), (GRID_WIDTH, GRID_HEIGHT))
print(f"[STAP] 🎥 Video Logging Pipeline Engaged -> {RECORDING_PATH}")

# =============================================================
# 10. MAIN PROCESS AND COMPOSITION LOOP (30 FPS)
# =============================================================
try:
    while True:
        t_loop = time.time()
        read_serial_incoming()
        is_offline = (ser is None) or (t_loop - last_comm_time > DATA_TIMEOUT)

        with frame_lock:
            imgs = [f.copy() if f is not None else None for f in latest_frames]

        with result_lock:
            local_boxes    = {k: list(v) for k, v in cached_boxes.items()}
            local_counts   = vehicle_counts.copy()
            local_statuses = lane_statuses.copy()
            local_occupancy = lane_live_occupancy_pct.copy()
            local_manual_lights = manual_lane_lights.copy() 

        with phase_lock:
            snap_lane    = PHASE_ORDER[current_phase_idx]
            snap_state   = phase_state
            snap_green   = committed_green
            snap_g_start = green_start_time
            snap_y_start = yellow_start_time
            snap_ar_start= all_red_start_time

        # Ensure every lane has a frame. If None, create an elegant neon offline placeholder frame
        # so that a single slow or offline stream doesn't freeze the loop or stop telemetry/phasing.
        for idx, lane in enumerate(LANE_NAMES):
            if imgs[idx] is None:
                placeholder = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
                cv2.rectangle(placeholder, (5, 5), (CAM_WIDTH-5, CAM_HEIGHT-5), (20, 20, 25), -1)
                cv2.rectangle(placeholder, (5, 5), (CAM_WIDTH-5, CAM_HEIGHT-5), ROI_COLORS[lane], 2)
                cv2.putText(placeholder, f"{lane} CONNECTING...", (160, CAM_HEIGHT//2 - 20), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.7, (100, 100, 255), 2, cv2.LINE_AA)
                cv2.putText(placeholder, "WAITING FOR VIDEO DECODER FEED", (110, CAM_HEIGHT//2 + 20), 
                            cv2.FONT_HERSHEY_DUPLEX, 0.5, (120, 120, 130), 1, cv2.LINE_AA)
                imgs[idx] = placeholder

        now = time.time()
        if snap_state == "GREEN":
            disp_remain = max(0, snap_green - int(now - snap_g_start))
        elif snap_state == "YELLOW":
            disp_remain = max(0, YELLOW_TIME - int(now - snap_y_start if snap_y_start > 0 else 0.0))
        else:
            disp_remain = max(0, ALL_RED_TIME - int(now - snap_ar_start if snap_ar_start > 0 else 0.0))

        display_greens = {lane: compute_green_time(lane, rain_detected) for lane in LANE_NAMES}
        display_greens[snap_lane] = snap_green

        # --- DYNAMIC HAZARD BLINK OVERRIDE FILTER PATTERN CHANNELS ---
        if all(local_manual_lights.get(l) == "YELLOW" for l in LANE_NAMES):
            last_hazard_blink_time = now
            
        is_hazard_pattern_active = (now - last_hazard_blink_time < 1.5)

        drawn = list(imgs)
        for idx, lane in enumerate(LANE_NAMES):
            fr = drawn[idx]
            lane_color = ROI_COLORS[lane]

            overlay = fr.copy()
            cv2.fillPoly(overlay, [ROI_POLYGONS[lane]], lane_color)
            cv2.addWeighted(overlay, ROI_ALPHA, fr, 1.0 - ROI_ALPHA, 0, fr)
            cv2.polylines(fr, [ROI_POLYGONS[lane]], isClosed=True, color=lane_color, thickness=2)

            # --- BIG APPROACH IDENTIFIER TEXT HUD LAYER ---
            label_text = f"{lane}"
            text_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_DUPLEX, 0.75, 2)[0]
            text_x = (CAM_WIDTH - text_size[0]) // 2
            cv2.putText(fr, label_text, (text_x + 1, 31), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(fr, label_text, (text_x, 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, lane_color, 2, cv2.LINE_AA)

            # --- ENGINE ARCHITECTURE FOR HUD TRAFFIC LIGHT DRAWING PASS ---
            tl_x, tl_y = 580, 15
            br_x, br_y = 625, 140
            cv2.rectangle(fr, (tl_x, tl_y), (br_x, br_y), (35, 35, 35), -1)   
            cv2.rectangle(fr, (tl_x, tl_y), (br_x, br_y), (100, 100, 100), 1) 

            light_is_red    = False
            light_is_yellow = False
            light_is_green  = False

            if manual_override:
                current_hardware_lamp = local_manual_lights.get(lane, "RED")
                if current_hardware_lamp == "GREEN":     
                    light_is_green = True
                elif current_hardware_lamp == "YELLOW": 
                    light_is_yellow = True
                else:                                   
                    if is_hazard_pattern_active:
                        light_is_red = False
                    else:
                        light_is_red = True
            else:
                if snap_state == "ALL_RED":
                    light_is_red = True
                elif lane == snap_lane:
                    if snap_state == "GREEN":   light_is_green = True
                    elif snap_state == "YELLOW": light_is_yellow = True
                else:
                    light_is_red = True 

            red_c    = (602, 36)
            yellow_c = (602, 78)
            green_c  = (602, 119)
            lens_radius = 15

            r_val = (0, 0, 255)   if light_is_red    else (0, 0, 50)
            y_val = (0, 255, 255) if light_is_yellow else (0, 50, 50)
            g_val = (0, 255, 0)   if light_is_green  else (0, 50, 0)

            cv2.circle(fr, red_c, lens_radius, r_val, -1)
            cv2.circle(fr, red_c, lens_radius, (120, 120, 120), 1) 
            cv2.circle(fr, yellow_c, lens_radius, y_val, -1)
            cv2.circle(fr, yellow_c, lens_radius, (120, 120, 120), 1)
            cv2.circle(fr, green_c, lens_radius, g_val, -1)
            cv2.circle(fr, green_c, lens_radius, (120, 120, 120), 1)

            for b in local_boxes[lane]:
                fx1, fy1, fx2, fy2 = b["coords"]
                cv2.rectangle(fr, (fx1, fy1), (fx2, fy2), b["color"], 2)
                cv2.putText(fr, b["label"], (fx1, max(fy1-7, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, b["color"], 1)

            los  = classify_los(local_counts[lane])
            base = BASE_GREEN[lane]
            adj  = display_greens[lane]
            red  = compute_red_time(lane, display_greens)
            
            with analytics_lock:
                lane_lifetime_total = sum(global_analytics_registry[lane].values())

            cv2.putText(fr, f"LOS: {los} | LIVE QUEUE: {local_counts[lane]}",
                        (8, CAM_HEIGHT-42), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255,255,255), 1)
            cv2.putText(fr, f"ROI SPATIAL DENSITY: {local_occupancy[lane]:.1f}%",
                        (8, CAM_HEIGHT-26), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            cv2.putText(fr, f"G:{adj}s(base:{base}s) R:{red}s | LOGGED UNIQUE: {lane_lifetime_total}",
                        (8, CAM_HEIGHT-12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200,255,200), 1)

            is_emg_confirmed = local_statuses[lane] == "EMERGENCY"
            is_emg_charging  = emg_buffer.is_charging(lane)

            if is_emg_confirmed: status_text = f"EMERGENCY PRIORITY"
            elif is_emg_charging:
                streak = emg_buffer.streak_elapsed(lane)
                status_text = f"EMG VEHICLE DETECTED [{streak:.1f}s]"
            elif manual_override:
                status_text = f"MANUAL CONTROL ACTIVE (Driven by ESP32 via Serial)"
                if is_hazard_pattern_active:
                    status_text = f"HAZARD MODE: FLASHING YELLOW"
            else:
                status_text = f"AUTOMATED STATE: RUNNING"

            cv2.putText(fr, status_text, (8, CAM_HEIGHT-58), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1)

        grid = np.vstack((np.hstack((drawn[0], drawn[1])), np.hstack((drawn[2], drawn[3]))))
        mode_label = "OFFLINE/FALLBACK" if is_offline else ("MANUAL OVERRIDE" if manual_override else "AUTO (SMART AI)")
        hud_color  = (0, 255, 255)
        if any(v == "EMERGENCY" for v in local_statuses.values()):
            mode_label = "!!! EMERGENCY PREEMPTION ACTIVE !!!"; hud_color = (0, 0, 255)
        elif rain_detected: mode_label += " + CONDITIONAL RAIN BUFFERS"
        if is_hazard_pattern_active and manual_override:
            mode_label = "HAZARD OVERRIDE PATTERN GENERATED"; hud_color = (0, 165, 255)

        cv2.putText(grid, f"SYSTEM MODE: {mode_label}", (15, grid.shape[0]-50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, hud_color, 2)
        
        if manual_override:
            cv2.putText(grid, f"MANUAL INTERSECTION ROUTING COMMANDS CONTROLLED BY HARDWARE CHANNELS",
                        (15, grid.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 200, 255), 2)
        else:
            cv2.putText(grid, f"ACTIVE PHASE: {snap_lane} | State Clock Remaining: {disp_remain}s | Target Ceiling Green: {snap_green}s",
                        (15, grid.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.62, hud_color, 2)

        video_writer.write(grid)
        cv2.imshow("STAP Local Engine Monitor", grid)

        # =============================================================
        # 10b. GENERATE DEDICATED POP-UP SCOREBOARD UI WINDOW
        # =============================================================
        db_w, db_h = 650, 420
        dashboard_img = np.zeros((db_h, db_w, 3), dtype=np.uint8)
        cv2.rectangle(dashboard_img, (0, 0), (db_w, db_h), (24, 24, 24), -1)
        cv2.rectangle(dashboard_img, (10, 10), (db_w-10, 50), (40, 40, 40), -1)
        
        secs_to_next_log = max(0, int(CSV_LOG_INTERVAL - (now - last_csv_log_time)))
        cv2.putText(dashboard_img, f"STAP LIVE DENSITY ANALYTICS DASHBOARD [CSV LOG: {secs_to_next_log}s]", (20, 36), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 255), 2)

        with analytics_lock:
            classes_to_render = sorted(list(known_classes_seen))
            cv2.putText(dashboard_img, f"{'CLASS':<14}{'NORTH':<8}{'SOUTH':<8}{'EAST':<8}{'WEST':<8}{'TOTAL':<6}", 
                        (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)
            cv2.line(dashboard_img, (15, 95), (db_w-15, 95), (70, 70, 70), 1)

            y_pos = 120
            grand_total_all_lanes = 0
            class_column_totals = collections.defaultdict(int)

            for cls_name in classes_to_render:
                n_v = global_analytics_registry["NORTH"].get(cls_name, 0)
                s_v = global_analytics_registry["SOUTH"].get(cls_name, 0)
                e_v = global_analytics_registry["EAST"].get(cls_name, 0)
                w_v = global_analytics_registry["WEST"].get(cls_name, 0)
                row_sum = n_v + s_v + e_v + w_v
                
                class_column_totals["NORTH"] += n_v
                class_column_totals["SOUTH"] += s_v
                class_column_totals["EAST"] += e_v
                class_column_totals["WEST"] += w_v
                grand_total_all_lanes += row_sum

                row_text = f"{cls_name[:11]:<14}{n_v:<8}{s_v:<8}{e_v:<8}{w_v:<8}{row_sum:<6}"
                cv2.putText(dashboard_img, row_text, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
                y_pos += 22
                if y_pos > db_h - 90: break 

            cv2.line(dashboard_img, (15, db_h-75), (db_w-15, db_h-75), (55, 55, 55), 1)
            
            density_row = f"{'LIVE DENSITY':<14}"\
                          f"{local_occupancy['NORTH']:.1f}%    "\
                          f"{local_occupancy['SOUTH']:.1f}%    "\
                          f"{local_occupancy['EAST']:.1f}%    "\
                          f"{local_occupancy['WEST']:.1f}%    "
            cv2.putText(dashboard_img, density_row, (20, db_h-55), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 215, 255), 1)

            cv2.line(dashboard_img, (15, db_h-45), (db_w-15, db_h-45), (0, 255, 255), 1)
            footer_text = f"{'GRAND TOTAL':<14}"\
                          f"{class_column_totals['NORTH']:<8}"\
                          f"{class_column_totals['SOUTH']:<8}"\
                          f"{class_column_totals['EAST']:<8}"\
                          f"{class_column_totals['WEST']:<8}"\
                          f"{grand_total_all_lanes:<6}"
            cv2.putText(dashboard_img, footer_text, (20, db_h-25), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

        cv2.imshow("STAP Analytics Dashboard", dashboard_img)

        # =============================================================
        # 10c. AUTOMATED RUN PERIODIC LEDGER EXPORT
        # =============================================================
        if now - last_csv_log_time >= CSV_LOG_INTERVAL:
            last_csv_log_time = now
            print(f"[STAP] 🕒 Log Interval Triggered. Appending active density metrics data to ledger...")
            
            with analytics_lock:
                try:
                    with open(CSV_PATH, mode='a', newline='', encoding='utf-8') as f:
                        writer = csv.writer(f)
                        current_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        
                        writer.writerow([f"--- INTERVAL RECORDING SNAPSHOT [{current_timestamp}] ---"])
                        writer.writerow(["Lane Approach"] + classes_to_render + ["Cumulative Total", "Live Area Density Occupancy %"])
                        
                        for lane in LANE_NAMES:
                            lane_row = [lane]
                            lane_sum = 0
                            for cls_name in classes_to_render:
                                val = global_analytics_registry[lane].get(cls_name, 0)
                                lane_row.append(val)
                                lane_sum += val
                            lane_row.append(lane_sum)
                            lane_row.append(f"{local_occupancy[lane]:.2f}%")
                            writer.writerow(lane_row)
                            
                        writer.writerow(["Intersection Cumulative Unique Vehicles Sum:", grand_total_all_lanes])
                        writer.writerow([])
                    print("[STAP] ✅ Density interval logging cycle complete.")
                except Exception as csv_append_err:
                    print(f"[STAP] ⚠️ Delayed appending metrics block: {csv_append_err}")

        for idx, lane in enumerate(LANE_NAMES):
            with lane_stream_locks[lane]:
                global_lane_frames[lane] = drawn[idx].copy()

        if not manual_override:
            if snap_state == "GREEN":
                emg = emergency_lane()
                if emg and emg != snap_lane: 
                    print(f"[STAP] 🚨 Confirmed Emergency vehicle on {emg} approach sustained for 3s. Preempting active lane.")
                    start_yellow(snap_lane)
                elif now - snap_g_start >= snap_green: 
                    start_yellow(snap_lane)
            elif snap_state == "YELLOW":
                if now - snap_y_start >= YELLOW_TIME: start_all_red()
            elif snap_state == "ALL_RED":
                if now - snap_ar_start >= ALL_RED_TIME: advance_phase()

        key = cv2.waitKey(1) & 0xFF
        if (key == ord('q') or 
            cv2.getWindowProperty("STAP Local Engine Monitor", cv2.WND_PROP_VISIBLE) < 1 or
            cv2.getWindowProperty("STAP Analytics Dashboard", cv2.WND_PROP_VISIBLE) < 1):
            break
        
        time.sleep(max(0.001, (1.0/TARGET_FPS) - (time.time() - t_loop)))

finally:
    # =============================================================
    # 11. CLEANUP EXITS & FINAL ABSOLUTE GRAND TOTALS SUMMARY REPORT
    # =============================================================
    print("\n[STAP] 🛑 Shutdown Signal Intercepted. Closing background modules cleanly...")
    
    for r in readers: r.running = False
    ai_core.running = False
    video_writer.release()
    cv2.destroyAllWindows()
    
    print(f"[STAP] 📊 Compiling absolute density data frames into master sheet ledger -> {CSV_PATH}")
    
    with analytics_lock:
        all_detected_classes = sorted(list(known_classes_seen))
        try:
            with open(CSV_PATH, mode='a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["========================================================="])
                writer.writerow(["FINAL INTERSECTION REPORT SUMMARY MATRIX"])
                writer.writerow(["Session Termination Completed Clock", datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
                writer.writerow([])
                
                header_row = ["Approach Lane Name"] + all_detected_classes + ["Absolute Grand Unique Count", "Final Snapshot Road Density"]
                writer.writerow(header_row)
                
                class_grand_totals = collections.defaultdict(int)
                intersection_grand_total = 0
                
                for lane in LANE_NAMES:
                    row = [lane]
                    lane_sum = 0
                    for cls_name in all_detected_classes:
                        val = global_analytics_registry[lane].get(cls_name, 0)
                        row.append(val)
                        lane_sum += val
                        class_grand_totals[cls_name] += val
                    row.append(lane_sum)
                    row.append(f"{local_occupancy[lane]:.2f}%")
                    intersection_grand_total += lane_sum
                    writer.writerow(row)
                
                totals_row = ["TOTAL INTERSECTION CORRIDOR"]
                for cls_name in all_detected_classes:
                    totals_row.append(class_grand_totals[cls_name])
                totals_row.append(intersection_grand_total)
                writer.writerow(totals_row)
                
            print(f"[STAP] ✅ Master Data Export successful. Total unique intersection vehicles logged: {intersection_grand_total}")
        except Exception as csv_err:
            print(f"[STAP] ❌ Failed to compile final absolute metrics spreadsheet rows: {csv_err}")
            
    print(f"[STAP] Run complete. Check folder path '{CURRENT_RUN_DIR}' for all generated session files.")
