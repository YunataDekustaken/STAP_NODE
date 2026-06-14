"""
STAP: Smart Traffic Automation Program
=======================================
v12.0 — Standards-Compliant Hybrid Sequential Micro-Phasing Architecture.

CHANGES FROM v11.1:
  │  FIX: Reduced MAX_ADJUSTMENT to 10s and lowered congestion safety threshold to 20.
  │  FIX: Conditional rain buffer applied only to highly congested approaches (LOS D/E/F).
  │  FIX: Modified Fix 4 to keep fixed predictable phase rotation (protects hardware counters)
  │       but implements Micro-Phasing (7-10s) for low-occupancy lanes to maximize throughput.
  │  FIX: Added explicit 2-second All-Red Clearance Interval satisfying international standards.
"""

from ultralytics import YOLO
import cv2
import time
import serial
import numpy as np
import requests
import threading
import torch
import os
import collections
from datetime import datetime
from flask import Flask, Response, request, jsonify

# =============================================================
# 1. CONFIGURATION & FLASK SERVER BOOT
# =============================================================
app = Flask(__name__)

SERIAL_PORT = "COM11"
BAUD_RATE   = 115200

MODEL_PATH = r"C:\Users\Raphael\Desktop\YOLO\mayor_gil6\runs\detect\train\weights\best.pt"

EMERGENCY_CLASS_IDS = [0, 5, 9]   # ambulance=0, firetruck=5, police=9
VEHICLE_CLASS_IDS   = [1, 2, 3, 4, 6, 7, 8, 10, 11, 12, 13]

LANE_NAMES  = ["NORTH", "SOUTH", "EAST", "WEST"]
PHASE_ORDER = ["NORTH", "SOUTH", "EAST", "WEST"]

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

# 💡 NOTE FOR FUTURE ROI UPGRADES:
# -----------------------------------------------------------------------------------
# To update these ROIs later using coordinates extracted from your video analyzer or tool,
# format them exactly as standard NumPy arrays containing nested coordinate lists: [[X1, Y1], [X2, Y2], ...].
# Ensure you enforce `dtype=np.int32` so the OpenCV detection functions do not throw calculation exceptions.
# They can have any number of vertex points (4 points, 7 points, 9 points, etc.).
# -----------------------------------------------------------------------------------
ROI_POLYGONS = {
    "WEST": np.array([[683, 1534], [1853, 427], [2526, 424], [2748, 1605]], dtype=np.int32),
    
    "NORTH": np.array([[2173, 2159], [2109, 999], [2065, 450], [2017, 137], [1779, 134], 
                       [1567, 464], [991, 1263], [497, 1880], [761, 2155]], dtype=np.int32),
    
    "EAST": np.array([[7, 1713], [-1, 1181], [683, 528], [932, 320], [1181, 175], 
                      [1835, 175], [2303, 1735]], dtype=np.int32),
    
    "SOUTH": np.array([[579, 897], [601, 528], [869, 318], [1250, 310], [1361, 927]], dtype=np.int32)
}

# Engineered base green times (from traffic study)
BASE_GREEN = {"NORTH": 50, "SOUTH": 50, "EAST": 39, "WEST": 35}

# International Standards Compliance Configuration Blocks
YELLOW_TIME       = 3     # Visual countdown baseline
ALL_RED_TIME      = 2     # Vienna Convention clearance buffer
CONGESTION_CEILING= 20    # Optimization safety check trigger threshold (Fix 2)

MIN_GREEN       = {lane: max(7,  int(BASE_GREEN[lane] * 0.40)) for lane in LANE_NAMES}
MAX_GREEN       = {lane: min(65, int(BASE_GREEN[lane] * 1.30)) for lane in LANE_NAMES}
MAX_ADJUSTMENT  = 10     # Tighter extension bounding rule (Fix 1)

LOS_THRESHOLDS = [("A",0,1),("B",2,3),("C",4,6),("D",7,10),("E",11,15),("F",16,999)]
LOS_DELTA      = {"A":-10,"B":-6,"C":0,"D":+6,"E":+8,"F":+10}

PING_INTERVAL = 0.4

# Detection Stability Tuning
CONF_THRESHOLD            = 0.50   
EMERGENCY_SUSTAIN_SECONDS = 3.0    
COUNT_SMOOTH_WINDOW       = 8      

# STAP Hub Public Cloud Endpoints
NODE_API_KEY       = "node_alpha_J7FVxdRBqwCBWQSdiKBN742lMHuEPX5A"
STAP_HUB_URL       = "https://your-free-webapp.render.com/api/v1/snapshots"
STAP_HEARTBEAT_URL = "https://your-free-webapp.render.com/api/v1/heartbeat"
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
    print("[STAP] ⚠️  CPU Mode — install CUDA PyTorch for better performance")

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

phase_lock         = threading.Lock()
current_phase_idx  = 0
phase_state        = "GREEN" # State space: GREEN, YELLOW, ALL_RED
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
# 5. AI INFERENCE CORE (Isolated Model Handles for Tracking)
# =============================================================
class BackgroundAIProcessor(threading.Thread):
    def __init__(self, model_path, device):
        super().__init__(daemon=True)
        self.models = {lane: YOLO(model_path) for lane in LANE_NAMES}
        self.device  = device
        self.labels  = self.models["NORTH"].names
        self.running = True
        self.half    = device != "cpu"
        
        if device != "cpu":
            for lane in LANE_NAMES:
                self.models[lane].to(device)
            print(f"[STAP] ✅ 4 Isolated Tracker Streams initialized on GPU (device={device})")
        else:
            print("[STAP] ⚠️  YOLO models on CPU")

    def run(self):
        global cached_boxes, vehicle_counts, lane_statuses
        while self.running:
            temp_counts = {l: 0 for l in LANE_NAMES}
            temp_statuses = {l: "CLEAR" for l in LANE_NAMES}
            temp_boxes = {l: [] for l in LANE_NAMES}

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
                        temp_boxes[lane].append({
                            "coords": (bx1, by1, bx2, by2),
                            "label" : f"{self.labels.get(cls_id, 'Vehicle')} {conf:.2f}",
                            "color" : (0, 0, 255) if is_emg else (0, 255, 0),
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
last_comm_time  = time.time()

def read_serial_incoming():
    global rain_detected, manual_override, last_comm_time
    if ser and ser.in_waiting:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line and "RAIN:" in line and "MODE:" in line:
                last_comm_time = time.time()
                for part in line.split(","):
                    if part.startswith("RAIN:"):
                        rain_detected = (part.split(":")[1] == "1")
                    elif part.startswith("MODE:"):
                        manual_override = (part.split(":")[1] == "MANUAL")
        except Exception: pass

def classify_los(count: int) -> str:
    for grade, lo, hi in LOS_THRESHOLDS:
        if lo <= count <= hi: return grade
    return "F"

def compute_green_time(lane: str, rain: bool) -> int:
    """
    Optimized Backend Adaptive Green Time Engine (Middle Ground Logic).
    Ensures safe minimum flow boundaries while preventing starvation bounds.
    """
    with result_lock: 
        current_queue = vehicle_counts[lane]
        # Evaluate global pressure across all competing approaches (Fix 2 threshold check)
        total_intersection_backpressure = sum(vehicle_counts[l] for l in LANE_NAMES if l != lane)
    
    # Philippine Setting Micro-Phasing Optimization Layer (Protects rhythm + countdowns)
    if current_queue <= 2:
        # Assign a tightened micro-minimum (7-10s) to clear out the minor approach quickly
        rain_mod = 1.20 if rain else 1.0
        return max(7, min(10, int(7 * rain_mod)))
        
    los   = classify_los(current_queue)
    delta = max(-MAX_ADJUSTMENT, min(MAX_ADJUSTMENT, LOS_DELTA[los]))
    
    # If backpressure across other lanes is building up (>20), damp greedy extensions
    if total_intersection_backpressure >= CONGESTION_CEILING and delta > 0:
        delta = int(delta * 0.5) # Cut active extension down by 50% to prevent starvation delays
        
    green = BASE_GREEN[lane] + delta
    
    # Conditional Weather Friction Addition (Fix 3 - Only targets heavy congestion levels)
    if rain and los in ["D", "E", "F"]:
        green += 5 # Slow discharge clearance window extension
        
    return max(MIN_GREEN[lane], min(MAX_GREEN[lane], green))

def compute_red_time(lane: str, greens: dict) -> int:
    # Account for yellow + international standard all-red clearance values cumulative tracking
    return sum(greens[l] + YELLOW_TIME + ALL_RED_TIME for l in PHASE_ORDER if l != lane)

def emergency_lane():
    with result_lock:
        for lane, status in lane_statuses.items():
            if status == "EMERGENCY": return lane
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
                requests.post(STAP_HUB_URL, json=body, headers=headers, timeout=1.5)
        except Exception as e:
            print(f"[STAP] Cloud synchronization delay: {e}")
    threading.Thread(target=_post, daemon=True).start()

def hub_heartbeat_thread():
    while True:
        try:
            requests.post(
                STAP_HEARTBEAT_URL,
                headers={"Authorization": f"Bearer {NODE_API_KEY}", "Accept": "application/json"},
                timeout=1.5
            )
        except Exception: pass
        time.sleep(30)

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

    global manual_override, current_phase_idx, phase_state, green_start_time, committed_green

    data = request.get_json(force=True)
    mode = data.get('mode', '').lower()

    if mode not in ['auto', 'manual', 'hazard']:
        return jsonify({'success': False, 'message': 'Invalid mode. Use auto, manual, or hazard.'}), 400

    if mode == 'auto':
        manual_override = False
        send_to_esp32('MODE:AUTO')
        with phase_lock: lane = PHASE_ORDER[current_phase_idx]
        green = compute_green_time(lane, rain_detected)
        start_green(lane, green)

    elif mode == 'manual':
        manual_override = True
        send_to_esp32('MODE:MANUAL')

    elif mode == 'hazard':
        manual_override = True
        send_to_esp32('MODE:HAZARD')
        for lane in LANE_NAMES:
            send_to_esp32(f'HAZARD:{lane}')

    return jsonify({'success': True, 'mode': mode})

@app.route('/control/light', methods=['POST', 'OPTIONS'])
def control_light():
    if request.method == 'OPTIONS': return jsonify({}), 200
    if not manual_override:
        return jsonify({'success': False, 'message': 'Node must be in manual or hazard mode first.'}), 422

    data  = request.get_json(force=True)
    lane  = data.get('lane', '').upper()
    state = data.get('state', '').lower()

    if lane not in LANE_NAMES or state not in ['red', 'yellow', 'green']:
        return jsonify({'success': False, 'message': 'Invalid parameters parameter fields.'}), 400

    send_to_esp32(f'MANUAL_LIGHT:{lane},{state.upper()}')
    return jsonify({'success': True, 'lane': lane, 'state': state})

@app.route('/control/emergency', methods=['POST', 'OPTIONS'])
def control_emergency():
    if request.method == 'OPTIONS': return jsonify({}), 200

    data = request.get_json(force=True)
    lane = data.get('lane', '').upper()

    if lane not in LANE_NAMES:
        return jsonify({'success': False, 'message': f'Invalid lane. Use: {LANE_NAMES}'}), 400

    start_yellow(PHASE_ORDER[current_phase_idx])
    time.sleep(0.1)
    start_green(lane, compute_green_time(lane, rain_detected))
    send_to_esp32(f'EMERGENCY_OVERRIDE:{lane}')

    return jsonify({'success': True, 'emergency_lane': lane})

@app.route('/status', methods=['GET'])
def get_status():
    with result_lock:
        counts   = vehicle_counts.copy()
        statuses = lane_statuses.copy()

    with phase_lock:
        active_lane   = PHASE_ORDER[current_phase_idx]
        current_state = phase_state
        green_dur     = committed_green

    now = time.time()
    if current_state == 'GREEN':
        elapsed   = now - green_start_time
        remaining = max(0, green_dur - int(elapsed))
    elif current_state == 'YELLOW':
        elapsed   = now - yellow_start_time if yellow_start_time > 0 else 0
        remaining = max(0, YELLOW_TIME - int(elapsed))
    else:
        elapsed   = now - all_red_start_time if all_red_start_time > 0 else 0
        remaining = max(0, ALL_RED_TIME - int(elapsed))

    los_per_lane = {lane: classify_los(counts[lane]) for lane in LANE_NAMES}

    return jsonify({
        'active_lane':    active_lane,
        'phase_state':    current_state,
        'remaining_secs': remaining,
        'green_duration': green_dur,
        'mode':           'manual' if manual_override else 'auto',
        'rain':           rain_detected,
        'vehicle_counts': counts,
        'los':            los_per_lane,
        'lane_statuses':  statuses,
    })

def run_flask_server():
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)

# =============================================================
# 9. TIMING PHASE SYSTEM TRANSITIONS (State Machine Engine)
# =============================================================
def start_yellow(lane: str):
    global phase_state, yellow_start_time
    with phase_lock:
        phase_state       = "YELLOW"
        yellow_start_time = time.time()
    send_to_esp32(f"YELLOW:{lane}")
    send_to_esp32(f"DISPLAY:YELLOW,{YELLOW_TIME}")

def start_all_red():
    global phase_state, all_red_start_time
    with phase_lock:
        phase_state        = "ALL_RED"
        all_red_start_time = time.time()
    # Issue absolute clearance state broadcast command over to serial line channels
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
    send_to_esp32(f"PHASE:{next_lane},DURATION:{duration}")
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
        send_to_esp32(f"PING:{active}")
        hub_tick += 1
        if hub_tick >= HUB_INTERVAL_TICKS:
            hub_tick = 0
            post_to_hub()

# Launch Core Keepalives & Flask Broadcast Channels
threading.Thread(target=keepalive_thread, daemon=True).start()
threading.Thread(target=hub_heartbeat_thread, daemon=True).start()
threading.Thread(target=run_flask_server, daemon=True).start() 
print("[STAP] ✅ Per-lane casting nodes are active on Local LAN Port 5000")

_first_lane  = PHASE_ORDER[0]
_first_green = compute_green_time(_first_lane, rain_detected)
start_green(_first_lane, _first_green)

cv2.namedWindow("STAP Local Engine Monitor", cv2.WINDOW_NORMAL)

# =============================================================
# 10. MAIN PROCESS AND COMPOSITION LOOP (30 FPS)
# =============================================================
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

    with phase_lock:
        snap_lane    = PHASE_ORDER[current_phase_idx]
        snap_state   = phase_state
        snap_green   = committed_green
        snap_g_start = green_start_time
        snap_y_start = yellow_start_time
        snap_ar_start= all_red_start_time

    if any(f is None for f in imgs):
        time.sleep(0.01); continue

    now = time.time()
    if snap_state == "GREEN":
        green_elapsed  = now - snap_g_start
        green_remain   = max(0, snap_green - int(green_elapsed))
        disp_remain    = green_remain
    elif snap_state == "YELLOW":
        yellow_elapsed = now - snap_y_start if snap_y_start > 0 else 0.0
        disp_remain    = max(0, YELLOW_TIME - int(yellow_elapsed))
    else:
        all_red_elapsed= now - snap_ar_start if snap_ar_start > 0 else 0.0
        disp_remain    = max(0, ALL_RED_TIME - int(all_red_elapsed))

    display_greens = {lane: compute_green_time(lane, rain_detected) for lane in LANE_NAMES}
    display_greens[snap_lane] = snap_green

    drawn = list(imgs)
    for idx, lane in enumerate(LANE_NAMES):
        fr = drawn[idx]
        
        cv2.polylines(fr, [ROI_POLYGONS[lane]], isClosed=True, color=(255,165,0), thickness=2)

        for b in local_boxes[lane]:
            fx1, fy1, fx2, fy2 = b["coords"]
            cv2.rectangle(fr, (fx1, fy1), (fx2, fy2), b["color"], 2)
            cv2.putText(fr, b["label"], (fx1, max(fy1-7, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, b["color"], 1)

        los  = classify_los(local_counts[lane])
        base = BASE_GREEN[lane]
        adj  = display_greens[lane]
        red  = compute_red_time(lane, display_greens)
        cv2.putText(fr, f"LOS:{los} V:{local_counts[lane]} G:{adj}s(b:{base}s) R:{red}s",
                    (8, CAM_HEIGHT-12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255,255,255), 1)

        is_emg_confirmed = local_statuses[lane] == "EMERGENCY"
        is_emg_charging  = emg_buffer.is_charging(lane)

        if is_emg_confirmed: sc = (0, 0, 255); status_text = f"{lane}: EMERGENCY"
        elif is_emg_charging:
            sc = (0, 165, 255); streak = emg_buffer.streak_elapsed(lane)
            status_text = f"{lane}: EMG? [{streak:.1f}/{EMERGENCY_SUSTAIN_SECONDS}s]"
        elif lane == snap_lane:
            sc = (0, 255, 0) if snap_state == "GREEN" else ((0, 255, 255) if snap_state == "YELLOW" else (0, 0, 255))
            status_text = f"{lane}: [{snap_state}]"
        else:
            sc = (140, 140, 140); status_text = f"{lane}: {local_statuses[lane]}"

        cv2.putText(fr, status_text, (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, sc, 2)

    grid = np.vstack((np.hstack((drawn[0], drawn[1])), np.hstack((drawn[2], drawn[3]))))
    hud_color  = (0, 255, 255)
    mode_label = "OFFLINE/FALLBACK" if is_offline else ("MANUAL OVERRIDE" if manual_override else "AUTO (SMART AI)")
    if any(v == "EMERGENCY" for v in local_statuses.values()):
        mode_label = "!!! EMERGENCY PREEMPTION ACTIVE !!!"; hud_color = (0, 0, 255)
    elif rain_detected: mode_label += " + CONDITIONAL RAIN BUFFERS"

    cv2.putText(grid, f"SYSTEM MODE: {mode_label}", (15, grid.shape[0]-50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, hud_color, 2)
    cv2.putText(grid, f"ACTIVE PHASE: {snap_lane} [{snap_state}] | State Clock Remaining: {disp_remain}s | Target Ceiling Green: {snap_green}s",
                (15, grid.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.62, hud_color, 2)

    cv2.imshow("STAP Local Engine Monitor", grid)

    for idx, lane in enumerate(LANE_NAMES):
        with lane_stream_locks[lane]:
            global_lane_frames[lane] = drawn[idx].copy()

    # Hardware Automation Logic Execution Block (State Transitions Cascade Chain)
    if not manual_override:
        if snap_state == "GREEN":
            emg = emergency_lane()
            if emg and emg != snap_lane: start_yellow(snap_lane)
            elif green_elapsed >= snap_green: start_yellow(snap_lane)
        elif snap_state == "YELLOW":
            if now - snap_y_start >= YELLOW_TIME: start_all_red()
        elif snap_state == "ALL_RED":
            if now - snap_ar_start >= ALL_RED_TIME: advance_phase()

    cv2.waitKey(1)
    time.sleep(max(0.001, (1.0/TARGET_FPS) - (time.time() - t_loop)))