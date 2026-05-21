from ultralytics import YOLO
import cv2
import torch
import subprocess
import platform
import serial
import re
from collections import defaultdict
import time

# ══════════════════════════════════════════════════════════════════════════════
#  AUTO HARDWARE DETECTION  (no third-party GPU libs required)
# ══════════════════════════════════════════════════════════════════════════════

def _query_windows_gpu():
    """Use built-in Windows WMI to get GPU name — no extra install needed."""
    try:
        out = subprocess.check_output(
            ["wmic", "path", "win32_VideoController", "get", "name"],
            stderr=subprocess.DEVNULL, timeout=4
        ).decode(errors="ignore")
        names = [l.strip() for l in out.splitlines()
                 if l.strip() and l.strip().lower() != "name"]
        return names
    except Exception:
        return []


def get_device():
    """
    Detection priority:
      1. NVIDIA CUDA  → torch.cuda (already in PyTorch)
      2. AMD / Intel  → detected via WMI name, runs on CPU path (best-effort)
      3. CPU fallback
    Returns (device_type, friendly_name, vram_gb)
    """
    if torch.cuda.is_available():
        name    = torch.cuda.get_device_name(0)
        vram_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"[Device] ✅ NVIDIA CUDA GPU: {name} ({vram_gb:.1f} GB VRAM)")
        return "cuda", name, vram_gb

    if platform.system() == "Windows":
        gpu_names = _query_windows_gpu()
        for name in gpu_names:
            name_lower = name.lower()
            if "nvidia" in name_lower:
                print(f"[Device] ⚠️  NVIDIA GPU found ({name}) but CUDA is NOT installed.")
                print("[Device]    Install CUDA-enabled PyTorch:")
                print("[Device]    https://pytorch.org/get-started/locally/")
                print("[Device]    Falling back to CPU for now.")
                return "cpu_nvidia_hint", name, 0
            if "amd" in name_lower or "radeon" in name_lower:
                print(f"[Device] ℹ️  AMD GPU detected: {name}")
                print("[Device]    Tip: try ONNX Runtime with DirectML for AMD acceleration:")
                print("[Device]    pip install onnxruntime-directml")
                print("[Device]    Falling back to CPU.")
                return "cpu_amd_hint", name, 0
            if "intel" in name_lower and ("arc" in name_lower or "xe" in name_lower):
                print(f"[Device] ℹ️  Intel Arc/Xe GPU detected: {name}")
                print("[Device]    For Intel GPU acceleration install OpenVINO:")
                print("[Device]    pip install openvino ultralytics[openvino]")
                print("[Device]    Falling back to CPU.")
                return "cpu_intel_hint", name, 0

    cpu_count = torch.get_num_threads()
    print(f"[Device] 🖥️  CPU mode  ({cpu_count} threads available)")
    return "cpu", "CPU", 0


device_type, device_name, vram_gb = get_device()

# ══════════════════════════════════════════════════════════════════════════════
#  IP ADDRESS CONFIG
# ══════════════════════════════════════════════════════════════════════════════

def get_drone_ip(com_port="COM5", baud_rate=115200, timeout_sec=30, max_attempts=None):
    """
    Read serial data from drone and extract router IP address.
    Used only to confirm the drone is connected; main.py reads video from
    the local Flask server (127.0.0.1:5000), not from the drone IP directly.
    """
    try:
        ser = serial.Serial(com_port, baud_rate, timeout=1)
    except serial.SerialException as e:
        print(f"[IP Error] Failed to open serial port {com_port}: {e}")
        return None

    start_time = time.time()
    attempt    = 0

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                print(f"[IP Timeout] No IP found within {timeout_sec}s. Returning None.")
                return None
            if max_attempts is not None and attempt >= max_attempts:
                print(f"[IP Attempts] Max attempts ({max_attempts}) reached. Returning None.")
                return None
            try:
                line = ser.readline().decode(errors="ignore").strip()
                attempt += 1
                if not line:
                    continue
                match = re.search(r'(\d+\.\d+\.\d+\.\d+)', line)
                if match:
                    ip = match.group(1)
                    print(f"[IP Found] Got drone IP: {ip} (after {elapsed:.1f}s)")
                    return ip
            except Exception as e:
                print(f"[IP Read Error] {e}")
                continue
    finally:
        ser.close()


# ══════════════════════════════════════════════════════════════════════════════
#  ADAPTIVE CONFIG
# ══════════════════════════════════════════════════════════════════════════════

def build_config(device_type, vram_gb):
    if device_type == "cuda":
        if vram_gb >= 8:
            return dict(model="yolov8x.pt", imgsz=960,  conf=0.50, iou=0.40,
                        skip=1, smooth=2, half=True,  device="cuda",
                        cam_w=1920, cam_h=1080, tier="High-end GPU (CUDA)")
        elif vram_gb >= 4:
            return dict(model="yolov8m.pt", imgsz=640,  conf=0.50, iou=0.40,
                        skip=1, smooth=2, half=True,  device="cuda",
                        cam_w=1280, cam_h=720,  tier="Mid-range GPU (CUDA)")
        else:
            return dict(model="yolov8s.pt", imgsz=640,  conf=0.45, iou=0.45,
                        skip=1, smooth=3, half=True,  device="cuda",
                        cam_w=1280, cam_h=720,  tier="Low-VRAM GPU (CUDA)")

    return dict(model="yolov8n.pt", imgsz=320,  conf=0.45, iou=0.45,
                skip=2, smooth=3, half=False, device="cpu",
                cam_w=640,  cam_h=480,  tier=f"CPU ({device_name})")


cfg = build_config(device_type, vram_gb)

print(f"\n[Config] Tier        : {cfg['tier']}")
print(f"[Config] Model       : {cfg['model']}")
print(f"[Config] Img size    : {cfg['imgsz']}")
print(f"[Config] FP16 (half) : {cfg['half']}")
print(f"[Config] Frame skip  : every {cfg['skip']} frame(s)")
print(f"[Config] Resolution  : {cfg['cam_w']}×{cfg['cam_h']}\n")

# ══════════════════════════════════════════════════════════════════════════════
#  LOAD MODEL
# ══════════════════════════════════════════════════════════════════════════════

try:
    model = YOLO(cfg["model"])
    model.to(cfg["device"])
    print(f"[Model] ✅ Loaded {cfg['model']} on {cfg['device']}")
except Exception as e:
    print(f"[Model Error] Failed to load {cfg['model']}: {e}")
    exit(1)

HUMAN_CLASS = 0

# ══════════════════════════════════════════════════════════════════════════════
#  VIDEO STREAM
#  BUG 2 FIX: main.py reads from the *local* Flask MJPEG server (127.0.0.1:5000)
#             NOT from the drone/router IP.  The drone IP from serial is only
#             used to confirm the ESP32 is live on the network.
# ══════════════════════════════════════════════════════════════════════════════

STREAM_PORT       = 5000
MAX_STREAM_RETRIES = 3
STREAM_RETRY_DELAY = 2

# Confirm drone is online via serial (get_drone_ip returns the router IP the
# ESP32 printed on its serial output; we don't connect OpenCV to that IP).
router_ip = get_drone_ip()
if router_ip is None:
    print("[Main Error] Could not retrieve drone IP address from serial.")
    exit(1)

print(f"[Serial] Drone reported IP: {router_ip}  (drone is online)")

# BUG 2: stream from localhost where send_image_stream.py (Flask) is running
stream_url = f"http://127.0.0.1:{STREAM_PORT}/"
print(f"[Stream] Connecting to local Flask MJPEG at {stream_url}...")

cap = None
for attempt in range(MAX_STREAM_RETRIES):
    try:
        # BUG 9: set CAP_PROP_BUFFERSIZE immediately after construction, before read
        cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # must be set before first read
        if cap.isOpened():
            print(f"[Stream] ✅ Connected to {stream_url}")
            break
        else:
            cap.release()
            cap = None
            if attempt < MAX_STREAM_RETRIES - 1:
                # BUG 10: denominator is MAX_STREAM_RETRIES, not MAX_STREAM_RETRIES-1
                print(f"[Stream] Retry {attempt + 1}/{MAX_STREAM_RETRIES}... "
                      f"(waiting {STREAM_RETRY_DELAY}s)")
                time.sleep(STREAM_RETRY_DELAY)
    except Exception as e:
        print(f"[Stream Error] {e}")
        if attempt < MAX_STREAM_RETRIES - 1:
            print(f"[Stream] Retry {attempt + 1}/{MAX_STREAM_RETRIES}... "
                  f"(waiting {STREAM_RETRY_DELAY}s)")
            time.sleep(STREAM_RETRY_DELAY)

if cap is None or not cap.isOpened():
    print(f"[Main Error] Could not connect to video stream after {MAX_STREAM_RETRIES} attempts")
    exit(1)

# Camera hints (may not apply to MJPEG HTTP, but harmless)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg["cam_w"])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["cam_h"])
cap.set(cv2.CAP_PROP_FPS, 30)

# ══════════════════════════════════════════════════════════════════════════════
#  STATE
# ══════════════════════════════════════════════════════════════════════════════

track_hits   = defaultdict(int)
track_misses = defaultdict(int)
cached_boxes = []
frame_idx    = 0
fps_t0       = time.time()
fps_frames   = 0
fps_val      = 0.0

# ══════════════════════════════════════════════════════════════════════════════
#  DRAW HELPER
# ══════════════════════════════════════════════════════════════════════════════

def draw_boxes(frame, boxes):
    """
    BUG 13 FIX: Clamp label background Y so it never goes negative, which can
    corrupt rendering or crash on some OpenCV builds.
    """
    for (x1, y1, x2, y2, conf, tid) in boxes:
        colour = (0, 220, 0)
        label  = (f"Human #{tid} ({conf:.0%})" if tid >= 0
                  else f"Human ({conf:.0%})")
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        label_y_top = max(0, y1 - th - 10)          # clamp to frame top
        cv2.rectangle(frame, (x1, label_y_top), (x1 + tw + 6, y1), colour, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        text_y = max(th + 4, y1 - 4)                # clamp text baseline too
        cv2.putText(frame, label, (x1 + 3, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════════════════════════════

try:
    while True:
        # BUG 9 FIX: grab() discards the oldest buffered frame so read() returns
        # the freshest available frame, reducing display latency.
        cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            print("[Main] Camera stream ended or disconnected")
            break

        frame_idx  += 1
        fps_frames += 1
        elapsed = time.time() - fps_t0
        if elapsed >= 1.0:
            fps_val    = fps_frames / elapsed
            fps_frames = 0
            fps_t0     = time.time()

        run_ai = (frame_idx % cfg["skip"] == 0)

        if run_ai:
            try:
                results = model.track(
                    frame,
                    persist = True,
                    classes = [HUMAN_CLASS],
                    conf    = cfg["conf"],
                    iou     = cfg["iou"],
                    imgsz   = cfg["imgsz"],
                    tracker = "bytetrack.yaml",
                    verbose = False,
                    half    = cfg["half"],
                )

                active_ids = set()
                new_boxes  = []

                for result in results:
                    if result.boxes is None:
                        continue
                    for box in result.boxes:
                        if int(box.cls[0]) != HUMAN_CLASS:
                            continue
                        conf = float(box.conf[0])
                        tid  = int(box.id[0]) if box.id is not None else -1
                        active_ids.add(tid)
                        track_hits[tid]  += 1
                        track_misses[tid] = 0
                        if track_hits[tid] < cfg["smooth"]:
                            continue
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        new_boxes.append((x1, y1, x2, y2, conf, tid))

                for tid in list(track_hits):
                    if tid not in active_ids:
                        track_misses[tid] += 1
                        if track_misses[tid] > 10:
                            track_hits.pop(tid, None)
                            track_misses.pop(tid, None)

                cached_boxes = new_boxes

            except Exception as e:
                # BUG 11 FIX: Do NOT continue here.  Fall through so draw_boxes
                # and imshow still run with the last good cached_boxes.
                # Without this fix, any YOLO error freezes the display window.
                print(f"[AI Error] Detection failed: {e}")

        draw_boxes(frame, cached_boxes)

        # ── HUD ───────────────────────────────────────────────────────────────
        hud = [
            f"Humans : {len(cached_boxes)}",
            f"Device : {cfg['tier']}",
            f"Model  : {cfg['model']}  imgsz={cfg['imgsz']}",
            f"FPS    : {fps_val:.1f}  ({'AI' if run_ai else 'cached'})",
        ]
        y = 28
        for line in hud:
            cv2.putText(frame, line, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, line, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30,  30,  30),  1)
            y += 26

        cv2.imshow("Human Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[Main] User quit")
            break

except KeyboardInterrupt:
    print("[Main] Interrupted by user (Ctrl+C)")
except Exception as e:
    print(f"[Main Error] Unexpected error: {e}")
finally:
    cap.release()
    cv2.destroyAllWindows()
    print("[Main] Cleanup complete")