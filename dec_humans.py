from ultralytics import YOLO
import cv2
import torch
import subprocess
import platform
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
        names = [l.strip() for l in out.splitlines() if l.strip() and l.strip().lower() != "name"]
        return names  # e.g. ['NVIDIA GeForce RTX 3060', 'Intel UHD Graphics 730']
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
    # ── 1. NVIDIA CUDA ────────────────────────────────────────────────────────
    if torch.cuda.is_available():
        name    = torch.cuda.get_device_name(0)
        vram_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"[Device] ✅ NVIDIA CUDA GPU: {name} ({vram_gb:.1f} GB VRAM)")
        return "cuda", name, vram_gb

    # ── 2. Windows WMI GPU name scan ─────────────────────────────────────────
    if platform.system() == "Windows":
        gpu_names = _query_windows_gpu()
        for name in gpu_names:
            name_lower = name.lower()

            # NVIDIA present but CUDA not installed → still CPU path
            if "nvidia" in name_lower:
                print(f"[Device] ⚠️  NVIDIA GPU found ({name}) but CUDA is NOT installed.")
                print("[Device]    Install CUDA-enabled PyTorch:")
                print("[Device]    https://pytorch.org/get-started/locally/")
                print("[Device]    Falling back to CPU for now.")
                return "cpu_nvidia_hint", name, 0

            # AMD GPU
            if "amd" in name_lower or "radeon" in name_lower:
                print(f"[Device] ℹ️  AMD GPU detected: {name}")
                print("[Device]    torch-directml is unavailable on PyPI for this Python/platform.")
                print("[Device]    Tip: try ONNX Runtime with DirectML for AMD acceleration:")
                print("[Device]    pip install onnxruntime-directml")
                print("[Device]    Falling back to CPU.")
                return "cpu_amd_hint", name, 0

            # Intel Arc / Xe
            if "intel" in name_lower and ("arc" in name_lower or "xe" in name_lower):
                print(f"[Device] ℹ️  Intel Arc/Xe GPU detected: {name}")
                print("[Device]    For Intel GPU acceleration install OpenVINO:")
                print("[Device]    pip install openvino ultralytics[openvino]")
                print("[Device]    Falling back to CPU.")
                return "cpu_intel_hint", name, 0

    # ── 3. CPU fallback ───────────────────────────────────────────────────────
    # Count physical CPU cores to rank CPU tier
    cpu_count = torch.get_num_threads()
    print(f"[Device] 🖥️  CPU mode  ({cpu_count} threads available)")
    return "cpu", "CPU", 0


device_type, device_name, vram_gb = get_device()

# ══════════════════════════════════════════════════════════════════════════════
# IP ADRESS CONFIG
# ══════════════════════════════════════════════════════════════════════════════

def get_router_ip():
    try:
        # Run ipconfig command
        output = subprocess.check_output("ipconfig", text=True)

        # Find Default Gateway
        matches = re.findall(r"Default Gateway[ .:]*([\d.]+)", output)

        for ip in matches:
            if ip.strip():
                return ip

    except Exception as e:
        print("Error:", e)

    return None

router_ip = get_router_ip()

if router_ip:
    print("Router IP:", router_ip)
else:
    print("Router IP not found")

# ══════════════════════════════════════════════════════════════════════════════
#  ADAPTIVE CONFIG
# ══════════════════════════════════════════════════════════════════════════════

def build_config(device_type, vram_gb):
    # ── NVIDIA CUDA ───────────────────────────────────────────────────────────
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

    # ── All CPU paths (pure CPU, or GPU found but no usable driver) ───────────
    # AMD/Intel/NVIDIA-no-CUDA all land here — same CPU tuning applies
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

model = YOLO(cfg["model"])
model.to(cfg["device"])

HUMAN_CLASS = 0

# ══════════════════════════════════════════════════════════════════════════════
#  WEBCAM
# ══════════════════════════════════════════════════════════════════════════════

router_ip = get_router_ip()
cap = cv2.VideoCapture(
    f"http://{router_ip}:3001",
    cv2.CAP_FFMPEG
)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency by keeping only the latest frame in buffer

# camera settings
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg["cam_w"])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["cam_h"])
cap.set(cv2.CAP_PROP_FPS, 30)

# ══════════════════════════════════════════════════════════════════════════════
#  STATE
# ══════════════════════════════════════════════════════════════════════════════

track_hits    = defaultdict(int)
track_misses  = defaultdict(int)
cached_boxes  = []
frame_idx     = 0
fps_t0        = time.time()
fps_frames    = 0
fps_val       = 0.0

# ══════════════════════════════════════════════════════════════════════════════
#  DRAW HELPER
# ══════════════════════════════════════════════════════════════════════════════

def draw_boxes(frame, boxes):
    for (x1, y1, x2, y2, conf, tid) in boxes:
        colour = (0, 220, 0)
        label  = f"Human #{tid}  {conf:.0%}" if tid != -1 else f"Human  {conf:.0%}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw + 6, y1), colour, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(frame, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════════════════════════════

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera error")
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

    draw_boxes(frame, cached_boxes)

    # ── HUD ───────────────────────────────────────────────────────────────────
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
        break

cap.release()
cv2.destroyAllWindows()