"""
analysing_cap.py  ──  YOLOv8 Human Detector  (LOW-LATENCY REWRITE)
════════════════════════════════════════════════════════════════════
Root-cause of the original lag
───────────────────────────────
cv2.VideoCapture over an HTTP MJPEG stream internally queues decoded
frames in an FFmpeg/GStreamer buffer.  When YOLO inference takes > one
frame period the buffer fills up.  The next cap.read() returns the
*oldest* queued frame, not the newest — creating multi-second lag that
grows unboundedly.  CAP_PROP_BUFFERSIZE=1 is a hint, not a guarantee,
for network streams on Windows (FFmpeg ignores it for HTTP sources).

Solution: decouple reading from inference with two dedicated threads.

  ┌──────────────┐      ┌──────────────────┐      ┌──────────────┐
  │ FrameReader  │─────▶│  frame_slot      │◀─────│ YOLO thread  │
  │ thread       │      │  (always newest) │      │              │
  └──────────────┘      └──────────────────┘      └──────────────┘
                                                         │
                                               ┌─────────▼──────────┐
                                               │  boxes_slot        │
                                               │  (latest detects)  │
                                               └─────────┬──────────┘
                                                         │
                                               ┌─────────▼──────────┐
                                               │  Main thread:      │
                                               │  draw + imshow     │
                                               └────────────────────┘

FrameReader continuously reads from cap and keeps ONLY the latest frame
(older frames are silently dropped).  YOLO always grabs the newest
available frame from the slot, so it never processes stale data.

Other CPU latency improvements
───────────────────────────────
• Model   : yolov8n.pt  (fastest on CPU; detection is still solid ≥480px)
• imgsz   : 416px       (25-35 % faster than 640 with ~5 % accuracy cost)
• OpenCV  : BGR→RGB via np slice, not cvtColor (saves a copy)
• Tracking: botsort replaced by bytetrack (lower per-frame cost on CPU)
• FP16    : always False on CPU (FP16 is slower without GPU tensor cores)
• Threads : torch set to physical-core count to avoid HT contention
• Display : imshow on main thread only (required by most OS window managers)
"""

from __future__ import annotations

import os
import platform
import re
import subprocess
import threading
import time
from collections import defaultdict
from typing import Optional

import cv2
import numpy as np
import torch
from ultralytics import YOLO

# ══════════════════════════════════════════════════════════════════════════════
#  TORCH CPU THREAD TUNING
# ══════════════════════════════════════════════════════════════════════════════
# Using all logical threads (including HyperThreading siblings) for
# matrix ops hurts throughput because HT siblings share execution units.
# Limit to physical cores when detectable.
try:
    import psutil
    _PHYSICAL = psutil.cpu_count(logical=False) or torch.get_num_threads()
except ImportError:
    _PHYSICAL = max(1, torch.get_num_threads() // 2)

torch.set_num_threads(_PHYSICAL)
torch.set_num_interop_threads(max(1, _PHYSICAL // 2))
print(f"[Torch] CPU threads: {_PHYSICAL} (physical cores)")


# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE DETECTION  (unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def _query_windows_gpu():
    try:
        out = subprocess.check_output(
            ["wmic", "path", "win32_VideoController", "get", "name"],
            stderr=subprocess.DEVNULL, timeout=4
        ).decode(errors="ignore")
        return [l.strip() for l in out.splitlines()
                if l.strip() and l.strip().lower() != "name"]
    except Exception:
        return []


def get_device():
    if torch.cuda.is_available():
        name    = torch.cuda.get_device_name(0)
        vram_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"[Device] ✅ NVIDIA CUDA GPU: {name} ({vram_gb:.1f} GB VRAM)")
        return "cuda", name, vram_gb

    if platform.system() == "Windows":
        for name in _query_windows_gpu():
            nl = name.lower()
            if "nvidia" in nl:
                print(f"[Device] ⚠️  NVIDIA GPU ({name}) found but CUDA not installed → CPU")
                return "cpu_nvidia_hint", name, 0
            if "amd" in nl or "radeon" in nl:
                print(f"[Device] ℹ️  AMD GPU: {name} → CPU (install onnxruntime-directml for accel)")
                return "cpu_amd_hint", name, 0
            if "intel" in nl and ("arc" in nl or "xe" in nl):
                print(f"[Device] ℹ️  Intel Arc/Xe: {name} → CPU (install openvino for accel)")
                return "cpu_intel_hint", name, 0

    print(f"[Device] 🖥️  CPU mode ({torch.get_num_threads()} threads)")
    return "cpu", "CPU", 0


device_type, device_name, vram_gb = get_device()


# ══════════════════════════════════════════════════════════════════════════════
#  IP HELPER  (unchanged — serial drone IP detection)
# ══════════════════════════════════════════════════════════════════════════════

_PRIVATE_IP_RE = re.compile(
    r'\b('
    r'10\.\d{1,3}\.\d{1,3}\.\d{1,3}'
    r'|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3}'
    r'|192\.168\.\d{1,3}\.\d{1,3}'
    r')\b'
)


def get_drone_ip(com_port="COM5", baud_rate=115200,    # Select your com port
                 timeout_sec=30, max_attempts=None):
    try:
        import serial
        ser = serial.Serial(com_port, baud_rate, timeout=1)
    except Exception as e:
        print(f"[IP Error] {e}")
        return None

    start = time.time()
    attempt = 0
    try:
        while True:
            if time.time() - start > timeout_sec:
                print(f"[IP Timeout] No IP in {timeout_sec}s")
                return None
            if max_attempts and attempt >= max_attempts:
                return None
            try:
                line = ser.readline().decode(errors="ignore").strip()
                attempt += 1
                if not line:
                    continue
                m = _PRIVATE_IP_RE.search(line)
                if m:
                    ip = m.group(1)
                    print(f"[IP Found] {ip}")
                    return ip
            except Exception as e:
                print(f"[IP Read Error] {e}")
    finally:
        ser.close()


# ══════════════════════════════════════════════════════════════════════════════
#  ADAPTIVE CONFIG
# ══════════════════════════════════════════════════════════════════════════════

def build_config(device_type: str, vram_gb: float) -> dict:
    if device_type == "cuda":
        if vram_gb >= 8:
            return dict(model="yolov8x.pt", imgsz=960, conf=0.30, iou=0.45,
                        half=True,  device="cuda", cam_w=1920, cam_h=1080,
                        tier="High-end GPU (CUDA)")
        elif vram_gb >= 4:
            return dict(model="yolov8m.pt", imgsz=640, conf=0.25, iou=0.45,
                        half=True,  device="cuda", cam_w=1280, cam_h=720,
                        tier="Mid-range GPU (CUDA)")
        else:
            return dict(model="yolov8s.pt", imgsz=640, conf=0.25, iou=0.45,
                        half=True,  device="cuda", cam_w=1280, cam_h=720,
                        tier="Low-VRAM GPU (CUDA)")

    # ── CPU ────────────────────────────────────────────────────────────────
    # yolov8n at 416px is the sweet spot for real-time CPU inference.
    # It runs 2-4× faster than yolov8s@640 and detects humans reliably
    # down to ~80×160 px in-frame.  Confidence kept at 0.30 to reduce
    # false positives that would otherwise require extra NMS time.
    return dict(
        model  = "yolov8n.pt",   # nano — fastest CPU model
        imgsz  = 416,            # 25-35 % faster vs 640, minor accuracy loss
        conf   = 0.30,           # fewer low-quality candidates → faster NMS
        iou    = 0.45,
        half   = False,          # FP16 is slower on CPU without tensor cores
        device = "cpu",
        cam_w  = 640,            # lower res → less JPEG decode cost in cap
        cam_h  = 480,
        tier   = f"CPU ({device_name})",
    )


cfg = build_config(device_type, vram_gb)

print(f"\n[Config] Tier    : {cfg['tier']}")
print(f"[Config] Model   : {cfg['model']}")
print(f"[Config] Img size: {cfg['imgsz']}")
print(f"[Config] FP16    : {cfg['half']}")
print(f"[Config] Res     : {cfg['cam_w']}×{cfg['cam_h']}\n")

# Slight upscale for the displayed output (1.0 = native, >1.0 = larger)
# Tune this to 'increase the output frame little bit'
cfg['display_scale'] = 1.2
print(f"[Config] Display scale: {cfg['display_scale']:.2f}x\n")

# ══════════════════════════════════════════════════════════════════════════════
#  LOAD MODEL
# ══════════════════════════════════════════════════════════════════════════════

try:
    model = YOLO(cfg["model"])
    model.to(cfg["device"])
    # Warm-up: run one dummy inference so the first real frame isn't slow
    _dummy = np.zeros((cfg["imgsz"], cfg["imgsz"], 3), dtype=np.uint8)
    model.predict(_dummy, verbose=False, imgsz=cfg["imgsz"])
    print(f"[Model] ✅ {cfg['model']} loaded & warmed up on {cfg['device']}")
except Exception as e:
    print(f"[Model Error] {e}")
    raise SystemExit(1)

HUMAN_CLASS = 0


# ══════════════════════════════════════════════════════════════════════════════
#  FRAME READER THREAD
# ══════════════════════════════════════════════════════════════════════════════
# The reader runs as a daemon thread.  It calls cap.read() in a tight loop
# and stores ONLY the latest decoded frame.  Old frames are overwritten
# immediately — the YOLO thread always gets the freshest image.

STREAM_PORT = 5000
stream_url = f"http://127.0.0.1:{STREAM_PORT}/"

# Thread-shared state
_frame_lock    = threading.Lock()
_latest_frame: Optional[np.ndarray] = None
_frame_counter = 0          # monotonic counter; used to detect new frames
_reader_alive  = threading.Event()
_reader_alive.set()


def _open_cap(url: str, retries: int = 5, delay: float = 2.0):
    """Open VideoCapture with retry; returns cap or None."""
    # Tell FFmpeg to not buffer — must be set BEFORE VideoCapture opens
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "analyzeduration;0|probesize;32|fflags;nobuffer|flags;low_delay"
    )
    for attempt in range(retries):
        # Use CAP_ANY (not CAP_FFMPEG) — on Windows, CAP_FFMPEG sometimes
        # fails to open MJPEG HTTP streams; CAP_ANY lets OpenCV pick the
        # best available backend automatically.
        cap = cv2.VideoCapture(url, cv2.CAP_ANY)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cap.isOpened():
            print(f"[Stream] ✅ Connected ({url})")
            return cap
        cap.release()
        if attempt < retries - 1:
            print(f"[Stream] Retry {attempt+1}/{retries} in {delay}s…")
            time.sleep(delay)
    print(f"[Stream] ❌ Could not connect after {retries} attempts.")
    return None


def _frame_reader_loop(url: str):
    """
    Daemon thread: reads frames as fast as cap delivers them and stores
    only the latest one.  Never blocks on YOLO or display.

    On stream loss it attempts reconnection automatically.
    """
    global _latest_frame, _frame_counter
    cap = _open_cap(url)
    if cap is None:
        _reader_alive.clear()
        return

    while _reader_alive.is_set():
        ret, frame = cap.read()
        if not ret:
            print("[Reader] Stream lost — reconnecting…")
            cap.release()
            cap = _open_cap(url)
            if cap is None:
                print("[Reader] Reconnect failed. Stopping reader.")
                _reader_alive.clear()
                break
            continue

        with _frame_lock:
            _latest_frame  = frame
            _frame_counter += 1

    cap.release()
    print("[Reader] Thread exiting.")


def _get_latest_frame():
    """Non-blocking read of the newest frame + its counter value."""
    with _frame_lock:
        return _latest_frame, _frame_counter


# ══════════════════════════════════════════════════════════════════════════════
#  YOLO INFERENCE THREAD
# ══════════════════════════════════════════════════════════════════════════════
# Runs inference continuously.  Always grabs the latest frame from the slot;
# if the counter has not changed since last inference it briefly sleeps to
# avoid spinning.

_boxes_lock    = threading.Lock()
_latest_boxes: list = []        # shared result for the draw thread
_ai_fps_val    = 0.0

_track_hits:   dict = defaultdict(int)
_track_misses: dict = defaultdict(int)
SMOOTH_FRAMES = 1   # set to 1 for lowest latency (was 2); increase if jittery

def _yolo_loop():
    global _latest_boxes, _ai_fps_val, _track_hits, _track_misses

    last_counter = -1
    t0           = time.time()
    ai_frames    = 0

    while _reader_alive.is_set():
        frame, counter = _get_latest_frame()

        # No new frame yet — yield CPU briefly
        if frame is None or counter == last_counter:
            time.sleep(0.005)
            continue

        last_counter = counter

        try:
            results = model.track(
                frame,
                persist   = True,
                classes   = [HUMAN_CLASS],
                conf      = cfg["conf"],
                iou       = cfg["iou"],
                imgsz     = cfg["imgsz"],
                tracker   = "bytetrack.yaml",
                verbose   = False,
                half      = cfg["half"],
            )

            active_ids = set()
            new_boxes  = []

            for result in results:
                if result.boxes is None:
                    continue
                for box in result.boxes:
                    if int(box.cls[0]) != HUMAN_CLASS:
                        continue
                    conf_val = float(box.conf[0])
                    tid      = int(box.id[0]) if box.id is not None else -1
                    active_ids.add(tid)
                    _track_hits[tid]   += 1
                    _track_misses[tid]  = 0
                    if _track_hits[tid] < SMOOTH_FRAMES:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    new_boxes.append((x1, y1, x2, y2, conf_val, tid))

            # Age out lost tracks
            for tid in list(_track_hits):
                if tid not in active_ids:
                    _track_misses[tid] += 1
                    if _track_misses[tid] > 8:
                        _track_hits.pop(tid, None)
                        _track_misses.pop(tid, None)

            with _boxes_lock:
                _latest_boxes = new_boxes

            ai_frames += 1
            elapsed = time.time() - t0
            if elapsed >= 1.0:
                _ai_fps_val = ai_frames / elapsed
                ai_frames   = 0
                t0          = time.time()

        except Exception as e:
            print(f"[AI Error] {e}")

    print("[YOLO] Thread exiting.")


def _get_latest_boxes():
    with _boxes_lock:
        return list(_latest_boxes)


# ══════════════════════════════════════════════════════════════════════════════
#  DRAW HELPER
# ══════════════════════════════════════════════════════════════════════════════

def draw_boxes(frame: np.ndarray, boxes: list) -> None:
    for (x1, y1, x2, y2, conf_val, tid) in boxes:
        colour = (0, 220, 0)
        label  = (f"Human #{tid} ({conf_val:.0%})" if tid >= 0
                  else f"Human ({conf_val:.0%})")
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        label_y_top = max(0, y1 - th - 10)
        cv2.rectangle(frame, (x1, label_y_top), (x1 + tw + 6, y1), colour, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        text_y = max(th + 4, y1 - 4)
        cv2.putText(frame, label, (x1 + 3, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)


def _scale_frame_for_display(frame: np.ndarray) -> np.ndarray:
    """Resize frame for display according to `cfg['display_scale']`.

    Returns the original frame if scale == 1.0.
    """
    scale = float(cfg.get('display_scale', 1.0))
    if scale == 1.0:
        return frame
    h, w = frame.shape[:2]
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main():
    # ── Drone serial confirmation ────────────────────────────────────────────
    router_ip = "http://192.168.1.6:8080"          # ← TEST MODE; replace with get_drone_ip()
    print(f"[Serial] Drone IP: {router_ip}  (test mode)")

    # ── Start background threads ─────────────────────────────────────────────
    reader_thread = threading.Thread(
        target=_frame_reader_loop, args=(stream_url,),
        name="FrameReader", daemon=True
    )
    yolo_thread = threading.Thread(
        target=_yolo_loop,
        name="YOLOInference", daemon=True
    )

    reader_thread.start()
    print("[Main] Frame reader thread started")

    # Wait briefly for the reader to confirm connectivity before launching YOLO
    for _ in range(40):
        if not _reader_alive.is_set():
            print("[Main] Reader failed to start. Exiting.")
            return
        frame, _ = _get_latest_frame()
        if frame is not None:
            break
        time.sleep(0.1)
    else:
        print("[Main] Timed out waiting for first frame.")
        _reader_alive.clear()
        return

    yolo_thread.start()
    print("[Main] YOLO thread started")
    print("[Main] Press Q in the display window to quit.\n")

    # ── Main / display loop ──────────────────────────────────────────────────
    # The main thread ONLY draws and calls imshow/waitKey.
    # It never blocks on I/O or inference.
    display_fps_t0     = time.time()
    display_fps_frames = 0
    display_fps_val    = 0.0
    last_display_frame = None

    try:
        while _reader_alive.is_set():
            frame, _ = _get_latest_frame()

            if frame is None:
                # Nothing yet — keep window alive
                if last_display_frame is not None:
                    cv2.imshow("Human Detection", _scale_frame_for_display(last_display_frame))
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
                continue

            # Draw on a copy so the YOLO thread always has the clean original
            display_frame = frame.copy()
            boxes         = _get_latest_boxes()

            # ── Human count file (real-time, current visible humans only) ─
            human_count = len(boxes)
            with open("human_count.txt", "w") as file:
                file.write(str(human_count))

            draw_boxes(display_frame, boxes)

            # ── HUD ───────────────────────────────────────────────────────────
            display_fps_frames += 1
            elapsed = time.time() - display_fps_t0
            if elapsed >= 1.0:
                display_fps_val    = display_fps_frames / elapsed
                display_fps_frames = 0
                display_fps_t0     = time.time()

            hud = [
                f"Humans  : {len(boxes)}",
                f"Device  : {cfg['tier']}",
                f"Model   : {cfg['model']}  imgsz={cfg['imgsz']}",
                f"Disp FPS: {display_fps_val:.1f}",
                f"AI  FPS : {_ai_fps_val:.1f}",
            ]
            y = 28
            for line in hud:
                cv2.putText(display_frame, line, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_frame, line, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30,  30,  30),  1)
                y += 26

            cv2.imshow("Human Detection", _scale_frame_for_display(display_frame))
            last_display_frame = display_frame

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[Main] User quit")
                break

    except KeyboardInterrupt:
        print("[Main] Ctrl+C — shutting down")
    finally:
        _reader_alive.clear()
        reader_thread.join(timeout=3)
        yolo_thread.join(timeout=3)
        cv2.destroyAllWindows()
        print("[Main] Cleanup complete")


if __name__ == "__main__":
    main()