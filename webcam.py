from ultralytics import YOLO
import cv2
import torch
from collections import defaultdict

# ── Auto-detect hardware ───────────────────────────────────────────────────────
def get_device():
    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        vram_gb  = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"[Device] GPU detected: {gpu_name} ({vram_gb:.1f} GB VRAM)")
        return "gpu", gpu_name, vram_gb
    try:
        import torch_directml          # Windows DirectML (AMD / Intel Arc)
        print("[Device] DirectML GPU detected")
        return "directml", "DirectML", 0
    except ImportError:
        pass
    print("[Device] No GPU found — using CPU")
    return "cpu", "CPU", 0

device_type, device_name, vram_gb = get_device()

# ── Adaptive config based on hardware ─────────────────────────────────────────
def build_config(device_type, vram_gb):
    """Return a config dict tuned for the detected hardware."""

    if device_type == "gpu":
        if vram_gb >= 8:                        # high-end GPU (RTX 3070+)
            return dict(
                model_path  = "yolov8x.pt",     # max accuracy
                imgsz       = 960,
                conf        = 0.50,
                iou         = 0.40,
                skip_frames = 1,                # every frame
                smooth      = 2,
                half        = True,             # FP16 for speed
                cam_w       = 1920, cam_h=1080,
                device      = "cuda",
                tier        = "High-end GPU",
            )
        elif vram_gb >= 4:                      # mid-range GPU (GTX 1660 / RTX 3050)
            return dict(
                model_path  = "yolov8m.pt",
                imgsz       = 640,
                conf        = 0.50,
                iou         = 0.40,
                skip_frames = 1,
                smooth      = 2,
                half        = True,
                cam_w       = 1280, cam_h=720,
                device      = "cuda",
                tier        = "Mid-range GPU",
            )
        else:                                   # low VRAM GPU (< 4 GB)
            return dict(
                model_path  = "yolov8s.pt",
                imgsz       = 640,
                conf        = 0.45,
                iou         = 0.45,
                skip_frames = 1,
                smooth      = 3,
                half        = True,
                cam_w       = 1280, cam_h=720,
                device      = "cuda",
                tier        = "Low VRAM GPU",
            )

    elif device_type == "directml":
        return dict(
            model_path  = "yolov8s.pt",
            imgsz       = 640,
            conf        = 0.45,
            iou         = 0.45,
            skip_frames = 1,
            smooth      = 3,
            half        = False,            # DirectML doesn't support FP16 reliably
            cam_w       = 1280, cam_h=720,
            device      = "cpu",            # fallback; DirectML needs special loader
            tier        = "DirectML GPU",
        )

    else:                                       # CPU
        return dict(
            model_path  = "yolov8n.pt",         # nano = fastest on CPU
            imgsz       = 320,
            conf        = 0.45,
            iou         = 0.45,
            skip_frames = 2,                    # infer every other frame
            smooth      = 3,
            half        = False,                # FP16 not supported on CPU
            cam_w       = 640, cam_h=480,
            device      = "cpu",
            tier        = "CPU",
        )

cfg = build_config(device_type, vram_gb)

print(f"[Config] Tier     : {cfg['tier']}")
print(f"[Config] Model    : {cfg['model_path']}")
print(f"[Config] Img size : {cfg['imgsz']}")
print(f"[Config] FP16     : {cfg['half']}")
print(f"[Config] Skip     : every {cfg['skip_frames']} frame(s)")

# ── Load model ────────────────────────────────────────────────────────────────
model = YOLO(cfg["model_path"])
model.to(cfg["device"])

HUMAN_CLASS_ID = 0

# ── Webcam ────────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg["cam_w"])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["cam_h"])
cap.set(cv2.CAP_PROP_FPS,          30)

# ── State ─────────────────────────────────────────────────────────────────────
track_hits   = defaultdict(int)
track_misses = defaultdict(int)
cached_boxes = []
frame_counter = 0

# FPS counter
import time
fps_timer   = time.time()
fps_counter = 0
fps_display = 0.0

def draw_boxes(frame, boxes):
    for (x1, y1, x2, y2, confidence, track_id) in boxes:
        colour = (0, 255, 0)
        label  = (f"Human #{track_id}  {confidence:.0%}"
                  if track_id != -1 else f"Human  {confidence:.0%}")
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw + 6, y1), colour, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(frame, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera error")
        break

    frame_counter += 1
    run_inference = (frame_counter % cfg["skip_frames"] == 0)

    # ── FPS calculation ───────────────────────────────────────────────────────
    fps_counter += 1
    elapsed = time.time() - fps_timer
    if elapsed >= 1.0:
        fps_display = fps_counter / elapsed
        fps_counter = 0
        fps_timer   = time.time()

    if run_inference:
        results = model.track(
            frame,
            persist = True,
            classes = [HUMAN_CLASS_ID],
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
                if int(box.cls[0]) != HUMAN_CLASS_ID:
                    continue

                confidence = float(box.conf[0])
                track_id   = int(box.id[0]) if box.id is not None else -1
                active_ids.add(track_id)

                track_hits[track_id]  += 1
                track_misses[track_id] = 0

                if track_hits[track_id] < cfg["smooth"]:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                new_boxes.append((x1, y1, x2, y2, confidence, track_id))

        # Prune stale tracks
        for tid in list(track_hits):
            if tid not in active_ids:
                track_misses[tid] += 1
                if track_misses[tid] > 10:
                    track_hits.pop(tid, None)
                    track_misses.pop(tid, None)

        cached_boxes = new_boxes

    # ── Draw ──────────────────────────────────────────────────────────────────
    draw_boxes(frame, cached_boxes)

    # ── HUD ───────────────────────────────────────────────────────────────────
    hud_lines = [
        f"Humans : {len(cached_boxes)}",
        f"Device : {cfg['tier']}  ({device_name})",
        f"Model  : {cfg['model_path']}  |  imgsz {cfg['imgsz']}",
        f"FPS    : {fps_display:.1f}  |  {'AI' if run_inference else 'cached'}",
    ]
    y = 28
    for line in hud_lines:
        cv2.putText(frame, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y += 26

    cv2.imshow("Human Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()