"""
test_cam.py  ──  Mobile Camera → WebSocket simulat
"""

import asyncio
import time
import cv2
import websockets

# ── Config ────────────────────────────────────────────────────────────────────
WS_URI = "ws://127.0.0.1:3001"

# Mobile IP Webcam URL

DRONE_CAM_IP = "http://192.168.1.2:8080/video"
CAM_WIDTH    = 320
CAM_HEIGHT   = 240
TARGET_FPS   = 15
FRAME_INTV   = 1.0 / TARGET_FPS

JPEG_QUAL    = 60
ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUAL]


async def send_frames():
    print(f"[Cam] Connecting to {WS_URI}…")

    async with websockets.connect(
        WS_URI,
        max_size=None,
        ping_interval=None,
        compression=None,
    ) as ws:

        print("[Cam] Connected")

        # ── USE MOBILE CAMERA STREAM ─────────────────────────────
        cap = cv2.VideoCapture(DRONE_CAM_IP)

        if not cap.isOpened():
            print("[Cam] ERROR: Cannot open mobile camera stream")
            return

        print(f"[Cam] Streaming mobile camera @ {TARGET_FPS} FPS")

        frame_count = 0
        t_next = time.monotonic()

        try:
            while True:
                ret, frame = cap.read()

                if not ret:
                    print("[Cam] Stream read failed")
                    await asyncio.sleep(0.1)
                    continue

                # Resize for faster transfer
                frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))

                success, buf = cv2.imencode(".jpg", frame, ENCODE_PARAM)

                if not success:
                    continue

                jpeg_bytes = buf.tobytes()

                await ws.send(jpeg_bytes)

                frame_count += 1

                if frame_count % 100 == 0:
                    print(f"[Cam] Sent {frame_count} frames")

                # FPS limiter
                t_next += FRAME_INTV
                sleep = t_next - time.monotonic()

                if sleep > 0:
                    await asyncio.sleep(sleep)
                else:
                    t_next = time.monotonic()

        except websockets.exceptions.ConnectionClosed as e:
            print(f"[Cam] Connection closed: {e}")

        finally:
            cap.release()
            print(f"[Cam] Done. Sent {frame_count} frames.")


if __name__ == "__main__":
    asyncio.run(send_frames())