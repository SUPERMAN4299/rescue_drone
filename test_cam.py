"""
test_cam.py  ──  Webcam → WebSocket simulator
"""

import asyncio
import time
import cv2
import websockets

# ── Config ────────────────────────────────────────────────────────────────────
WS_URI       = "ws://127.0.0.1:3001"
CAM_WIDTH    = 320          # smaller = faster send = no ping timeout
CAM_HEIGHT   = 240
TARGET_FPS   = 15           # realistic for CPU pipeline
FRAME_INTV   = 1.0 / TARGET_FPS
JPEG_QUAL    = 60           # smaller payload
ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUAL]


async def send_frames():
    print(f"[Cam] Connecting to {WS_URI}…")

    async with websockets.connect(
        WS_URI,
        max_size      = None,
        ping_interval = None,   # ← DISABLE ping entirely; avoids timeout
        compression   = None,
    ) as ws:
        print("[Cam] Connected")

        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        if not cap.isOpened():
            print("[Cam] ERROR: Cannot open webcam (index 0)")
            return

        print(f"[Cam] Streaming {CAM_WIDTH}×{CAM_HEIGHT} @ {TARGET_FPS}fps  Q={JPEG_QUAL}")

        frame_count = 0
        t_next      = time.monotonic()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[Cam] Camera read failed — retrying…")
                    await asyncio.sleep(0.1)
                    continue

                success, buf = cv2.imencode(".jpg", frame, ENCODE_PARAM)
                if not success:
                    continue

                jpeg_bytes = buf.tobytes()

                # await the send directly — no create_task
                # ping_interval=None means no ping competes with this await
                await ws.send(jpeg_bytes)

                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"[Cam] Sent {frame_count} frames  ({len(jpeg_bytes)//1024} KB last)")

                # Rate-limit
                t_next += FRAME_INTV
                sleep   = t_next - time.monotonic()
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