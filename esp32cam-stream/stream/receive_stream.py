import asyncio
import os
import websockets
import cv2
import numpy as np
from io import BytesIO

# ── Shared state ──────────────────────────────────────────────────────────────
latest_frame = None
IMAGE_PATH   = "image.jpg"
TMP_PATH     = "image.jpg.tmp"

# ── Frame handler ─────────────────────────────────────────────────────────────
async def handle_connection(websocket):
    """
    Receives binary JPEG frames from the ESP32 over WebSocket.

    Fixes applied:
      - BUG 3 / BUG 4 : Write decoded frame to image.jpg atomically
                         (write to .tmp then os.replace) to prevent Flask
                         reading a half-written file.
      - BUG 7          : Removed is_valid_image() double-decode; cv2.imdecode
                         returns None on failure — that is sufficient.
      - BUG 8          : Added bare except inside the loop so a single bad frame
                         (corrupt JPEG, numpy error, OS write error) does NOT
                         kill the entire connection handler.
    """
    global latest_frame
    print("[WS] ESP32 Connected")

    while True:
        try:
            message = await websocket.recv()

            # Skip obviously too-small payloads (noise / control messages)
            if len(message) < 5000:
                continue

            # Single decode — if it fails, imdecode returns None
            np_arr = np.frombuffer(message, np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                # Not a valid JPEG — skip silently
                continue

            latest_frame = frame

            # ── Atomic write to disk (BUG 3 + BUG 4) ─────────────────────────
            # Re-encode to JPEG and write to a temp file, then rename.
            # os.replace() is atomic on POSIX and effectively atomic on Windows
            # (same-filesystem rename), so Flask never sees a partial file.
            ret, buf = cv2.imencode('.jpg', frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ret:
                with open(TMP_PATH, 'wb') as f:
                    f.write(buf.tobytes())
                os.replace(TMP_PATH, IMAGE_PATH)

        except websockets.exceptions.ConnectionClosed:
            print("[WS] Disconnected")
            break

        except Exception as e:
            # BUG 8: catch all other per-frame errors; log and continue
            print(f"[WS] Frame handling error: {e}")
            continue


# ── Server entry point ────────────────────────────────────────────────────────
async def main():
    async def wrapper(websocket):
        await handle_connection(websocket)

    server = await websockets.serve(wrapper, "0.0.0.0", 3001)
    print("[WS] Server running on port 3001")
    await server.wait_closed()


asyncio.run(main())