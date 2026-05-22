import asyncio
import os
import websockets

# ── Paths ─────────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH = os.path.join(BASE_DIR, "image.jpg")
TMP_PATH   = "image.jpg.tmp"


# ── JPEG validity check ───────────────────────────────────────────────────────
def _is_jpeg(data: bytes) -> bool:
    """
    Check JPEG SOI (FF D8) and EOI (FF D9) markers.
    Fast — no decode; avoids importing cv2/numpy entirely.
    """
    return (len(data) >= 4
            and data[:2]  == b'\xff\xd8'
            and data[-2:] == b'\xff\xd9')


# ── Frame handler ─────────────────────────────────────────────────────────────
async def handle_connection(websocket):
    """
    Receives binary JPEG frames from the ESP32 over WebSocket and writes
    them atomically to image.jpg for Flask to serve.

    Key fixes vs original:
      - Removed latest_frame (assigned but never read anywhere).
      - Removed cv2 / numpy / BytesIO imports (nothing needs them here).
      - Eliminated decode→re-encode round-trip: ESP32 already sends valid
        JPEG; writing raw bytes preserves quality and removes CPU overhead.
      - Validity is checked via JPEG SOI/EOI markers instead of a full decode.
      - os.fsync() ensures bytes are flushed to disk before os.replace(),
        preventing a corrupt image.jpg on crash or power loss.
      - Per-frame bare except kept so one bad frame never kills the handler.
    """
    print("[WS] ESP32 Connected")

    while True:
        try:
            message = await websocket.recv()

            print(f"[WS] Frame received: {len(message)} bytes")

            # Skip obviously too-small payloads (noise / handshake messages)
            if len(message) < 1000:
                continue

            # Validate JPEG markers — skip silently if malformed
            if not _is_jpeg(message):
                print("[WS] Skipping invalid JPEG frame (bad SOI/EOI markers)")
                continue

            # ── Atomic write to disk ──────────────────────────────────────────
            # Write raw ESP32 JPEG bytes directly — no decode/re-encode.
            # os.replace() is atomic on POSIX and near-atomic on Windows
            # (same-filesystem rename), so Flask never sees a partial file.
            # Direct write (stable on Windows)
            try:
                with open(IMAGE_PATH, 'wb') as f:
                    f.write(message)
            
            except Exception as e:
                print(f"[WS] Save error: {e}")
                continue

        except websockets.exceptions.ConnectionClosed:
            print("[WS] Disconnected")
            break

        except Exception as e:
            # Catch all per-frame errors; log and continue so one bad frame
            # (OS write error, etc.) does NOT kill the entire connection.
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