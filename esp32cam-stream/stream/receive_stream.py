"""
receive_stream.py  ──  WebSocket frame receiver
"""

import asyncio
import os
import threading
import time
import websockets

BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH = os.path.join(BASE_DIR, "image.jpg")

# ── In-memory frame slot ──────────────────────────────────────────────────────
_frame_lock  = threading.Lock()
LATEST_FRAME = b""
_frame_ts    = 0.0


def get_latest_frame() -> bytes:
    with _frame_lock:
        return LATEST_FRAME


def _store_frame(data: bytes) -> None:
    global LATEST_FRAME, _frame_ts
    with _frame_lock:
        LATEST_FRAME = data
        _frame_ts    = time.time()
    try:
        with open(IMAGE_PATH, "wb", buffering=0) as f:
            f.write(data)
    except OSError:
        pass


def _is_jpeg(data: bytes) -> bool:
    return (len(data) >= 4
            and data[:2]  == b'\xff\xd8'
            and data[-2:] == b'\xff\xd9')


async def handle_connection(websocket):
    peer = websocket.remote_address
    print(f"[WS] Client connected: {peer}")
    frame_count = 0
    try:
        async for message in websocket:
            if len(message) < 100:   # lowered from 1000 — 320×240 Q60 can be small
                continue
            if not _is_jpeg(message):
                print(f"[WS] Invalid JPEG ({len(message)} bytes)")
                continue
            _store_frame(message)
            frame_count += 1
            if frame_count % 100 == 0:
                print(f"[WS] {frame_count} frames received")
    except websockets.exceptions.ConnectionClosed:
        print(f"[WS] Client disconnected: {peer}  ({frame_count} frames received)")
    except Exception as e:
        print(f"[WS] Error: {e}")


async def main():
    server = await websockets.serve(
        handle_connection,
        "0.0.0.0", 3001,
        max_size      = 5 * 1024 * 1024,
        ping_interval = None,   # ← DISABLE ping — matches client setting
        compression   = None,
    )
    print("[WS] Server running on port 3001")
    await server.wait_closed()


if __name__ == "__main__":
    asyncio.run(main())