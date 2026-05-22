"""
send_image_stream.py  ──  Flask MJPEG server
Reads image.jpg written by receive_stream.py and serves it as MJPEG.
Simplified: always reads from disk, no import tricks.
"""

import os
import time
from flask import Flask, Response

app = Flask(__name__)

BASE_DIR         = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH       = os.path.join(BASE_DIR, "image.jpg")
PLACEHOLDER_PATH = os.path.join(BASE_DIR, "placeholder.jpg")

TARGET_FPS     = 25
FRAME_INTERVAL = 1.0 / TARGET_FPS

JPEG_HEADER = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
JPEG_TAIL   = b'\r\n'

# Cache
_last_mtime    = 0.0
_current_frame = b''


def _load_placeholder() -> bytes:
    try:
        with open(PLACEHOLDER_PATH, "rb") as f:
            return f.read()
    except OSError:
        return b''


PLACEHOLDER_BYTES = _load_placeholder()


@app.route('/')
def index():
    return Response(
        _mjpeg_generator(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def _mjpeg_generator():
    global _last_mtime, _current_frame

    while True:
        t0 = time.monotonic()

        # Always try to read latest image.jpg
        try:
            mtime = os.path.getmtime(IMAGE_PATH)
            if mtime != _last_mtime:
                for _ in range(2):
                    try:
                        with open(IMAGE_PATH, "rb") as f:
                            data = f.read()
                        if len(data) > 4 and data[:2] == b'\xff\xd8':
                            _current_frame = data
                            _last_mtime    = mtime
                        break
                    except OSError:
                        time.sleep(0.002)
        except OSError:
            pass

        frame = _current_frame if _current_frame else PLACEHOLDER_BYTES

        if frame:
            yield JPEG_HEADER + frame + JPEG_TAIL

        elapsed = time.monotonic() - t0
        sleep   = FRAME_INTERVAL - elapsed
        if sleep > 0:
            time.sleep(sleep)


if __name__ == "__main__":
    print(f"[Flask] Serving MJPEG from {IMAGE_PATH}")
    print(f"[Flask] Stream at http://0.0.0.0:5000/")
    app.run(host='0.0.0.0', port=5000,
            debug=False, threaded=True, use_reloader=False)