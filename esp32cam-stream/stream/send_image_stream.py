import os
import time
from flask import Flask, Response

app = Flask(__name__)

# ── Config ────────────────────────────────────────────────────────────────────
TARGET_FPS     = 25
FRAME_INTERVAL = 1.0 / TARGET_FPS   # seconds between yields (~40 ms)

# ── Frame cache ───────────────────────────────────────────────────────────────
# Tracks the mtime of the last image.jpg we read so we avoid re-reading (and
# re-sending) the exact same file when the ESP32 is slower than TARGET_FPS.
_last_mtime    = 0.0
_current_frame = b''   # raw JPEG bytes of the most recently read live frame


@app.route('/')
def index():
    return Response(get_image(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def get_image():
    """
    MJPEG generator that reads image.jpg written by receive_stream.py.

    Key fixes vs original:
      - Removed PIL / BytesIO decode→re-encode round-trip.  image.jpg is
        already a valid JPEG; raw bytes are streamed directly.
      - Added mtime check so the same file is not re-read from disk every
        iteration when no new frame has arrived (reduces unnecessary I/O).
      - Added a single read-retry (2 ms back-off) to handle the brief window
        on Windows where os.replace() may fail if the file is held open.
      - Placeholder path uses the same raw-bytes approach for consistency.
      - Added use_reloader=False at startup to prevent Werkzeug from forking
        the process and running two copies of this generator.
    """
    global _last_mtime, _current_frame

    while True:
        t0 = time.time()
        frame_sent = False

        # ── Try to serve the latest live frame ────────────────────────────────
        try:
            mtime = os.path.getmtime("image.jpg")

            if mtime != _last_mtime:
                # File has been updated — read it (with one retry on failure)
                img_bytes = b''
                for _ in range(2):
                    try:
                        with open("image.jpg", "rb") as f:
                            img_bytes = f.read()
                        break
                    except OSError:
                        time.sleep(0.002)   # 2 ms back-off, retry once

                # Only cache if it looks like a real JPEG (SOI marker check)
                if len(img_bytes) > 4 and img_bytes[:2] == b'\xff\xd8':
                    _current_frame = img_bytes
                    _last_mtime    = mtime

            if _current_frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n'
                       + _current_frame + b'\r\n')
                frame_sent = True

        except Exception as e:
            print(f"[Flask] image.jpg error: {e}")

        # ── Fall back to placeholder if live frame failed ─────────────────────
        if not frame_sent:
            try:
                with open("placeholder.jpg", "rb") as f:
                    placeholder_bytes = f.read()
                if len(placeholder_bytes) > 4 and placeholder_bytes[:2] == b'\xff\xd8':
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n'
                           + placeholder_bytes + b'\r\n')
            except Exception as e2:
                # Placeholder also missing/corrupt — log and keep looping
                print(f"[Flask] Placeholder error: {e2}")

        # ── Rate-limit to TARGET_FPS ──────────────────────────────────────────
        elapsed    = time.time() - t0
        sleep_time = FRAME_INTERVAL - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    # use_reloader=False: prevents Werkzeug from forking the process on startup,
    # which would otherwise run two copies of the get_image() generator and
    # double CPU usage.  debug=False already disables the reloader by default,
    # but being explicit here ensures it stays off even if debug is toggled.
    app.run(host='0.0.0.0', port=5000,
            debug=False, threaded=True, use_reloader=False)