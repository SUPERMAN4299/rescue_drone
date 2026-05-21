import time
from io import BytesIO
from PIL import Image
from flask import Flask, Response

app = Flask(__name__)

# ── Config ────────────────────────────────────────────────────────────────────
TARGET_FPS     = 25
FRAME_INTERVAL = 1.0 / TARGET_FPS   # seconds between yields (~40 ms)


@app.route('/')
def index():
    return Response(get_image(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def get_image():
    """
    MJPEG generator that reads image.jpg written by receive_stream.py.

    Fixes applied:
      - BUG 5 : Added time.sleep so the loop runs at TARGET_FPS instead of
                spinning 100% CPU re-encoding the same stale file thousands
                of times per second.
      - BUG 6 : Wrapped the placeholder fallback in its own try/except so a
                missing/corrupt placeholder.jpg does NOT kill the generator
                (which would silently close the MJPEG stream).
    """
    while True:
        t0 = time.time()

        # ── Try to serve the latest live frame ────────────────────────────────
        frame_sent = False
        try:
            with open("image.jpg", "rb") as f:
                image_bytes = f.read()
            image = Image.open(BytesIO(image_bytes))
            img_io = BytesIO()
            image.save(img_io, 'JPEG')
            img_io.seek(0)
            img_bytes = img_io.read()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + img_bytes + b'\r\n')
            frame_sent = True

        except Exception as e:
            print(f"[Flask] image.jpg error: {e}")

        # ── Fall back to placeholder if live frame failed ─────────────────────
        if not frame_sent:
            try:
                with open("placeholder.jpg", "rb") as f:
                    image_bytes = f.read()
                image = Image.open(BytesIO(image_bytes))
                img_io = BytesIO()
                image.save(img_io, 'JPEG')
                img_io.seek(0)
                img_bytes = img_io.read()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + img_bytes + b'\r\n')
            except Exception as e2:
                # BUG 6: placeholder also missing/corrupt — log and keep looping
                print(f"[Flask] Placeholder error: {e2}")

        # ── Rate-limit to TARGET_FPS (BUG 5) ─────────────────────────────────
        elapsed    = time.time() - t0
        sleep_time = FRAME_INTERVAL - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)