#!/usr/bin/env python3
"""
Drone Serial-HTTP Bridge
Forwards HTTP POST /cmd  →  Arduino Nano serial port
Serves index.html on GET /

Usage:
  pip install pyserial
  python bridge.py --port COM3          # Windows
  python bridge.py --port /dev/ttyUSB0  # Linux/Mac

Then open http://localhost:5000 in your browser.
"""

import argparse, threading, serial
from http.server import HTTPServer, BaseHTTPRequestHandler

# ── config ────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--port",  default="COM3",   help="Serial port of the Nano")
parser.add_argument("--baud",  default=115200,   type=int)
parser.add_argument("--http",  default=5000,     type=int, help="Local HTTP port")
args = parser.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=1)
print(f"[bridge] Serial open: {args.port} @ {args.baud}")

# ── serial reader (prints Nano replies to terminal) ───────────────
def serial_reader():
    while True:
        try:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(f"[nano] {line}")
        except Exception as e:
            print(f"[serial error] {e}")

threading.Thread(target=serial_reader, daemon=True).start()

# ── HTTP handler ──────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):   # silence default access log
        pass

    def do_GET(self):
        try:
            html = open("index.html", "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(html)
        except FileNotFoundError:
            self.send_error(404, "index.html not found — put it next to bridge.py")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        cmd    = self.rfile.read(length).decode().strip()
        print(f"[http→serial] {cmd}")
        ser.write((cmd + "\n").encode())
        ser.flush()

        # Wait up to 1 s for a reply from the Nano
        reply = ser.readline().decode(errors="replace").strip()
        print(f"[nano reply] {reply}")

        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(reply.encode())

    def do_OPTIONS(self):   # CORS preflight
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

# ── start server ──────────────────────────────────────────────────
server = HTTPServer(("0.0.0.0", args.http), Handler)
print(f"[bridge] HTTP server on http://localhost:{args.http}")
print(f"[bridge] Open http://localhost:{args.http} in your browser")
print(f"[bridge] Ctrl+C to stop\n")
server.serve_forever()
