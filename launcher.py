"""
launcher.py  ──  Run the entire drone AI pipeline in one command
════════════════════════════════════════════════════════════════
Starts all 3 scripts simultaneously in separate processes:
  1. receive_stream.py   — WebSocket server (port 3001)
  2. send_image_stream.py — Flask MJPEG server (port 5000)
  3. analysing_cap.py    — YOLO human detector

Usage:
  python launcher.py

  # With webcam simulator instead of ESP32:
  python launcher.py --test

Press Ctrl+C to stop everything.
"""

import subprocess
import sys
import time
import os
import argparse
import threading

# ── Config ────────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON   = sys.executable   # use the same python/venv that runs this script

def p(*parts):
    """Build an absolute path relative to launcher.py location."""
    return os.path.join(BASE_DIR, *parts)

SCRIPTS = {
    "receive_stream"    : p("esp32cam-stream", "stream", "receive_stream.py"),
    "send_image_stream" : p("esp32cam-stream", "stream", "send_image_stream.py"),
    "analysing_cap"     : p("analysing_cap.py"),
    "test_cam"          : p("test_cam.py"),
}

# How long to wait between starting each script (seconds)
# receive_stream must be up before test_cam/ESP32 connects
# Flask must be up before analysing_cap connects
STARTUP_DELAYS = {
    "receive_stream"    : 0,
    "send_image_stream" : 1,   # wait 1s after receive_stream
    "analysing_cap"     : 3,   # wait 3s for Flask to be ready
    "test_cam"          : 2,   # wait 2s after receive_stream
}

# ANSI colors for each process log prefix
COLORS = {
    "receive_stream"    : "\033[36m",   # cyan
    "send_image_stream" : "\033[33m",   # yellow
    "analysing_cap"     : "\033[32m",   # green
    "test_cam"          : "\033[35m",   # magenta
}
RESET = "\033[0m"

processes = {}   # name → subprocess.Popen


# ── Log streamer ──────────────────────────────────────────────────────────────
def _stream_output(name: str, pipe):
    """Read lines from a process pipe and print them with a colored prefix."""
    color  = COLORS.get(name, "")
    prefix = f"{color}[{name}]{RESET} "
    try:
        for line in iter(pipe.readline, b""):
            print(prefix + line.decode(errors="replace").rstrip())
    except Exception:
        pass


def _start(name: str):
    """Start a single script as a subprocess and stream its output."""
    path = SCRIPTS[name]
    if not os.path.exists(path):
        print(f"[Launcher] ⚠️  {path} not found — skipping {name}")
        return

    print(f"[Launcher] ▶  Starting {name}…")
    proc = subprocess.Popen(
        [PYTHON, path],
        stdout = subprocess.PIPE,
        stderr = subprocess.STDOUT,   # merge stderr into stdout
        cwd    = BASE_DIR,
    )
    processes[name] = proc

    # Stream output on a daemon thread so it doesn't block the launcher
    t = threading.Thread(
        target = _stream_output,
        args   = (name, proc.stdout),
        daemon = True,
        name   = f"log-{name}",
    )
    t.start()


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Drone AI pipeline launcher")
    parser.add_argument(
        "--test", action="store_true",
        help="Also start test_cam.py (webcam simulator, use when no ESP32)"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  Drone AI Pipeline Launcher")
    print("=" * 60)
    if args.test:
        print("  Mode: TESTING  (webcam simulator enabled)")
    else:
        print("  Mode: ESP32-CAM  (connect ESP32 to WiFi)")
    print("=" * 60)
    print()

    # Build startup sequence
    sequence = ["receive_stream", "send_image_stream", "analysing_cap"]
    if args.test:
        sequence.append("test_cam")

    # Start each script with its delay
    for name in sequence:
        delay = STARTUP_DELAYS[name]
        if delay > 0:
            print(f"[Launcher] Waiting {delay}s before starting {name}…")
            time.sleep(delay)
        _start(name)

    print()
    print("[Launcher] ✅ All scripts running.  Press Ctrl+C to stop all.")
    print()

    # Monitor loop — restart crashed processes
    try:
        while True:
            time.sleep(3)
            for name, proc in list(processes.items()):
                ret = proc.poll()
                if ret is not None:
                    print(f"[Launcher] ⚠️  {name} exited (code {ret}) — restarting…")
                    time.sleep(1)
                    _start(name)
    except KeyboardInterrupt:
        print("\n[Launcher] Ctrl+C received — stopping all processes…")

    # Shutdown all
    for name, proc in processes.items():
        print(f"[Launcher] Stopping {name}…")
        proc.terminate()

    # Wait for clean exit
    for name, proc in processes.items():
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            print(f"[Launcher] Force-killing {name}…")
            proc.kill()

    print("[Launcher] All stopped. Goodbye.")


if __name__ == "__main__":
    main()