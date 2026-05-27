# Drone Navigation Stack — Arduino Edition
### `analysing_cap_v6_arduino.py` + `final_verdict_auto.ino`

Human-tracking drone controlled by a PC running YOLOv8 vision, sending flight commands over USB serial to an Arduino Nano running the `final_verdict_auto.ino` flight controller.

---

## How the Two Files Talk to Each Other

```
PC (Python)                          Arduino Nano
─────────────────────────────        ─────────────────────────
master_decision() picks motion  ──►  ARM
                                ──►  THROTTLE 110
                                ──►  PITCH -10.0          (nose-down = forward)
                                ──►  ROLL 0.0
                                ──►  YAW 0.0

STATUS (every 0.8 s keepalive)  ──►  {"state":"ARMED","roll":0.12,...}  ◄──
```

The Python code **never sends the old single-letter tokens** (F, B, H …).  
It sends full commands that the sketch understands: `ARM`, `DISARM`, `THROTTLE`, `ROLL`, `PITCH`, `YAW`.

---

## Hardware Wiring

| Component | Pin / Connection |
|---|---|
| Motor FL | Arduino D9 |
| Motor FR | Arduino D10 |
| Motor RL | Arduino D11 |
| Motor RR | Arduino D3 |
| Status LED | Arduino D13 |
| MPU-6050 SDA | Arduino A4 |
| MPU-6050 SCL | Arduino A5 |
| MPU-6050 VCC | 5 V |
| MPU-6050 AD0 | GND |
| ESP32-CAM | Same Wi-Fi network as PC (stream only) |
| Arduino → PC | USB cable (serial at 115200 baud) |

---

## Step 1 — Flash the Arduino

1. Open `final_verdict_auto.ino` in the Arduino IDE.
2. Select board: **Arduino Nano**, processor: **ATmega328P (Old Bootloader)** if needed.
3. Select the correct COM port and click **Upload**.
4. Open Serial Monitor at **115200 baud** and confirm you see:

```
=== Drone FC Nano v1.0 ===
[IMU] OK
[CAL] Keep STILL & LEVEL ...
[CAL] Done
[BOOT] Ready — send ARM
```

> **If you see `[IMU] ERROR`** — check SDA→A4, SCL→A5, and that AD0 is tied to GND.

---

## Step 2 — Install Python Dependencies

Requires **Python 3.9+**.

```bash
pip install ultralytics opencv-python torch numpy pyserial psutil
```

For NVIDIA GPU (optional, faster inference):
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

---

## Step 3 — Configure the Python File

Open `analysing_cap.py` and change these three things:

**A. Runtime mode** (line ~104):
```python
ACTIVE_MODE: RuntimeMode = RuntimeMode.SAFE_TEST_MODE   # ← start here, no motors spin
```
Change to `REAL_FLIGHT_MODE` only after testing in safe mode.

**B. Arduino serial port** (line ~235 inside `DroneConfig`):
```python
serial_port: str = "COM5"        # Windows
# serial_port: str = "/dev/ttyUSB0"   # Linux
# serial_port: str = "/dev/cu.usbserial-0001"  # macOS
```

**C. ESP32-CAM stream URL** (line ~2561):
```python
stream_url = "http://192.168.1.4:8080/video"   # ← change to your camera's IP
```

---

## Step 4 — Run

```bash
python analysing_cap_v6_arduino.py
```

A window titled **Human Detection** opens. Press **Q** to quit cleanly (safely disarms the drone first).

---

## Startup Sequence (What Happens Automatically)

| # | What the Python does | What the Arduino does |
|---|---|---|
| 1 | Connects to serial port, waits 2 s for bootloader | Boots, calibrates IMU (~4 s, keep still) |
| 2 | Starts keepalive thread (STATUS every 0.8 s) | Sends telemetry every 0.2 s |
| 3 | Detects first motion command | Receives `ARM`, runs pre-arm checks |
| 4 | Sends `THROTTLE` / `ROLL` / `PITCH` / `YAW` | Runs PID loop at 250 Hz |
| 5 | Emergency stop → sends `DISARM` | Stops all motors immediately |

> The Arduino will **auto-disarm** if no command arrives within **2 seconds**.  
> The Python keepalive thread prevents this by sending `STATUS` every 0.8 s.

---

## Pre-Arm Checks (Done by the Arduino)

The sketch refuses `ARM` if any check fails:

| Check | Limit | Fix |
|---|---|---|
| IMU reachable | Must respond on I²C | Check wiring |
| Roll angle | ≤ 10° | Place drone level |
| Pitch angle | ≤ 10° | Place drone level |

You will see `ARMING SUCCESS` or `ARM DENIED` in the serial monitor.

---

## Safety Limits (Automatic Disarm)

The Arduino disarms automatically if:

| Condition | Threshold |
|---|---|
| Roll or pitch exceeds | ±25° |
| No serial command received | > 2 seconds |
| IMU stops responding | Any I²C error |
| Sensor data stalls | > 500 ms gap |

---

## What the HUD Shows (Real Flight Mode)

The on-screen HUD has a `── FC TELEMETRY ──` section showing live data read back from the Arduino:

```
FC State: ARMED   IMU:OK
FC Roll : +0.12°  Pitch:-0.34°
FC Thr  : 110     Ovr:0
FC Motor: FL110 FR108 RL112 RR109
```

And a `FC TX (next)` line showing exactly what will be sent next:
```
FC TX (next): THR=110 R=0.0 P=-10.0 Y=0.0
```

---

## Motion → Serial Command Translation

| Python Motion | Sent to Arduino |
|---|---|
| `MOVE_FORWARD` | `THROTTLE 110` + `PITCH -10.0` |
| `MOVE_FORWARD_FAST` | `THROTTLE 140` + `PITCH -18.0` |
| `MOVE_BACKWARD` | `THROTTLE 90` + `PITCH +10.0` |
| `MOVE_HOVER` | `THROTTLE 75` + level setpoints |
| `MOVE_YAW_LEFT` | `THROTTLE 80` + `YAW -20.0` |
| `MOVE_YAW_RIGHT` | `THROTTLE 80` + `YAW +20.0` |
| `MOVE_EMERGENCY_STOP` | `DISARM` (motors cut immediately) |

> All throttle values are automatically reduced in `SAFE_TEST_MODE`.

---

## Recommended First-Run Checklist

- [ ] Arduino flashed and showing `[BOOT] Ready` in Serial Monitor
- [ ] IMU calibration done (drone was still and level during boot)
- [ ] `ACTIVE_MODE = SAFE_TEST_MODE` set in Python file
- [ ] Correct COM port set in Python file
- [ ] ESP32-CAM IP set in Python file
- [ ] Props **removed** for first test
- [ ] Run Python — confirm HUD shows `FC State: ARMED` after first motion
- [ ] Walk in front of camera — drone should track and show HOVER/TRACK intent
- [ ] Once satisfied, reattach props and switch to `REAL_FLIGHT_MODE`

---

## Troubleshooting

**`Could not open COM5`** — Wrong port. Check Device Manager (Windows) or `ls /dev/tty*` (Linux/Mac). Update `serial_port` in config.

**`ARM DENIED`** — Drone not level at boot, or IMU wiring issue. Check MPU-6050 connections, restart with drone flat and still.

**`[IMU] ERROR`** — SDA/SCL not connected, or AD0 not tied to GND. Recheck wiring.

**Drone disarms immediately after arming** — Python not sending commands fast enough, or serial lag. Confirm keepalive thread started (`[FC keepalive + telemetry threads started]` in console).

**No video / black window** — ESP32-CAM unreachable. Check `stream_url` IP address and that both devices are on the same Wi-Fi network.

**Model download on first run** — Normal. Ultralytics downloads `yolov8n.pt` (~6 MB) automatically if not cached.