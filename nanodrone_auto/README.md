# Drone Navigation Stack — Arduino Edition

> **Human-tracking autonomous drone** controlled by a PC running YOLOv8 computer vision,
> sending flight commands over USB serial to an Arduino Nano running a 250 Hz PID flight controller.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Structure](#repository-structure)
3. [System Architecture](#system-architecture)
4. [Hardware](#hardware)
   - [Bill of Materials](#bill-of-materials)
   - [Wiring Diagram](#wiring-diagram)
5. [File Reference](#file-reference)
   - [final_verdict_auto.ino](#final_verdict_autoino)
   - [analysing_cap.py](#analysing_cappy)
   - [drone_launcher.py](#drone_launcherpy)
6. [How the Two Files Talk to Each Other](#how-the-two-files-talk-to-each-other)
7. [Installation](#installation)
   - [Step 1 — Flash the Arduino](#step-1--flash-the-arduino)
   - [Step 2 — Install Python Dependencies](#step-2--install-python-dependencies)
8. [Configuration](#configuration)
   - [Runtime Modes](#runtime-modes)
   - [Key Parameters](#key-parameters)
9. [Running the Stack](#running-the-stack)
   - [Using the GUI Launcher (Recommended)](#using-the-gui-launcher-recommended)
   - [Running Manually](#running-manually)
10. [GUI Launcher Reference](#gui-launcher-reference)
11. [Flight Controller Deep Dive](#flight-controller-deep-dive)
    - [Loop Timing](#loop-timing)
    - [IMU Pipeline](#imu-pipeline)
    - [PID Controller](#pid-controller)
    - [Motor Mixing](#motor-mixing)
    - [Serial Protocol](#serial-protocol)
    - [Safety Failsafes](#safety-failsafes)
12. [Navigation Stack Deep Dive](#navigation-stack-deep-dive)
    - [Architecture Layers](#architecture-layers)
    - [AI Decision Layer](#ai-decision-layer)
    - [Navigation FSM](#navigation-fsm)
    - [Emergency Safety Layer](#emergency-safety-layer)
    - [Master Command Arbiter](#master-command-arbiter)
    - [Motor Abstraction Layer](#motor-abstraction-layer)
13. [Motion Primitive Reference](#motion-primitive-reference)
14. [HUD Overlay Reference](#hud-overlay-reference)
15. [Tuning Guide](#tuning-guide)
    - [PID Gains](#pid-gains)
    - [Proximity Thresholds](#proximity-thresholds)
    - [Stability Filters](#stability-filters)
16. [Extending the Stack](#extending-the-stack)
17. [Troubleshooting](#troubleshooting)
18. [Safety Rules](#safety-rules)
19. [Version History](#version-history)

---

## Project Overview

This project is a complete autonomous drone system split across two separate computers:

| Computer | Role | File |
|---|---|---|
| **Arduino Nano** | Low-level flight control — IMU reading, Kalman filtering, PID loops, PWM motor output | `final_verdict_auto.ino` |
| **PC (Python)** | High-level vision + navigation — YOLO object detection, obstacle avoidance, human tracking, command generation | `analysing_cap.py` |
| **PC (Python)** | GUI launcher — configure and run the whole stack without editing code | `drone_launcher.py` |

The PC decides *what* to do (forward, hover, yaw left …) and the Arduino decides *how* to do it (exact PWM values per motor, PID-stabilised attitude control).

---

## Repository Structure

```
drone-nav/
├── final_verdict_auto.ino   # Arduino Nano flight controller (C++)
├── analysing_cap.py         # PC navigation stack (Python)
├── drone_launcher.py        # GUI launcher — no code editing needed
├── drone_config.json        # Auto-saved launcher settings (gitignore this)
├── human_count.txt          # Written at runtime — live human count
└── README.md                # This file
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        PC  (Python)                          │
│                                                             │
│  ESP32-CAM stream ──► OpenCV frames                        │
│                              │                              │
│                         YOLO v8 inference                   │
│                              │                              │
│              ┌───────────────▼──────────────┐               │
│              │      AI Decision Layer        │               │
│              │  (WHERE is the human?)        │               │
│              │  AIIntent: TRACK_LEFT/RIGHT/  │               │
│              │           CENTER / HOVER /    │               │
│              │           SEARCH_TARGET       │               │
│              └───────────────┬──────────────┘               │
│                              │                              │
│              ┌───────────────▼──────────────┐               │
│              │    Navigation FSM             │               │
│              │  Obstacle avoidance, search,  │               │
│              │  emergency stop               │               │
│              │  NavState enum                │               │
│              └───────────────┬──────────────┘               │
│                              │                              │
│              ┌───────────────▼──────────────┐               │
│              │    Master Command Arbiter     │               │
│              │  Priority: Emergency >        │               │
│              │  Navigation > AI > Search     │               │
│              │  MotionPrimitive enum         │               │
│              └───────────────┬──────────────┘               │
│                              │                              │
│              ┌───────────────▼──────────────┐               │
│              │   ArduinoController           │               │
│              │  Translates motion →          │               │
│              │  ARM / THROTTLE / ROLL /      │               │
│              │  PITCH / YAW commands         │               │
│              └───────────────┬──────────────┘               │
└──────────────────────────────┼──────────────────────────────┘
                               │ USB Serial (115200 baud)
┌──────────────────────────────▼──────────────────────────────┐
│                   Arduino Nano                               │
│                                                             │
│  Serial RX ──► processCmd()                                 │
│                     │                                        │
│               ARM / DISARM                                   │
│               THROTTLE n                                     │
│               ROLL ±30°                                      │
│               PITCH ±30°                                     │
│               YAW ±30°/s                                     │
│                     │                                        │
│              250 Hz PID loop                                 │
│              MPU-6050 → Kalman filter                        │
│              roll PID + pitch PID                            │
│              motor mixing (X-frame)                          │
│                     │                                        │
│              D9 D10 D3 D11 ──► 4× motors                    │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware

### Bill of Materials

| Component | Notes |
|---|---|
| Arduino Nano (ATmega328P) | Any clone works |
| MPU-6050 IMU | I²C, 6-axis gyro + accelerometer |
| 4× Brushed coreless motors | 7mm or 8.5mm, matched pair CW/CCW |
| 4× SI2300 N-channel MOSFETs | Motor drivers |
| ESP32-CAM | Video stream only, same Wi-Fi as PC |
| 3.7V LiPo battery | 1S, 500–800 mAh recommended |
| 5V regulator | Powers Arduino and MPU-6050 |
| PC / laptop | Runs Python navigation stack |
| USB A→Mini-B cable | Arduino to PC serial |

### Wiring Diagram

```
Arduino Nano
│
├─ A4 (SDA) ──────────────── MPU-6050 SDA
├─ A5 (SCL) ──────────────── MPU-6050 SCL
├─ 5V ────────────────────── MPU-6050 VCC
├─ GND ───────────────────── MPU-6050 GND
├─ GND ───────────────────── MPU-6050 AD0  (sets I²C address 0x68)
│
├─ D9  (Timer1A, ~488 Hz) ── MOSFET Gate ── Motor FL (Front Left)
├─ D10 (Timer1B, ~488 Hz) ── MOSFET Gate ── Motor FR (Front Right)
├─ D11 (Timer2A, ~977 Hz) ── MOSFET Gate ── Motor RL (Rear Left)
├─ D3  (Timer2B, ~977 Hz) ── MOSFET Gate ── Motor RR (Rear Right)
│
├─ D13 ───────────────────── Status LED (HIGH = armed)
│
└─ USB ───────────────────── PC (serial 115200 baud)

ESP32-CAM ── Wi-Fi ── Router ── Wi-Fi ── PC
```

**X-Frame motor layout (top view):**

```
    FL (D9)  ●─────────● FR (D10)
              \       /
               \     /
                \   /
                 \ /
                  ×
                 / \
                /   \
               /     \
    RL (D11) ●─────────● RR (D3)
```

---

## File Reference

### `final_verdict_auto.ino`

The Arduino flight controller. Runs a hard real-time 250 Hz loop entirely on the AVR microcontroller. No operating system, no dynamic allocation.

**Key sections:**

| Section | What it does |
|---|---|
| Loop timing | Spin-wait scheduler, 4000 µs budget per tick, overrun detection |
| `motorSetup()` | Configures Timer1 (D9/D10) and Timer2 (D11/D3) for PWM |
| `initIMU()` | Wakes MPU-6050, sets ±500°/s gyro, ±8g accel, 21 Hz DLPF |
| `calibrateIMU()` | Averages 1000 samples to compute gyro/accel bias |
| `readIMU()` | Burst-reads 14 bytes, applies bias, LPF, Kalman filter |
| `kalmanUpdate()` | 1D Kalman filter per axis (roll + pitch) |
| `pidCalc()` | Standard PID with integral clamp and derivative spike limiter |
| `mixMotors()` | X-frame mixer: Roll, Pitch, Yaw → per-motor PWM |
| `processCmd()` | Parses newline-terminated ASCII commands from serial |
| `checkFailsafes()` | Auto-disarms on angle/timeout/IMU error |
| `armingChecks()` | Pre-arm: IMU alive, roll ≤10°, pitch ≤10° |

**Loop sub-task rotation (one per 4 ticks = 1 ms):**

| Tick mod 4 | Task |
|---|---|
| 0 | `handleSerial()` — process one incoming command |
| 1 | I²C health ping (every 1 s) |
| 2 | Reserved (battery ADC, RC, barometer …) |
| 3 | Telemetry print at 5 Hz |

---

### `analysing_cap.py`

The PC navigation stack. Multi-threaded Python application.

**Threads:**

| Thread | Role |
|---|---|
| `_frame_reader_loop` | Reads MJPEG frames from ESP32-CAM over HTTP |
| `_yolo_loop` | Runs YOLOv8 tracking inference on latest frame |
| `_motor_ramp_loop` | Smoothly ramps virtual motor PWM targets |
| `FC_Keepalive` | Sends `STATUS` every 0.8 s to beat Arduino's 2 s cmd-timeout |
| `FC_Telemetry` | Reads and parses all serial output from Arduino |
| Main (display) | Calls AI + Nav + Arbiter, draws HUD, shows OpenCV window |

**Key classes:**

| Class | Role |
|---|---|
| `DroneConfig` | Single dataclass holding every tunable parameter |
| `ArduinoController` | Translates motion primitives → ARM/THROTTLE/ROLL/PITCH/YAW |
| `DryRunController` | No-op controller used in non-flight modes |
| `VirtualSensorSuite` | Simulates altitude, velocity, IMU, battery in sim/safe-test mode |
| `DroneMixer` | Maps MotionPrimitive → per-motor PWM, calls `set_motor_speed()` |
| `PerformanceMonitor` | Tracks YOLO ms, frame latency, serial latency, dropped frames |
| `ShutdownManager` | Graceful shutdown: sends ES, joins threads, closes serial, destroys windows |

---

### `drone_launcher.py`

A Tkinter GUI that patches `analysing_cap.py` in-memory and launches it as a subprocess. **The original source file is never modified.**

**Panels:**

| Panel | Purpose |
|---|---|
| Script File | Path to `analysing_cap.py` |
| Runtime Mode | Dropdown with live hint text |
| Serial / Arduino | Port + baud rate |
| ESP32-CAM Stream | Full stream URL |
| YOLO Inference | Model, confidence, image size |
| Console tab | Live coloured subprocess output |
| Serial Monitor tab | Direct Arduino serial read/write with quick-command buttons |
| Config Preview tab | Shows exactly which lines will be patched |

Settings auto-save to `drone_config.json` on quit.

---

## How the Two Files Talk to Each Other

```
PC sends:                        Arduino responds:

ARM\n                ──────►    [PRE-ARM] checks → ARMING SUCCESS
THROTTLE 110\n       ──────►    (sets sp_thr = 110)
PITCH -10.0\n        ──────►    (sets sp_pitch = -10.0°)
ROLL 0.0\n           ──────►    (sets sp_roll = 0.0°)
YAW 0.0\n            ──────►    (sets sp_yaw = 0.0°/s)

STATUS\n             ──────►    {"state":"ARMED","roll":0.12,
                                 "pitch":-0.34,"imu":"OK",
                                 "thr":110,"FL":112,"FR":108,
                                 "RR":111,"RL":109,"hz":250,
                                 "ovr":0,"worstUs":0,...}

DISARM\n             ──────►    DISARMED — pilot
```

The Arduino also streams telemetry at 5 Hz without being asked:
```
[ARM] R:0.12 P:-0.34 rPID:1.2 pPID:-0.8 FL:112 FR:108 RR:111 RL:109 thr:110 ...
```

---

## Installation

### Step 1 — Flash the Arduino

1. Open **Arduino IDE 2.x**
2. Open `final_verdict_auto.ino`
3. **Tools → Board → Arduino Nano**
4. **Tools → Processor → ATmega328P (Old Bootloader)** *(use this if upload fails)*
5. **Tools → Port** → select your COM port
6. Click **Upload**
7. Open Serial Monitor at **115200 baud, Newline line ending**

Expected boot output:
```
=== Drone FC Nano v1.0 ===
[TIMING] 250 Hz | period 4000 us | dt 0.004000 s
[IMU] OK
[CAL] Keep STILL & LEVEL ...
[CAL] gx=0.00123 gy=-0.00045 gz=0.00078
[CAL] ax=0.00234 ay=-0.00156 azScale=1.00321
[CAL] Done
[FILTER] Seeded roll=0.12 pitch=-0.34
[BOOT] Ready — send ARM
```

If you see `[IMU] ERROR` see [Troubleshooting](#troubleshooting).

### Step 2 — Install Python Dependencies

**Minimum (CPU only):**
```bash
pip install ultralytics opencv-python torch numpy pyserial psutil
```

**With NVIDIA GPU (faster inference):**
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install ultralytics opencv-python numpy pyserial psutil
```

**Verify:**
```bash
python -c "import ultralytics, cv2, torch; print('OK')"
```

---

## Configuration

All settings live in `DroneConfig` inside `analysing_cap.py`. When using the GUI launcher you never need to edit the file — the launcher patches them at runtime.

### Runtime Modes

| Mode | Serial | Motors | Use for |
|---|---|---|---|
| `SAFE_TEST_MODE` | Off | Capped at `safe_test_pwm_max` (120/255) | Bench testing, first flights |
| `SIMULATION_MODE` | Off | Virtual only | Algorithm development, no hardware |
| `REAL_FLIGHT_MODE` | On | Full power | Production flight |
| `DEBUG_MODE` | Off | Off | Verbose logging, algorithm debugging |
| `LOW_POWER_MODE` | Off | Off | Frame-skip, reduced resolution |

### Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `serial_port` | `"COM5"` | Arduino serial port |
| `serial_baud` | `115200` | Must match sketch |
| `stream_url` | `http://192.168.1.4:8080/video` | ESP32-CAM MJPEG stream |
| `model` | `yolov8n.pt` | YOLO model — n/s/m/x trade speed vs accuracy |
| `conf` | `0.30` | Detection confidence threshold (0–1) |
| `imgsz` | `416` | YOLO input resolution (pixels) |
| `emergency_area_frac` | `0.68` | Obstacle fill fraction that triggers emergency stop |
| `safe_test_es_frac` | `0.35` | Emergency threshold in safe-test mode |
| `command_hold_time` | `0.55 s` | Minimum time between motion changes |
| `nav_stability_min` | `5` | Consecutive frames before nav decision commits |
| `ai_stability_min` | `5` | Consecutive frames before AI intent commits |

---

## Running the Stack

### Using the GUI Launcher (Recommended)

```bash
python drone_launcher.py
```

1. Set **Script File** → path to `analysing_cap.py`
2. Set **Runtime Mode** → start with `SAFE_TEST_MODE`
3. Set **Port** → your Arduino's COM port
4. Set **Stream URL** → your ESP32-CAM IP
5. Click **💾 Save Config**
6. Click **▶ LAUNCH**
7. Watch the Console tab for output

### Running Manually

If you prefer to run without the launcher, edit these three lines directly in `analysing_cap.py`:

```python
# Line ~104 — runtime mode
ACTIVE_MODE: RuntimeMode = RuntimeMode.SAFE_TEST_MODE

# Line ~235 inside DroneConfig — serial port
serial_port: str = "COM5"

# Line ~2561 — stream URL
stream_url = "http://192.168.1.4:8080/video"
```

Then:
```bash
python analysing_cap.py
```

Press **Q** in the video window or **Ctrl+C** in the terminal to quit cleanly. The shutdown manager sends DISARM before exiting.

---

## GUI Launcher Reference

```
┌─ 🚁 Drone Navigation Launcher ──────────────────── ● STOPPED ─┐
│                                                                 │
│ ┌─ Config (left) ──────┐  ┌─ Console / Serial / Preview ────┐  │
│ │ 📄 Script File       │  │                                  │  │
│ │ ⚙️  Runtime Mode     │  │  [ARM] R:0.12 P:-0.34 ...       │  │
│ │ 🔌 Serial/Arduino    │  │  [Model] ✅ yolov8n.pt loaded    │  │
│ │ 📷 Stream URL        │  │  [BOOT] Ready — send ARM         │  │
│ │ 🤖 YOLO Inference    │  │                                  │  │
│ │                      │  │                                  │  │
│ │  ▶ LAUNCH  ■ STOP   │  │                                  │  │
│ │  💾 Save Config      │  │                                  │  │
│ └──────────────────────┘  └──────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

**Serial Monitor tab** quick-command buttons: `ARM` `DISARM` `STATUS` `RECAL` `RESETSTATS` + free-text entry.

**Config Preview tab** — click "Refresh Preview" to see the exact patched lines before launching.

**`drone_config.json`** — auto-saved on every quit. Delete it to reset to defaults.

---

## Flight Controller Deep Dive

### Loop Timing

```
250 Hz loop = 4000 µs period

Each tick:
  ├── readIMU()         ~500–900 µs (I²C + Kalman)
  ├── checkFailsafes()  ~10 µs
  ├── PID calc          ~20 µs
  ├── mixMotors()       ~5 µs
  ├── writeMotors()     ~5 µs
  └── sub-task (rotated, ≤500 µs budget)

Overruns logged in `overrunCount` and `worstOverrunUs`,
visible in STATUS response and telemetry.
```

### IMU Pipeline

```
Raw MPU-6050 (14-byte burst)
         │
         ▼
Bias subtraction (calibrated at boot)
         │
         ▼
LPF_GYRO  (α=0.80, fc≈159 Hz)   LPF_ACCEL (α=0.60, fc≈48 Hz)
         │                                │
         ▼                                ▼
  Filtered gx, gy            atan2 → roll°, pitch°
         │                                │
         └──────────► Kalman filter ◄─────┘
                      Q_angle = 0.001
                      Q_bias  = 0.003
                      R_meas  = 0.030
                             │
                             ▼
                   rollAngle, pitchAngle (°)
```

### PID Controller

Standard PID with:
- Integral clamped to `±ilim` (100°·s) — prevents windup
- Derivative clamped to `±500°/s²` — suppresses sensor glitch spikes
- Output LPF (α=0.80) — smooths PID output before motor mixing

Default gains (tune for your specific airframe):

| Axis | Kp | Ki | Kd |
|---|---|---|---|
| Roll | 0.5 | 0.02 | 8.0 |
| Pitch | 0.5 | 0.02 | 8.0 |
| Yaw | 1.0 | 0.0 | 0.5 |

> Yaw PID is wired up but inactive — it needs a yaw-rate measurement. Feed `gz` from the gyro to activate it.

### Motor Mixing

X-frame sign convention:

```
Roll+  → right tilt  → FL↑ RL↑  FR↓ RR↓
Pitch+ → nose up     → RR↑ RL↑  FL↓ FR↓
Yaw+   → CW (top)   → FL↑ RR↑  FR↓ RL↓

mFL = base + roll + pitch + yaw
mFR = base − roll + pitch − yaw
mRR = base − roll − pitch + yaw
mRL = base + roll − pitch − yaw
```

All outputs clamped to `[MIN_THR=0, MAX_THR=200]`. PID contributions clamped to `±50` before mixing.

### Serial Protocol

All commands are ASCII, newline-terminated (`\n`). The sketch processes one command per 4 ms (slot 0 of the sub-task rotation).

| Command | Arguments | Effect |
|---|---|---|
| `ARM` | — | Run pre-arm checks; start motors at idle if passed |
| `DISARM` | — | Stop all motors immediately |
| `STATUS` | — | Print JSON telemetry |
| `RECAL` | — | Disarm, re-run IMU calibration, reset filters |
| `RESETSTATS` | — | Zero overrun counters |
| `THROTTLE n` | 0–200 | Set throttle setpoint |
| `ROLL f` | ±30° | Set roll angle setpoint |
| `PITCH f` | ±30° | Set pitch angle setpoint |
| `YAW f` | ±30°/s | Set yaw rate setpoint |

### Safety Failsafes

The Arduino disarms automatically on any of these conditions:

| Condition | Threshold | Constant |
|---|---|---|
| Roll angle exceeded | ±25° | `SAFE_ANGLE` |
| Pitch angle exceeded | ±25° | `SAFE_ANGLE` |
| No serial command received | 2000 ms | `CMD_TO_MS` |
| IMU sensor data gap | 500 ms | `FAILSAFE_MS` |
| IMU I²C error | any | `imuOk` flag |
| Pre-arm roll not level | >10° | `ARM_ANGLE_LIM` |
| Pre-arm pitch not level | >10° | `ARM_ANGLE_LIM` |

---

## Navigation Stack Deep Dive

### Architecture Layers

```
PERCEPTION        _frame_reader_loop → _yolo_loop
      ↓           (camera frames → detected boxes)
AI INTENT         ai_decision()  → AIIntent enum
      ↓           (where is the human?)
NAVIGATION FSM    nav_decision() → NavState enum
      ↓           (obstacle avoidance, search, density)
EMERGENCY LAYER   _check_emergency()
      ↓           (proximity, oscillation detection)
MASTER ARBITER    master_decision() → MotionPrimitive
      ↓           (single authority, priority-ordered)
MOTOR ABSTRACTION drone_mixer.apply() → set_motor_speed()
      ↓           (MotionPrimitive → per-motor PWM)
MOTOR SAFETY      _motor_ramp_loop() → MAX_PWM_STEP
      ↓           (smooth ramp, no sudden spikes)
FLIGHT CONTROLLER ArduinoController.send_motion()
      ↓           (ARM + THROTTLE/ROLL/PITCH/YAW)
ARDUINO NANO      250 Hz PID → PWM → motors
```

### AI Decision Layer

`ai_decision()` answers: **where is the human target?**

| AIIntent | Condition |
|---|---|
| `TRACK_LEFT` | Largest human bounding box is left of centre (beyond dead-band) |
| `TRACK_RIGHT` | Largest human bounding box is right of centre |
| `TRACK_CENTER` | Human is centred — move forward |
| `HOVER` | Human is close and centred, or proximity is borderline |
| `SEARCH_TARGET` | No human visible for `search_enter_delay` (6) consecutive frames |

Stability rules:
- Intent must hold for `ai_stability_min` (5) consecutive frames before committing
- L↔R flips require `lr_switch_cooldown` (6) frames of the new direction to prevent jitter
- Centre dead-band is `ai_center_thresh` = ±22% of frame width

### Navigation FSM

`nav_decision()` answers: **are there obstacles and how bad?**

| NavState | Trigger | Action |
|---|---|---|
| `CLEAR` | No obstacles in centre | Fast forward |
| `CAUTION` | Obstacle ahead, low proximity | Slow forward |
| `BLOCKED_FRONT` | Centre obstacle above `front_danger_frac` (10%) | Backward |
| `BLOCKED_LEFT` | Left obstacle above `side_danger_frac` (5%) | Yaw right |
| `BLOCKED_RIGHT` | Right obstacle above `side_danger_frac` (5%) | Yaw left |
| `DENSE` | ≥3 centre obstacles | Safe search |
| `CEILING` | Top-zone obstacle | Stop |
| `SEARCH` | No obstacles — searching for target | Rotate search cycle |
| `EMERGENCY` | Emergency layer triggered | Emergency stop |

**Search FSM cycle:**  
`SEARCH_LEFT` (2.5 s) → `SEARCH_RIGHT` (2.5 s) → `SCAN_FORWARD` (0.5 s) → repeat  
After 4 cycles without finding a target, a forward scan is attempted if the path is clear.

### Emergency Safety Layer

`_check_emergency()` triggers on:

| Trigger | Threshold |
|---|---|
| Any obstacle fills ≥68% of frame | `emergency_area_frac` |
| Safe-test mode: obstacle ≥35% | `safe_test_es_frac` |
| Dangerous oscillation detected ≥12 times in 1.5 s | `oscillation_guard_limit` |

**Emergency phases:**
1. `ACTIVE` — holds emergency stop for `emergency_hold_duration` (2.5 s), resending every 0.8 s
2. `RECOVERY` — waits until max live proximity drops below `safe_release_frac` (35%)
3. `IDLE` — normal operation resumes

### Master Command Arbiter

`master_decision()` is the **single authority** for drone motion. Priority order:

```
1. EMERGENCY  (stale frame / reader dead / nav emergency / emergency layer active)
   → EMERGENCY_STOP, force=True

2. NAVIGATION (blocked front/left/right, dense, ceiling)
   → BACKWARD / YAW_LEFT / YAW_RIGHT / SAFE_SEARCH / STOP

3. AI TRACKING (human visible and stable)
   → HOVER / YAW_LEFT / YAW_RIGHT / FORWARD_SLOW / FORWARD_FAST

4. SEARCH / CLEAR (no human, path open)
   → SEARCH_LEFT / SEARCH_RIGHT / SCAN_FORWARD / FORWARD_FAST
```

A `command_hold_time` (0.55 s) cooldown prevents flicker between commands. An FSM transition validator (`_validate_transition`) blocks illegal jumps (e.g. BACKWARD → FORWARD_FAST).

### Motor Abstraction Layer

`set_motor_speed(fl, fr, rl, rr)` is the single entry point for all PWM writes.

- In `SAFE_TEST_MODE`, values are capped at `safe_test_pwm_max` (120/255)
- `_motor_ramp_loop` advances actual PWM toward target at `max_pwm_step` (20) per tick every 50 ms
- `get_motor_pwm_snapshot()` returns current (ramped) values for the HUD

**PWM values per motion primitive:**

| Motion | FL | FR | RL | RR |
|---|---|---|---|---|
| FORWARD | 160 | 160 | 160 | 160 |
| FORWARD_FAST | 220 | 220 | 220 | 220 |
| FORWARD_SLOW | 130 | 130 | 130 | 130 |
| BACKWARD | 100 | 100 | 100 | 100 |
| YAW_LEFT | 110 | 160 | 110 | 160 |
| YAW_RIGHT | 160 | 110 | 160 | 110 |
| HOVER | 150 | 150 | 150 | 150 |
| STOP / EMERGENCY | 0 | 0 | 0 | 0 |

---

## Motion Primitive Reference

Full mapping from MotionPrimitive → Arduino serial commands sent:

| Python Motion | Arduino Commands | Effect |
|---|---|---|
| `MOVE_FORWARD` | `THROTTLE 110` + `PITCH -10.0` | Gentle nose-down → forward |
| `MOVE_FORWARD_FAST` | `THROTTLE 140` + `PITCH -18.0` | Steep pitch → fast forward |
| `MOVE_FORWARD_SLOW` | `THROTTLE 90` + `PITCH -6.0` | Shallow pitch → slow creep |
| `MOVE_BACKWARD` | `THROTTLE 90` + `PITCH +10.0` | Nose-up → backward |
| `MOVE_YAW_LEFT` | `THROTTLE 80` + `YAW -20.0` | Yaw left at 20°/s |
| `MOVE_YAW_RIGHT` | `THROTTLE 80` + `YAW +20.0` | Yaw right at 20°/s |
| `MOVE_HOVER` | `THROTTLE 75` | Level hover |
| `MOVE_STOP` | `THROTTLE 55` | Back to idle throttle |
| `MOVE_SEARCH_LEFT` | `THROTTLE 70` + `YAW -12.0` | Slow scan left |
| `MOVE_SEARCH_RIGHT` | `THROTTLE 70` + `YAW +12.0` | Slow scan right |
| `MOVE_SCAN_FORWARD` | `THROTTLE 85` + `PITCH -5.0` | Creep forward while scanning |
| `MOVE_SAFE_SEARCH` | `THROTTLE 65` + `YAW +8.0` | Very slow yaw, low throttle |
| `MOVE_EMERGENCY_STOP` | `DISARM` | All motors cut immediately |

> In `SAFE_TEST_MODE`, throttle values are scaled: `thr × safe_test_pwm_max / 200`

---

## HUD Overlay Reference

The OpenCV display window shows:

**Left side (text HUD):**
```
Humans  : 1              ← YOLO human count
Device  : CPU            ← inference device
Model   : yolov8n.pt  imgsz=416
Mode    : SAFE_TEST_MODE
── CONTROL STACK ──────────
AI INTENT   : AI_HOVER
NAV STATE   : NAV_CLEAR_PATH
MASTER OWNER: AI_TRACKING
MOTION CMD  : MOVE_HOVER
FC TX (next): THR=75 R=0.0 P=0.0 Y=0.0
── FC TELEMETRY ────────────
FC State: ARMED  IMU:OK
FC Roll : +0.12°  Pitch:-0.34°
FC Thr  : 75  Ovr:0
FC Motor: FL75 FR74 RL76 RR75
── DIAGNOSTICS ────────────
Dr State: TRACKING
NAV FSM : NAV_HOVER
Depth   : 24%
Mem Obs : 2
── MOTORS (ramped) ─────────
FL:150  FR:150
RL:150  RR:150
── PERFORMANCE ─────────────
YOLO ms : 45.2
Frm  ms : 12.1
Cmd  ms : 0.8
```

**Right side (simulation panel — safe-test/simulation modes only):**
```
── SIMULATION ────
Alt   : 0.85 m
Vel X : +0.32 m/s
Vel Y : -0.01 m/s
Dist  : 1.42 m
── IMU (MPU6050) ─
Roll  : +0.1°
Pitch : -0.2°
Yaw   : 45.2°
── MOTORS (PWM) ──
FL:150  FR:150
RL:150  RR:150
── ARBITER ───────
Owner : AI_TRACKING
Motion: MOVE_HOVER
Battery: ████░░ 72%  3.81 V
SAFE TEST MODE
```

**Bottom-right corner:** Navigation arrow icon showing current motion direction.

**Zone grid overlay:** Coloured regions showing L/C/R horizontal zones and TOP/MIDDLE/BOTTOM vertical zones used for obstacle classification.

---

## Tuning Guide

### PID Gains

Edit in `final_verdict_auto.ino` around line 115:

```cpp
static float kp_r = 0.5f, ki_r = 0.02f, kd_r = 8.0f;   // Roll
static float kp_p = 0.5f, ki_p = 0.02f, kd_p = 8.0f;   // Pitch
static float kp_y = 1.0f, ki_y = 0.0f,  kd_y = 0.5f;   // Yaw
```

**Tuning procedure:**
1. Start with all gains at zero. Add Kp until the drone oscillates, then back off 30%.
2. Add Kd to damp oscillation (high Kd = twitchy on sensor noise).
3. Add Ki last, small amounts, to correct steady-state lean.
4. Watch `kBR` / `kBP` (Kalman bias) in telemetry — should converge to near zero.

### Proximity Thresholds

In `DroneConfig` inside `analysing_cap.py`:

| Parameter | Default | Effect of raising |
|---|---|---|
| `front_danger_frac` | 0.10 | Backs off earlier from front obstacles |
| `side_danger_frac` | 0.05 | Avoids side obstacles at greater distance |
| `emergency_area_frac` | 0.68 | Emergency stop triggers less easily |
| `human_hover_frac` | 0.08 | Hovers when human is farther away |
| `safe_release_frac` | 0.35 | Waits longer before leaving recovery |

### Stability Filters

| Parameter | Default | Effect of raising |
|---|---|---|
| `command_hold_time` | 0.55 s | Fewer command changes, more stable, slower response |
| `nav_stability_min` | 5 | Nav decisions commit more slowly |
| `ai_stability_min` | 5 | AI intent commits more slowly |
| `ai_center_thresh` | 0.22 | Wider dead-band, less L↔R chasing |
| `lr_switch_cooldown` | 6 | More frames needed before L↔R flip |
| `depth_ema_alpha` | 0.25 | Lower = heavier smoothing on proximity |

---

## Extending the Stack

The codebase has hardware-ready comment markers (`# HARDWARE-READY COMMENT`) at every integration point:

**Add a ToF / ultrasonic sensor:**
1. Create a concrete `AbstractSensorFusion` subclass
2. Replace `sensor_fusion = NullSensorFusion()` with your implementation
3. Uncomment the sensor blend block in `estimate_pseudo_depth_v3()`

**Enable real MPU-6050 PID on the Python side:**
1. Parse real IMU values from the Arduino's telemetry stream
2. Replace `virtual_sensors.state.imu_*` with real values
3. Implement `PIDHook.compute()` in the stub classes
4. Feed outputs into `DroneMixer` offsets before `set_motor_speed()`

**Add GPS / coordinate navigation:**
1. Create a `CoordinateNavigator` class with a waypoint queue
2. Replace or augment the search FSM with coordinate-driven nav
3. Add it as a new priority layer in `master_decision()`

**Use the Arduino's reserved slot (tick mod 4 == 2):**
```cpp
case 2:
    // Add: battery ADC read, RC receiver parse, barometer, etc.
    // Keep under ~500 µs
    break;
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `[IMU] ERROR` on boot | SDA/SCL miswired or AD0 not at GND | Check A4→SDA, A5→SCL, AD0→GND |
| `ARM DENIED` | Drone not level at calibration | Place flat and still, then restart Arduino |
| `Could not open COM5` | Wrong serial port | Check Device Manager / `ls /dev/tty*`; update port in launcher |
| Drone disarms immediately | Python keepalive not running or serial lag | Confirm `FC keepalive + telemetry threads started` in console |
| Black video window | Wrong stream URL or camera not on same Wi-Fi | Ping ESP32-CAM IP; update `stream_url` |
| High `overrunCount` in telemetry | Critical path too slow (I²C congestion) | Lower `Wire.setClock` or reduce DLPF bandwidth |
| Drone drifts in one direction | IMU calibration bias off | Issue `RECAL` command and keep drone still during recalibration |
| Emergency stop won't release | Obstacle still too close | Move obstacle; recovery waits for proximity < 35% |
| Model not found error | `yolov8n.pt` not downloaded | Run once with internet; it auto-downloads (~6 MB) |
| Excessive L↔R oscillation | `ai_center_thresh` too small | Raise to 0.25–0.30 in `DroneConfig` |
| Motor spinning at wrong speed | Timer prescaler mismatch | Verify D9/D10 use Timer1, D3/D11 use Timer2 |

---

## Safety Rules

### Automatic (enforced by the code)

- Arduino **auto-disarms** on roll/pitch >25°, no command for 2 s, or IMU error
- Python sends **DISARM** on emergency stop, stale frame, or reader thread death
- `SAFE_TEST_MODE` caps PWM at 120/255 and disables `FORWARD_FAST`
- Pre-arm checks block arming if the drone is not level (±10°)
- Motor ramp limiter (`max_pwm_step=20`) prevents sudden PWM spikes

### Manual (your responsibility)

- **Always remove propellers during software testing**
- **Always start in `SAFE_TEST_MODE`** — switch to `REAL_FLIGHT_MODE` only after confirming correct behaviour
- **Never arm indoors** without a safety net or cage on first flights
- Keep a finger on the laptop's **Q** key to quit and disarm instantly
- Calibrate the IMU (issue `RECAL`) any time the drone is moved or the environment changes significantly

---

## Version History

| Version | File | Changes |
|---|---|---|
| v1.0 | `final_verdict_auto.ino` | Initial port from ESP32-CAM FC v4.0 to Arduino Nano; 250 Hz loop, Kalman filter, X-frame mixer, full serial protocol |
| v4.0 | `analysing_cap.py` | ESP32-CAM era; YOLO tracking, navigation FSM, emergency layer, master arbiter |
| v5.0 | `analysing_cap.py` | Safe-test mode, motor abstraction layer, virtual sensor suite, PID hooks (roll/pitch), serial stale-command TTL, motor ramp limiter |
| v6.0 | `analysing_cap.py` | Stability pass: wider dead-band, L↔R cooldown, search-enter delay, forward-stability guard, nav-override guard, secondary proximity EMA, hover buffer frame |
| v1.0 | `drone_launcher.py` | GUI launcher — runtime patching, console output, serial monitor, config save/load |