# 🚁 Drone Flight Controller — Arduino Nano v1.0

A complete drone flight controller system built on an Arduino Nano with MPU-6050 IMU, featuring a browser-based web controller and a Python serial bridge.

---

## 📁 Project Files

| File | Description |
|---|---|
| `esp32cam_drone_fc.ino` | Arduino Nano flight controller firmware |
| `drone_controller.html` | Browser-based web controller UI |
| `controller_page.h` | HTML page as a C string literal (for ESP32 web server) |
| `bridge.py` | Python HTTP-to-Serial bridge (PC-based control) |

---

## 🏗️ System Architecture

```
┌─────────────────┐        HTTP POST /cmd        ┌──────────────┐
│  Browser        │ ──────────────────────────▶  │  bridge.py   │
│  (controller    │ ◀──────────── reply ───────── │  (Python,    │
│   HTML page)    │                               │   on PC)     │
└─────────────────┘                               └──────┬───────┘
                                                         │ Serial
                                                         │ 115200 baud
                                                  ┌──────▼───────┐
                                                  │ Arduino Nano │
                                                  │  + MPU-6050  │
                                                  │  + 4 Motors  │
                                                  └──────────────┘
```

---

## 🔧 Hardware Requirements

- Arduino Nano (ATmega328P)
- MPU-6050 IMU module
- 4x Brushed DC motors
- 4x MOSFETs or motor driver (e.g. L298N) — one per motor
- LiPo or Li-ion battery (suitable for your motors)
- USB cable (Mini-B or Micro-B) for flashing and PC control
- Drone frame (X-configuration)

---

## 🔌 Wiring

### MPU-6050 → Arduino Nano

| MPU-6050 Pin | Nano Pin |
|---|---|
| VCC | 5V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |
| AD0 | GND |

### Motors → Arduino Nano

| Motor Position | Nano Pin | Timer |
|---|---|---|
| Front-Left (FL) | D9 | Timer1 A |
| Front-Right (FR) | D10 | Timer1 B |
| Rear-Right (RR) | D3 | Timer2 B |
| Rear-Left (RL) | D11 | Timer2 A |

> ⚠️ **Never connect motors directly to Nano pins.** Use MOSFETs or a motor driver. Motor power must come from the battery, not the Nano's 5V rail.

### Motor Layout (X-frame top view)

```
    FL(D9)   FR(D10)
       \       /
        \     /
         [FC]
        /     \
       /       \
   RL(D11)   RR(D3)
```

---

## ⚙️ Firmware — `esp32cam_drone_fc.ino`

### Features

- 250 Hz control loop with spin-wait scheduler
- Kalman filter for roll and pitch angle estimation
- Low-pass filters on gyro, accel, and PID outputs
- MPU-6050 auto-calibration on boot
- PID controllers for roll, pitch, and yaw (yaw ready, pending rate sensor)
- X-frame motor mixing
- Pre-arm safety checks (IMU, level check)
- Failsafes: tilt limit, sensor timeout, command timeout, IMU disconnect
- 5 Hz telemetry stream over Serial
- Both long-form and short-form serial commands

### Serial Commands

Connect at **115200 baud, newline-terminated**.

| Command | Description |
|---|---|
| `ARM` | Run pre-arm checks and arm the drone |
| `DISARM` | Disarm and stop all motors |
| `STATUS` | Return JSON telemetry snapshot |
| `RECAL` | Disarm, recalibrate IMU, reset filters |
| `RESETSTATS` | Reset overrun counters |
| `THROTTLE <n>` | Set throttle 0–200 (long form) |
| `ROLL <f>` | Set roll setpoint ±30° (long form) |
| `PITCH <f>` | Set pitch setpoint ±30° (long form) |
| `YAW <f>` | Set yaw rate setpoint ±30°/s (long form) |
| `T:<n>` | Set throttle 0–200 (short form, from web UI) |
| `R:<f>` | Set roll setpoint ±30° (short form) |
| `P:<f>` | Set pitch setpoint ±30° (short form) |
| `Y:<f>` | Set yaw setpoint ±30°/s (short form) |

### Safety Limits

| Parameter | Value |
|---|---|
| Max tilt before disarm | ±25° |
| Max arm angle | ±10° |
| Command timeout | 2000 ms |
| Sensor timeout | 500 ms |
| Max setpoint | ±30° |

### PID Gains (default)

| Axis | Kp | Ki | Kd |
|---|---|---|---|
| Roll | 0.5 | 0.02 | 8.0 |
| Pitch | 0.5 | 0.02 | 8.0 |
| Yaw | 1.0 | 0.0 | 0.5 |

---

## 💻 Setup & Installation

### 1. Flash the Firmware

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Open `esp32cam_drone_fc.ino`
3. Select **Tools → Board → Arduino Nano**
4. Select **Tools → Processor → ATmega328P**
   - If upload fails, try **ATmega328P (Old Bootloader)**
5. Select your COM port under **Tools → Port**
6. Click **Upload**

### 2. Install Python Bridge Dependency

```bash
pip install pyserial
```

### 3. Run the Bridge

Place `bridge.py` and `drone_controller.html` in the same folder, then:

```bash
# Windows
python bridge.py --port COM3

# Linux
python bridge.py --port /dev/ttyUSB0

# Mac
python bridge.py --port /dev/cu.usbserial-XXXX
```

**Finding your port:**
- Windows: Device Manager → Ports (COM & LPT)
- Linux: `ls /dev/ttyUSB*`
- Mac: `ls /dev/cu.*`

### 4. Open the Controller

Go to **http://localhost:5000** in your browser.

---

## 🕹️ Flying the Drone

### Pre-flight Checklist

- [ ] Drone on flat, level surface
- [ ] MPU-6050 wired correctly
- [ ] Motors wired through driver/MOSFETs
- [ ] Battery connected
- [ ] Bridge script running, terminal shows `[BOOT] Ready`
- [ ] Browser shows **ONLINE** (green dot)
- [ ] Roll and pitch angles within ±10° in telemetry

### Arming

1. Set throttle slider to **0**
2. Click **▶ ARM**
3. Terminal must show `PASS IMU`, `PASS Roll`, `PASS Pitch`
4. STATUS tile turns green and flashes **ARMED**
5. Motors spin at idle — keep fingers clear

### Controls

| Control | Action |
|---|---|
| Throttle slider | Increase/decrease power to all motors |
| Roll joystick (left/right) | Tilt drone left or right |
| Pitch joystick (up/down) | Tilt drone forward or backward |
| Yaw strip (left/right) | Rotate drone left or right |
| ■ DISARM | Disarm safely |
| ⚠ STOP | Emergency stop — cuts all motors instantly |

### First Flight Sequence

1. Raise throttle slowly to ~80–100
2. Increase until drone lifts off
3. Use small roll/pitch inputs to hover and stabilize
4. Lower throttle slowly to land
5. Click **DISARM** once on the ground

### Auto-Disarm Triggers (Failsafes)

| Trigger | Cause |
|---|---|
| Roll or pitch > ±25° | Tilt limit exceeded |
| No command for 2 s | Controller disconnected |
| IMU read failure for 500 ms | Sensor timeout |
| IMU not responding on I2C ping | Hardware disconnect |

---

## 🔩 PID Tuning

Edit these values in `esp32cam_drone_fc.ino` and re-flash:

```cpp
static float kp_r = 0.5f;   // Roll  — increase if sluggish
static float ki_r = 0.02f;  //       — increase to fix steady drift
static float kd_r = 8.0f;   //       — increase to dampen oscillations

static float kp_p = 0.5f;   // Pitch
static float ki_p = 0.02f;
static float kd_p = 8.0f;
```

**Tuning guide:**
- Drone oscillates rapidly → reduce `Kd`
- Drone is slow to respond → increase `Kp`
- Drone drifts to one side steadily → increase `Ki`
- Change one value at a time, small steps only

---

## 📡 Telemetry

The firmware streams telemetry at 5 Hz over Serial. Example output:

```
[ARM] R:0.32 P:-0.18 rPID:1.2 pPID:-0.4 FL:62 FR:58 RR:61 RL:59 thr:60 kBR:0.0021 kBP:-0.0014 crit:1823us per:4001us ovr:0 worst:0us
```

The `STATUS` command returns a JSON snapshot:

```json
{
  "state": "ARMED",
  "roll": 0.32,
  "pitch": -0.18,
  "imu": "OK",
  "thr": 60,
  "FL": 62, "FR": 58, "RR": 61, "RL": 59,
  "hz": 250,
  "ovr": 0,
  "worstUs": 0,
  "kBR": 0.0021,
  "kBP": -0.0014
}
```

---

## 🛠️ Troubleshooting

| Problem | Solution |
|---|---|
| `[IMU] ERROR` on boot | Check SDA→A4, SCL→A5, AD0→GND, VCC→5V |
| `ARM DENIED` | Level the drone, wait for angles to settle within ±10° |
| Motors don't spin | Check MOSFET wiring, motor driver power, MIN_THR value |
| Browser shows OFFLINE | Check bridge.py is running and COM port is correct |
| Drone oscillates | Reduce Kd, then Kp |
| Upload fails | Try Old Bootloader processor option in Arduino IDE |
| `UNKNOWN:` in terminal | Command format wrong — check short vs long form |

---

## 📜 License

MIT License — free to use, modify, and distribute.

---

## 🙏 Credits

Ported and extended from ESP32-CAM FC v4.0. Built with Arduino, MPU-6050, and vanilla Python.
