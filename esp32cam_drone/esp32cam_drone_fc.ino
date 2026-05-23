// ================================================================
//  ESP32-CAM INTEGRATED DRONE FIRMWARE
//  Flight Controller + Camera Streamer + Wireless Control
//  ────────────────────────────────────────────────────────────────
// Connect ESP32-CAM via FTDI programmer (TX→RX, RX→TX, GND→GND, 5V→5V)
// Hold IO0/BOOT button, press Reset, release BOOT
// Click Upload in Arduino IDE
// After upload: press Reset once, disconnect IO0
//  Merges:
//    • quad_flight_controller.ino  (MPU6050 / PID / ARM-DISARM)
//    • cam.ino                     (OV2640 / WebSocket JPEG stream)
//
//  Architecture:
//    Core 0 (loop)  → Flight controller: IMU → PID → motor mix → write
//    Core 1 (task)  → Camera capture + WebSocket JPEG stream
//    Web server     → HTTP REST endpoint for wireless commands
//    WebSocket      → Binary JPEG frames to PC viewer
//
//  Wireless Control API (HTTP POST to /cmd):
//    ARM              → run pre-arm checks, arm if passed
//    DISARM           → immediate stop
//    STATUS           → JSON telemetry reply
//    THROTTLE <0-200> → set base throttle (replaces IDLE_THROTTLE)
//    ROLL     <float> → set roll setpoint  (degrees, ±30 max)
//    PITCH    <float> → set pitch setpoint (degrees, ±30 max)
//    YAW      <float> → set yaw setpoint   (degrees/s, ±30 max)
//
//  Packet-loss failsafe:
//    If no wireless command received within WIRELESS_TIMEOUT_MS
//    while ARMED, drone auto-disarms. Keep-alive: send STATUS
//    periodically from your ground station.
//
//  Camera stream:
//    Connect PC to the same Wi-Fi network this AP broadcasts, then
//    run receive_stream.py pointed at WS_PORT (3001).
//    Or open http://<AP_IP>/stream in a browser (MJPEG).
//
//  WIRING (ESP32-CAM AI-Thinker):
//    MPU6050 : VCC→3.3V, GND→GND, SDA→GPIO14, SCL→GPIO15, AD0→GND
//    Motor FL: GPIO12   (PWM via LEDC — NOT analogWrite)
//    Motor FR: GPIO13
//    Motor RR: GPIO2
//    Motor RL: GPIO16
//    Note: GPIO0 used by camera XCLK — do NOT use for motors.
//          GPIO4 is the onboard flash LED.
//          UART0 (TX=GPIO1, RX=GPIO3) reserved for Serial debug.
//
//  IMPORTANT — Pin differences vs original Arduino Nano sketch:
//    The Nano used D3/D5/D6/D9 (AVR PWM) and A4/A5 (I2C).
//    ESP32-CAM uses LEDC for PWM and has fixed I2C pins above.
//    All motor values (0-200) and all PID logic are UNCHANGED.
//
//  LIBRARIES (Arduino IDE → Manage Libraries):
//    ESP32 board support       (Espressif Systems)
//    Adafruit MPU6050
//    Adafruit Unified Sensor
//    ArduinoWebsockets         (Gil Maimon)
//    ESPAsyncWebServer         (ESP Async Web Server — lacamera fork)
//    AsyncTCP                  (required by ESPAsyncWebServer)
//
//  Board settings:
//    Board            : AI Thinker ESP32-CAM
//    Partition Scheme : Huge APP (3MB No OTA)
//    CPU Frequency    : 240 MHz
// ================================================================

// ── Core ESP32 / camera / network includes ───────────────────────
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiAP.h>
#include <Wire.h>
#include <ArduinoWebsockets.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// ── IMU includes ─────────────────────────────────────────────────
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

using namespace websockets;

// ================================================================
//  SECTION 1 — CONFIGURATION  (change these for your setup)
// ================================================================

// ── Wi-Fi Access Point ───────────────────────────────────────────
// The ESP32-CAM hosts its own AP so your PC / phone connects to it.
// Change these to whatever SSID/password you like.
#define AP_SSID     "DroneFC-CAM"
#define AP_PASSWORD "drone1234"   // min 8 chars; leave "" for open AP

// ── WebSocket JPEG stream (cam.ino server port) ──────────────────
// PC running receive_stream.py connects here for live JPEG frames.
const int   WS_PORT  = 3001;
const char* WS_PATH  = "/";

// ── HTTP command server port ─────────────────────────────────────
// POST /cmd  with body = command string (see API at top of file)
const int   HTTP_PORT = 80;

// ── Camera frame config ──────────────────────────────────────────
// QVGA (320×240) keeps bandwidth low and matches the YOLO pipeline.
// Options: FRAMESIZE_96X96 / QQVGA / QVGA / CIF / VGA
#define CAM_FRAME_SIZE  FRAMESIZE_QVGA
#define CAM_JPEG_QUAL   12    // 0=best 63=worst; 10-15 good for live stream
#define CAM_FPS_DELAY   66    // ms between frames ≈ 15 fps

// ── ESP32-CAM I2C pins for MPU6050 ──────────────────────────────
// GPIO14 / GPIO15 are free on AI-Thinker and not used by camera.
#define I2C_SDA  14
#define I2C_SCL  15

// ── ESP32-CAM Motor pins (LEDC channels) ─────────────────────────
// Avoid: 0 (XCLK), 1 (TX), 3 (RX), 4 (flash), 26/27/25/23/22/21/19/18/5/34-39 (camera)
#define PIN_MOTOR_FL  12   // LEDC channel 4
#define PIN_MOTOR_FR  13   // LEDC channel 5
#define PIN_MOTOR_RR   2   // LEDC channel 6  — onboard LED; desolder or ignore blink
#define PIN_MOTOR_RL  16   // LEDC channel 7

// LEDC PWM parameters — matches ~490 Hz of Arduino Nano analogWrite
#define LEDC_FREQ_HZ   490
#define LEDC_RES_BITS    8   // 8-bit → 0-255 range, same as analogWrite
#define LEDC_CH_FL       4
#define LEDC_CH_FR       5
#define LEDC_CH_RR       6
#define LEDC_CH_RL       7

// ── Flash LED ────────────────────────────────────────────────────
#define FLASH_GPIO_NUM   4

// ================================================================
//  SECTION 2 — FLIGHT CONTROLLER CONSTANTS
//  *** IDENTICAL to quad_flight_controller.ino ***
// ================================================================

// ── Throttle constants ───────────────────────────────────────────
#define MIN_THROTTLE      0     // Motors fully off (DISARMED)
#define IDLE_THROTTLE     55    // Gentle spin when ARMED, no lift — tune this
#define MAX_THROTTLE      200   // Hard ceiling — never exceed
#define PID_MAX_CORRECT   50.0f // Max ±PID correction per motor

// ── Safety limits ────────────────────────────────────────────────
#define SAFE_ANGLE_DEG      25.0f  // Auto-disarm if tilt exceeds this
#define ARM_ANGLE_LIMIT     10.0f  // Must be within this angle to arm
#define FAILSAFE_TIMEOUT_MS 500UL  // Auto-disarm if no sensor read within this window
#define IMU_PING_INTERVAL   1000UL // How often to check I2C bus health
#define GYRO_CAL_SAMPLES    500    // Samples averaged for gyro bias calibration

// ── Wireless packet-loss failsafe ────────────────────────────────
// If no command received from ground station within this window
// while ARMED, drone auto-disarms. Send STATUS as keep-alive.
#define WIRELESS_TIMEOUT_MS 2000UL

// ── Complementary filter ─────────────────────────────────────────
#define CF_ALPHA  0.98f  // 98% gyro, 2% accel — increase for smoother, decrease for faster accel response

// ── Setpoint limits — clamp incoming wireless commands ──────────
#define MAX_SETPOINT_DEG  30.0f   // ±30° max roll/pitch from ground station

// ================================================================
//  SECTION 3 — PID GAINS
//  *** IDENTICAL to quad_flight_controller.ino ***
//  Tune order: set ki=kd=0, raise kp until oscillation,
//  halve it, then add kd, then tiny ki last.
// ================================================================
float kp_roll  = 0.5f,  ki_roll  = 0.02f, kd_roll  = 8.0f;
float kp_pitch = 0.5f,  ki_pitch = 0.02f, kd_pitch = 8.0f;  // Activate in loop() when ready
float kp_yaw   = 1.0f,  ki_yaw   = 0.0f,  kd_yaw   = 0.5f;  // Activate when yaw added

// ── Setpoints (0 = level) ────────────────────────────────────────
// Written by web command handler (Core 1 task / ISR context)
// Read by flight controller (Core 0 loop)
// Declared volatile + protected by portMUX for safe cross-core access.
volatile float setpoint_roll  = 0.0f;
volatile float setpoint_pitch = 0.0f;
volatile float setpoint_yaw   = 0.0f;
volatile int   cmd_throttle   = IDLE_THROTTLE; // replaces IDLE_THROTTLE when set wirelessly

portMUX_TYPE setpointMux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
//  SECTION 4 — IMU STATE
//  *** IDENTICAL to quad_flight_controller.ino ***
// ================================================================
Adafruit_MPU6050 mpu;
float rollAngle  = 0.0f;
float pitchAngle = 0.0f;
float gyroBiasX  = 0.0f;
float gyroBiasY  = 0.0f;
bool  imuOk      = false;

// ── PID state ────────────────────────────────────────────────────
float roll_integral  = 0.0f, roll_prevError  = 0.0f;
float pitch_integral = 0.0f, pitch_prevError = 0.0f;

// ── Motor PWM buffers ────────────────────────────────────────────
int motor_FL = 0, motor_FR = 0, motor_RR = 0, motor_RL = 0;

// ── ARM/DISARM state machine ─────────────────────────────────────
typedef enum { DISARMED = 0, ARMED = 1 } FlightState;
volatile FlightState flightState = DISARMED;  // Always boot DISARMED
portMUX_TYPE flightStateMux = portMUX_INITIALIZER_UNLOCKED;

// ── Timing ───────────────────────────────────────────────────────
unsigned long prevMicros          = 0;
unsigned long lastPrintMs         = 0;
unsigned long lastSensorReadMs    = 0;
unsigned long lastImuPingMs       = 0;
volatile unsigned long lastCmdMs  = 0;   // updated on every wireless cmd received

// ── Serial command buffer ────────────────────────────────────────
String serialBuf       = "";
bool   serialLineReady = false;

// ================================================================
//  SECTION 5 — CAMERA / NETWORK STATE
//  *** from cam.ino, adapted for AP mode ***
// ================================================================

// WebSocket server — sends JPEG frames to PC viewer
WebsocketsServer wsServer;
WebsocketsClient wsStreamClient;
bool wsClientConnected = false;

// HTTP server — receives control commands from ground station
AsyncWebServer httpServer(HTTP_PORT);

// Camera task handle (runs on Core 1)
TaskHandle_t camTaskHandle = NULL;

// ================================================================
//  SECTION 6 — CAMERA PIN CONFIG (AI Thinker ESP32-CAM)
//  *** IDENTICAL to cam.ino ***
// ================================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// ================================================================
//  SECTION 7 — LEDC MOTOR HELPERS (ESP32 replaces analogWrite)
//  The 0-200 value range and all mixing math are UNCHANGED.
// ================================================================

void motorLedcSetup() {
  // Configure LEDC channels — 8-bit resolution, ~490 Hz matches Nano analogWrite
  ledcSetup(LEDC_CH_FL, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcSetup(LEDC_CH_FR, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcSetup(LEDC_CH_RR, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcSetup(LEDC_CH_RL, LEDC_FREQ_HZ, LEDC_RES_BITS);

  ledcAttachPin(PIN_MOTOR_FL, LEDC_CH_FL);
  ledcAttachPin(PIN_MOTOR_FR, LEDC_CH_FR);
  ledcAttachPin(PIN_MOTOR_RR, LEDC_CH_RR);
  ledcAttachPin(PIN_MOTOR_RL, LEDC_CH_RL);
}

// Drop-in replacement for analogWrite() using LEDC
inline void motorWrite(uint8_t ch, int val) {
  ledcWrite(ch, (uint32_t)constrain(val, 0, 255));
}


// ================================================================
//  SECTION 8 — CAMERA INIT
//  *** IDENTICAL to cam.ino initCamera() ***
// ================================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;   // send JPEG directly — no re-encode

  // Frame size: QVGA (320×240) for low latency on CPU pipeline
  // Options: FRAMESIZE_96X96 / QQVGA / QVGA / CIF / VGA
  // Use QVGA for your CPU YOLO pipeline (matches 320×240 in test_cam.py)
  config.frame_size   = CAM_FRAME_SIZE;  // 320×240
  config.jpeg_quality = CAM_JPEG_QUAL;   // 0=best 63=worst; 10-15 is good
  config.fb_count     = 1;              // 1 buffer = always freshest frame
  config.grab_mode    = CAMERA_GRAB_LATEST;  // discard old frames automatically

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] Init failed: 0x%x\n", err);
    return false;
  }

  // Fine-tune image quality
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 0);      // -2 to 2
  s->set_contrast(s, 0);        // -2 to 2
  s->set_saturation(s, 0);      // -2 to 2
  s->set_sharpness(s, 0);       // -2 to 2
  s->set_whitebal(s, 1);        // auto white balance on
  s->set_awb_gain(s, 1);        // auto white balance gain on
  s->set_exposure_ctrl(s, 1);   // auto exposure on
  s->set_aec2(s, 1);            // AEC DSP on
  s->set_ae_level(s, 0);        // -2 to 2
  s->set_gain_ctrl(s, 1);       // auto gain on
  s->set_agc_gain(s, 0);        // 0-30
  s->set_gainceiling(s, (gainceiling_t)0); // 2x
  s->set_bpc(s, 0);             // black pixel correction off
  s->set_wpc(s, 1);             // white pixel correction on
  s->set_raw_gma(s, 1);         // gamma correction on
  s->set_lenc(s, 1);            // lens correction on
  s->set_hmirror(s, 0);         // horizontal mirror
  s->set_vflip(s, 0);           // vertical flip
  s->set_dcw(s, 1);             // downsize enable

  Serial.println("[Camera] ✅ Initialized");
  return true;
}


// ================================================================
//  SECTION 9 — IMU FUNCTIONS
//  *** IDENTICAL to quad_flight_controller.ino ***
// ================================================================
bool initIMU() {
  // ESP32-CAM uses custom I2C pins — must call Wire.begin() before mpu.begin()
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // 21 Hz DLPF attenuates motor vibration
  delay(100);
  return true;
}

void calibrateGyro() {
  double sumX = 0, sumY = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    delay(2);
  }
  gyroBiasX = sumX / GYRO_CAL_SAMPLES;
  gyroBiasY = sumY / GYRO_CAL_SAMPLES;
}

// Complementary filter: angle = α*(angle + gyroRate*dt) + (1-α)*accelAngle
// Gyro: fast but drifts. Accel: noisy but drift-free. Filter blends both.
void readIMU(float dt) {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float accelRoll  = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accelPitch = atan2f(-a.acceleration.x, a.acceleration.z) * 180.0f / PI;
  float gyroRateX  = (g.gyro.x - gyroBiasX) * 180.0f / PI;
  float gyroRateY  = (g.gyro.y - gyroBiasY) * 180.0f / PI;

  rollAngle  = CF_ALPHA * (rollAngle  + gyroRateX * dt) + (1.0f - CF_ALPHA) * accelRoll;
  pitchAngle = CF_ALPHA * (pitchAngle + gyroRateY * dt) + (1.0f - CF_ALPHA) * accelPitch;
}

bool pingIMU() {
  Wire.beginTransmission(0x68);
  return Wire.endTransmission() == 0;
}


// ================================================================
//  SECTION 10 — PID
//  *** IDENTICAL to quad_flight_controller.ino ***
// ================================================================
float computePID(float setpoint, float measurement,
                 float kp, float ki, float kd,
                 float &integral, float &prevError,
                 float dt, float integralLimit) {
  float error      = setpoint - measurement;
  float P          = kp * error;
  integral         = constrain(integral + error * dt, -integralLimit, integralLimit);
  float I          = ki * integral;
  float derivative = constrain((dt > 0.0f) ? (error - prevError) / dt : 0.0f, -500.0f, 500.0f);
  float D          = kd * derivative;
  prevError        = error;
  return P + I + D;
}


// ================================================================
//  SECTION 11 — MOTOR MIXING
//  *** IDENTICAL to quad_flight_controller.ino ***
//  Sign matrix (X-frame):        FL   FR   RR   RL
//    Roll  (right tilt → FL/RL+): +1   -1   -1   +1
//    Pitch (nose down  → FL/FR+): +1   +1   -1   -1
//    Yaw   (CW pair):             +1   -1   +1   -1
// ================================================================
void mixMotors(float base, float pidRoll, float pidPitch, float pidYaw) {
  pidRoll  = constrain(pidRoll,  -PID_MAX_CORRECT, PID_MAX_CORRECT);
  pidPitch = constrain(pidPitch, -PID_MAX_CORRECT, PID_MAX_CORRECT);
  pidYaw   = constrain(pidYaw,   -PID_MAX_CORRECT, PID_MAX_CORRECT);

  motor_FL = constrain((int)(base + pidRoll + pidPitch + pidYaw), MIN_THROTTLE, MAX_THROTTLE);
  motor_FR = constrain((int)(base - pidRoll + pidPitch - pidYaw), MIN_THROTTLE, MAX_THROTTLE);
  motor_RR = constrain((int)(base - pidRoll - pidPitch + pidYaw), MIN_THROTTLE, MAX_THROTTLE);
  motor_RL = constrain((int)(base + pidRoll - pidPitch - pidYaw), MIN_THROTTLE, MAX_THROTTLE);
}

// Safety gate: disarmed state always wins — no code path can spin motors while disarmed
// Uses LEDC instead of analogWrite — all values 0-200 unchanged.
void writeMotors() {
  if (flightState != ARMED) { stopMotors(); return; }
  motorWrite(LEDC_CH_FL, motor_FL);
  motorWrite(LEDC_CH_FR, motor_FR);
  motorWrite(LEDC_CH_RR, motor_RR);
  motorWrite(LEDC_CH_RL, motor_RL);
}

void stopMotors() {
  motor_FL = motor_FR = motor_RR = motor_RL = MIN_THROTTLE;
  motorWrite(LEDC_CH_FL, MIN_THROTTLE);
  motorWrite(LEDC_CH_FR, MIN_THROTTLE);
  motorWrite(LEDC_CH_RR, MIN_THROTTLE);
  motorWrite(LEDC_CH_RL, MIN_THROTTLE);
}


// ================================================================
//  SECTION 12 — ARM / DISARM
//  *** IDENTICAL to quad_flight_controller.ino ***
// ================================================================

// All conditions must pass before arming is allowed
bool checkArmingConditions() {
  bool safe = true;
  Serial.println(F("\n[PRE-ARM CHECK]"));

  if (!imuOk) {
    Serial.println(F("  FAIL — IMU not initialised (MPU ERROR at boot)"));
    safe = false;
  } else {
    Serial.println(F("  PASS — IMU OK"));
  }

  if (!pingIMU()) {
    Serial.println(F("  FAIL — IMU not responding on I2C right now"));
    imuOk = false; safe = false;
  } else {
    Serial.println(F("  PASS — IMU responding on I2C"));
  }

  if (fabsf(rollAngle) > ARM_ANGLE_LIMIT) {
    Serial.print(F("  FAIL — Roll ")); Serial.print(rollAngle, 1);
    Serial.print(F("° > ±")); Serial.print(ARM_ANGLE_LIMIT, 0); Serial.println(F("° limit"));
    safe = false;
  } else {
    Serial.print(F("  PASS — Roll ")); Serial.print(rollAngle, 1); Serial.println(F("°"));
  }

  if (fabsf(pitchAngle) > ARM_ANGLE_LIMIT) {
    Serial.print(F("  FAIL — Pitch ")); Serial.print(pitchAngle, 1);
    Serial.print(F("° > ±")); Serial.print(ARM_ANGLE_LIMIT, 0); Serial.println(F("° limit"));
    safe = false;
  } else {
    Serial.print(F("  PASS — Pitch ")); Serial.print(pitchAngle, 1); Serial.println(F("°"));
  }

  Serial.println(safe ? F("SAFE TO ARM\n") : F("NOT SAFE TO ARM\n"));
  return safe;
}

void armDrone() {
  portENTER_CRITICAL(&flightStateMux);
  flightState = ARMED;
  portEXIT_CRITICAL(&flightStateMux);

  roll_integral = roll_prevError = pitch_integral = pitch_prevError = 0.0f;
  lastSensorReadMs = millis();
  lastCmdMs        = millis();  // reset wireless timeout clock on arm

  motor_FL = motor_FR = motor_RR = motor_RL = IDLE_THROTTLE;
  motorWrite(LEDC_CH_FL, IDLE_THROTTLE);
  motorWrite(LEDC_CH_FR, IDLE_THROTTLE);
  motorWrite(LEDC_CH_RR, IDLE_THROTTLE);
  motorWrite(LEDC_CH_RL, IDLE_THROTTLE);
  Serial.println(F("ARMING SUCCESS — motors at idle"));
}

void disarmDrone(const char* reason) {
  portENTER_CRITICAL(&flightStateMux);
  flightState = DISARMED;
  portEXIT_CRITICAL(&flightStateMux);

  stopMotors();
  Serial.print(F("DISARMED — ")); Serial.println(reason);
}

// Failsafes — called every loop while ARMED
// *** IDENTICAL to quad_flight_controller.ino + wireless timeout added ***
void checkFailsafes() {
  if (fabsf(rollAngle) > SAFE_ANGLE_DEG) {
    Serial.print(F("FAILSAFE ACTIVATED — Roll ")); Serial.print(rollAngle, 1); Serial.println(F("° exceeded limit"));
    disarmDrone("roll angle limit"); return;
  }
  if (fabsf(pitchAngle) > SAFE_ANGLE_DEG) {
    Serial.print(F("FAILSAFE ACTIVATED — Pitch ")); Serial.print(pitchAngle, 1); Serial.println(F("° exceeded limit"));
    disarmDrone("pitch angle limit"); return;
  }
  if ((millis() - lastSensorReadMs) > FAILSAFE_TIMEOUT_MS) {
    Serial.println(F("FAILSAFE ACTIVATED — sensor timeout"));
    disarmDrone("sensor timeout"); return;
  }
  if (!imuOk) {
    Serial.println(F("FAILSAFE ACTIVATED — IMU health lost"));
    disarmDrone("IMU error"); return;
  }
  // ── Wireless packet-loss failsafe (NEW) ──────────────────────
  // If no command received from ground station within WIRELESS_TIMEOUT_MS,
  // auto-disarm to prevent flyaway on signal loss.
  // Ground station must send STATUS (or any command) as a keep-alive heartbeat.
  if ((millis() - lastCmdMs) > WIRELESS_TIMEOUT_MS) {
    Serial.println(F("FAILSAFE ACTIVATED — wireless signal lost"));
    disarmDrone("wireless timeout"); return;
  }
}


// ================================================================
//  SECTION 13 — COMMAND PROCESSOR
//  Handles commands from BOTH Serial monitor AND HTTP /cmd endpoint.
//  Called from:
//    checkArming()     → Serial path (Core 0)
//    httpCmdHandler()  → HTTP path  (Core 1 / async callback)
// ================================================================
String processCommand(const String& rawCmd) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toUpperCase();

  // Update wireless heartbeat timestamp on every command received
  lastCmdMs = millis();

  // ── ARM ──────────────────────────────────────────────────────
  if (cmd == "ARM") {
    if (flightState == ARMED) {
      return "Already armed. Send DISARM first.";
    } else if (checkArmingConditions()) {
      armDrone();
      return "ARMING SUCCESS";
    } else {
      return "ARM DENIED — pre-arm checks failed (see Serial)";
    }
  }

  // ── DISARM ───────────────────────────────────────────────────
  if (cmd == "DISARM") {
    disarmDrone("pilot command");
    return "DISARMED";
  }

  // ── STATUS — returns JSON telemetry ─────────────────────────
  if (cmd == "STATUS") {
    char buf[256];
    snprintf(buf, sizeof(buf),
      "{\"state\":\"%s\","
      "\"roll\":%.2f,\"pitch\":%.2f,"
      "\"imu\":\"%s\","
      "\"throttle\":%d,"
      "\"motors\":{\"FL\":%d,\"FR\":%d,\"RR\":%d,\"RL\":%d},"
      "\"ws_client\":%s}",
      (flightState == ARMED) ? "ARMED" : "DISARMED",
      rollAngle, pitchAngle,
      imuOk ? "OK" : "ERROR",
      cmd_throttle,
      motor_FL, motor_FR, motor_RR, motor_RL,
      wsClientConnected ? "true" : "false"
    );
    return String(buf);
  }

  // ── THROTTLE <value> ─────────────────────────────────────────
  // Sets the base throttle used by mixMotors (replaces IDLE_THROTTLE).
  // Range 0-200. Example: "THROTTLE 80"
  if (cmd.startsWith("THROTTLE ")) {
    int val = cmd.substring(9).toInt();
    val = constrain(val, MIN_THROTTLE, MAX_THROTTLE);
    portENTER_CRITICAL(&setpointMux);
    cmd_throttle = val;
    portEXIT_CRITICAL(&setpointMux);
    char r[32]; snprintf(r, sizeof(r), "THROTTLE=%d", val);
    return String(r);
  }

  // ── ROLL <degrees> ───────────────────────────────────────────
  // Sets roll setpoint. Range ±MAX_SETPOINT_DEG. Example: "ROLL 5.0"
  if (cmd.startsWith("ROLL ")) {
    float val = constrain(cmd.substring(5).toFloat(), -MAX_SETPOINT_DEG, MAX_SETPOINT_DEG);
    portENTER_CRITICAL(&setpointMux);
    setpoint_roll = val;
    portEXIT_CRITICAL(&setpointMux);
    char r[32]; snprintf(r, sizeof(r), "ROLL=%.2f", val);
    return String(r);
  }

  // ── PITCH <degrees> ──────────────────────────────────────────
  // Sets pitch setpoint. Range ±MAX_SETPOINT_DEG. Example: "PITCH -3.0"
  if (cmd.startsWith("PITCH ")) {
    float val = constrain(cmd.substring(6).toFloat(), -MAX_SETPOINT_DEG, MAX_SETPOINT_DEG);
    portENTER_CRITICAL(&setpointMux);
    setpoint_pitch = val;
    portEXIT_CRITICAL(&setpointMux);
    char r[32]; snprintf(r, sizeof(r), "PITCH=%.2f", val);
    return String(r);
  }

  // ── YAW <degrees/s> ──────────────────────────────────────────
  // Sets yaw rate setpoint. Range ±MAX_SETPOINT_DEG. Example: "YAW 10.0"
  if (cmd.startsWith("YAW ")) {
    float val = constrain(cmd.substring(4).toFloat(), -MAX_SETPOINT_DEG, MAX_SETPOINT_DEG);
    portENTER_CRITICAL(&setpointMux);
    setpoint_yaw = val;
    portEXIT_CRITICAL(&setpointMux);
    char r[32]; snprintf(r, sizeof(r), "YAW=%.2f", val);
    return String(r);
  }

  // ── Unknown ──────────────────────────────────────────────────
  if (cmd.length() > 0) {
    return "UNKNOWN CMD: " + rawCmd + " — use ARM, DISARM, STATUS, THROTTLE, ROLL, PITCH, YAW";
  }
  return "";
}


// ================================================================
//  SECTION 14 — SERIAL INPUT (non-blocking)
//  *** IDENTICAL to quad_flight_controller.ino ***
// ================================================================
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) serialLineReady = true;
    } else if (serialBuf.length() < 64) {
      serialBuf += c;
    }
  }
}

// Processes serial commands — routes through shared processCommand()
void checkArming() {
  if (!serialLineReady) return;
  serialLineReady = false;

  String response = processCommand(serialBuf);
  if (response.length() > 0) Serial.println(response);

  serialBuf = "";
}


// ================================================================
//  SECTION 15 — HTTP SERVER SETUP
//  POST /cmd  body = command string → response = result string / JSON
//  GET  /     → simple status page
// ================================================================
void setupHttpServer() {
  // POST /cmd — wireless control endpoint
  httpServer.on("/cmd", HTTP_POST, [](AsyncWebServerRequest* req) {
    // Body arrives in the onBody handler below; the request handler
    // is registered here for routing only — actual processing happens
    // in the body handler via AsyncCallbackWebHandler approach.
    // For simplicity we use a URL-param fallback here too.
    if (req->hasParam("c", true)) {
      String cmd = req->getParam("c", true)->value();
      String resp = processCommand(cmd);
      req->send(200, "text/plain", resp);
    } else {
      req->send(400, "text/plain", "Send command in POST body or ?c= param");
    }
  });

  // POST /cmd with raw body (preferred — send command string as body)
  httpServer.addHandler(new AsyncCallbackWebHandler()).onRequest(
    [](AsyncWebServerRequest* req) {},
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      // This onBody lambda is called when /cmd receives a body
      if (req->url() == "/cmd" && len > 0) {
        String cmd = String((char*)data).substring(0, len);
        String resp = processCommand(cmd);
        req->send(200, "text/plain", resp);
      }
    }
  );

  // GET / — minimal status page
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    char html[512];
    snprintf(html, sizeof(html),
      "<html><body style='font-family:monospace'>"
      "<h2>DroneFC-CAM</h2>"
      "<p>State: <b>%s</b></p>"
      "<p>Roll: %.1f&deg;  Pitch: %.1f&deg;</p>"
      "<p>IMU: %s | Stream clients: %s</p>"
      "<p>POST commands to <code>/cmd</code></p>"
      "<p>Commands: ARM, DISARM, STATUS, THROTTLE n, ROLL n, PITCH n, YAW n</p>"
      "</body></html>",
      (flightState == ARMED) ? "ARMED" : "DISARMED",
      rollAngle, pitchAngle,
      imuOk ? "OK" : "ERROR",
      wsClientConnected ? "1" : "0"
    );
    req->send(200, "text/html", html);
  });

  httpServer.begin();
  Serial.printf("[HTTP] Server started on port %d\n", HTTP_PORT);
}


// ================================================================
//  SECTION 16 — CAMERA STREAM TASK (runs on Core 1)
//  *** Logic from cam.ino loop(), adapted as FreeRTOS task ***
//  Accepts WebSocket connections, streams JPEG frames continuously.
// ================================================================
void cameraStreamTask(void* pvParams) {
  Serial.println("[CamTask] Starting on Core 1");

  wsServer.listen(WS_PORT);
  Serial.printf("[CamTask] WebSocket server listening on port %d\n", WS_PORT);

  unsigned long lastFrame  = 0;
  unsigned long frameCount = 0;

  while (true) {
    // Accept new client if none connected
    if (!wsClientConnected) {
      if (wsServer.poll()) {
        wsStreamClient     = wsServer.accept();
        wsClientConnected  = true;
        Serial.println("[WS] ✅ Stream client connected");

        wsStreamClient.onEvent([](WebsocketsEvent event, String data) {
          if (event == WebsocketsEvent::ConnectionClosed) {
            wsClientConnected = false;
            Serial.println("[WS] Stream client disconnected");
          } else if (event == WebsocketsEvent::GotPing) {
            // pong handled automatically by library
          }
        });
      }
    }

    // Service WebSocket (pings, close frames, etc.)
    if (wsClientConnected) {
      wsStreamClient.poll();
    }

    // Rate-limit frame capture
    unsigned long now = millis();
    if ((now - lastFrame) < CAM_FPS_DELAY) {
      vTaskDelay(1);
      continue;
    }
    lastFrame = now;

    // Capture frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[Camera] Frame capture failed");
      vTaskDelay(10);
      continue;
    }

    // Send raw JPEG bytes over WebSocket (only if client connected)
    if (wsClientConnected) {
      bool ok = wsStreamClient.sendBinary((const char*)fb->buf, fb->len);
      if (ok) {
        frameCount++;
        if (frameCount % 100 == 0) {
          Serial.printf("[WS] Sent %lu frames  last=%u bytes\n", frameCount, fb->len);
        }
      } else {
        Serial.println("[WS] Send failed — dropping client");
        wsClientConnected = false;
      }
    }

    // IMPORTANT: always return the frame buffer
    esp_camera_fb_return(fb);
  }
}


// ================================================================
//  SECTION 17 — TELEMETRY
//  *** IDENTICAL to quad_flight_controller.ino ***
// ================================================================
void printTelemetry(float pidRoll, float pidPitch, float dt) {
  Serial.print(flightState == ARMED ? F("[ARMED  ] ") : F("[DISARMD] "));
  Serial.print(F("R:")); Serial.print(rollAngle,   1);
  Serial.print(F("° P:")); Serial.print(pitchAngle, 1);
  Serial.print(F("° | rPID:")); Serial.print(pidRoll,  1);
  Serial.print(F(" pPID:"));    Serial.print(pidPitch, 1);
  Serial.print(F(" | FL:")); Serial.print(motor_FL);
  Serial.print(F(" FR:")); Serial.print(motor_FR);
  Serial.print(F(" RR:")); Serial.print(motor_RR);
  Serial.print(F(" RL:")); Serial.print(motor_RL);
  Serial.print(F(" | thr:")); Serial.print(cmd_throttle);
  Serial.print(F(" | dt:")); Serial.print(dt * 1000.0f, 1); Serial.println(F("ms"));
}


// ================================================================
//  SECTION 18 — SETUP
//  Sequence: motors safe → serial → IMU → camera → Wi-Fi AP →
//            HTTP server → cam stream task (Core 1) → FC ready
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== ESP32-CAM DRONE FC — INTEGRATED FIRMWARE ==="));

  // ── 1. Motors FIRST — pins LOW before anything else runs ─────
  motorLedcSetup();
  stopMotors();
  Serial.println(F("[BOOT] Motors → 0 (LEDC)"));

  // ── 2. Flash LED pin ─────────────────────────────────────────
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, HIGH);  // turn on flash LED (matches cam.ino)

  // ── 3. IMU init ──────────────────────────────────────────────
  Serial.println(F("[BOOT] Initialising MPU6050..."));
  imuOk = initIMU();
  if (!imuOk) {
    Serial.println(F("MPU ERROR — IMU init failed! Arming permanently blocked."));
    Serial.printf("  Check: VCC=3.3V, SDA=GPIO%d, SCL=GPIO%d, AD0=GND\n", I2C_SDA, I2C_SCL);
    // Stay DISARMED — do NOT halt so camera + network still run
  } else {
    Serial.println(F("[BOOT] MPU6050 OK"));
    // Gyro calibration — craft must be STILL and LEVEL
    Serial.println(F("[CAL ] Keep craft still — calibrating gyro..."));
    calibrateGyro();
    Serial.print(F("[CAL ] Done. Bias X=")); Serial.print(gyroBiasX, 5);
    Serial.print(F("  Y=")); Serial.println(gyroBiasY, 5);
  }

  // ── 4. Camera init ───────────────────────────────────────────
  Serial.println(F("[BOOT] Initialising OV2640 camera..."));
  if (!initCamera()) {
    Serial.println(F("[BOOT] ⚠ Camera init failed — stream unavailable"));
    // Not fatal — FC continues without camera
  }

  // ── 5. Wi-Fi Access Point ────────────────────────────────────
  Serial.printf("[WiFi] Starting AP: SSID=%s\n", AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print(F("[WiFi] ✅ AP started  IP: ")); Serial.println(apIP);
  Serial.printf("[WiFi] Connect to %s, then:\n", AP_SSID);
  Serial.printf("       HTTP cmds : http://%s/cmd  (POST)\n", apIP.toString().c_str());
  Serial.printf("       JPEG stream: ws://%s:%d%s\n",  apIP.toString().c_str(), WS_PORT, WS_PATH);

  // ── 6. HTTP command server ───────────────────────────────────
  setupHttpServer();

  // ── 7. Camera stream task on Core 1 ─────────────────────────
  // Pinned to Core 1 so flight controller loop() runs undisturbed on Core 0
  xTaskCreatePinnedToCore(
    cameraStreamTask,   // task function
    "CamStreamTask",    // name
    8192,               // stack (bytes) — camera + WS needs ~6 KB
    NULL,               // parameter
    1,                  // priority (1 = low, keeps FC at default 1 on Core 0)
    &camTaskHandle,     // handle
    1                   // core 1
  );

  prevMicros       = micros();
  lastSensorReadMs = millis();
  lastCmdMs        = millis();

  Serial.println(F("[BOOT] ✅ Ready. Send ARM via Serial or HTTP /cmd to arm.\n"));
}


// ================================================================
//  SECTION 19 — MAIN LOOP (Core 0)
//  Order: serial → dt → sensors → failsafes → PID → mix → write
//  *** IDENTICAL to quad_flight_controller.ino loop() ***
//  Only change: setpoints/throttle read via volatile + portMUX.
// ================================================================
void loop() {
  handleSerial();

  // dt
  unsigned long nowUs = micros();
  float dt = (nowUs - prevMicros) / 1e6f;
  prevMicros = nowUs;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

  // Read sensors
  if (imuOk) {
    readIMU(dt);
    lastSensorReadMs = millis();
  }

  // Periodic I2C health ping
  unsigned long nowMs = millis();
  if (nowMs - lastImuPingMs >= IMU_PING_INTERVAL) {
    lastImuPingMs = nowMs;
    if (!pingIMU()) {
      imuOk = false;
      Serial.println(F("MPU ERROR — I2C health check failed!"));
      disarmDrone("MPU6050 disconnected");
    }
  }

  // Process ARM/DISARM/STATUS/setpoint serial commands
  checkArming();

  // Failsafes — only relevant while armed
  if (flightState == ARMED) checkFailsafes();

  float pidRoll = 0.0f, pidPitch = 0.0f;

  // Snapshot volatile setpoints safely
  float sp_roll, sp_pitch, sp_yaw;
  int   base_throttle;
  portENTER_CRITICAL(&setpointMux);
  sp_roll      = setpoint_roll;
  sp_pitch     = setpoint_pitch;
  sp_yaw       = setpoint_yaw;
  base_throttle = cmd_throttle;
  portEXIT_CRITICAL(&setpointMux);

  if (flightState == ARMED) {
    pidRoll = computePID(sp_roll, rollAngle,
                         kp_roll, ki_roll, kd_roll,
                         roll_integral, roll_prevError, dt, 100.0f);

    // Pitch inactive: replace 0.0f gains with kp_pitch, ki_pitch, kd_pitch to enable
    pidPitch = computePID(sp_pitch, pitchAngle,
                          0.0f, 0.0f, 0.0f,
                          pitch_integral, pitch_prevError, dt, 100.0f);

    float pidYaw = 0.0f;  // Add yaw PID here when ready

    mixMotors((float)base_throttle, pidRoll, pidPitch, pidYaw);
    writeMotors();

  } else {
    stopMotors();
    // Reset integrators so there's no windup carried into next arm
    roll_integral = roll_prevError = 0.0f;
    pitch_integral = pitch_prevError = 0.0f;
  }

  if (nowMs - lastPrintMs >= 200) {
    lastPrintMs = nowMs;
    printTelemetry(pidRoll, pidPitch, dt);
  }
}


// ================================================================
//  EXPANSION NOTES (preserved from quad_flight_controller.ino)
//
//  Pitch PID:  In loop(), replace the three 0.0f pitch gains
//              with kp_pitch, ki_pitch, kd_pitch.
//
//  Wireless:   setpoint_roll, setpoint_pitch, setpoint_yaw, and
//              cmd_throttle are updated by processCommand() which
//              is called from both the HTTP handler (Core 1) and
//              the Serial handler (Core 0). Both paths use portMUX
//              for safe cross-core writes. The entire PID + safety
//              system works unchanged below the setpoint snapshot.
//
//  Keep-alive: Ground station MUST send a command (e.g. STATUS or
//              any setpoint update) at least every WIRELESS_TIMEOUT_MS
//              (default 2000 ms) to prevent auto-disarm.
//
//  Camera:     If OV2640 is unavailable, the FC continues normally.
//              Camera task silently skips frame capture on init fail.
// ================================================================
