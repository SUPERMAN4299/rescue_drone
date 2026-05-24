// Drone Flight Controller — Arduino Nano v1.0
// Ported from ESP32-CAM FC v4.0
//
// Wiring:
//   MPU-6050 : SDA→A4, SCL→A5, VCC→5V, GND, AD0→GND
//   Motor FL : D9   (Timer1 A)
//   Motor FR : D10  (Timer1 B)
//   Motor RR : D3   (Timer2 B)
//   Motor RL : D11  (Timer2 A)
//   Status   : D13  (on = ARMED)
//
// Serial commands (115200 baud, newline-terminated):
//   ARM, DISARM, STATUS, RECAL, RESETSTATS
//   THROTTLE <n>  0–200
//   ROLL <f>      ±30°
//   PITCH <f>     ±30°
//   YAW <f>       ±30°/s

#include <Wire.h>
#include <math.h>

// ── LOOP TIMING ──────────────────────────────────────────────────

#define LOOP_HZ           250UL
#define LOOP_PERIOD_US    (1000000UL / LOOP_HZ)
#define LOOP_DT_F         (1.0f / (float)LOOP_HZ)
#define OVERRUN_LIMIT_US  LOOP_PERIOD_US

// ── PINS ─────────────────────────────────────────────────────────

#define PIN_FL    9
#define PIN_FR   10
#define PIN_RR    3
#define PIN_RL   11
#define PIN_STATUS_LED  13

// ── THROTTLE / SAFETY ────────────────────────────────────────────

#define MIN_THR     0
#define IDLE_THR   55
#define MAX_THR   200
#define PID_LIMIT  50.0f

#define SAFE_ANGLE      25.0f
#define ARM_ANGLE_LIM   10.0f
#define FAILSAFE_MS     500UL
#define CMD_TO_MS      2000UL
#define IMU_PING_MS    1000UL
#define IMU_PING_TICKS (IMU_PING_MS * LOOP_HZ / 1000UL)

#define CAL_SAMPLES    1000
#define MAX_SP          30.0f

// Telemetry at 5 Hz
#define TELEM_TICKS    (LOOP_HZ / 5UL)

// ── FILTER TUNING ────────────────────────────────────────────────
//
// Alpha → fc = α·LOOP_HZ / (2π·(1-α)):
//   LPF_GYRO_ALPHA  0.80 → fc ≈ 159 Hz
//   LPF_ACCEL_ALPHA 0.60 → fc ≈  48 Hz
//   LPF_PID_ALPHA   0.80 → fc ≈ 159 Hz

#define LPF_GYRO_ALPHA    0.80f
#define LPF_ACCEL_ALPHA   0.60f
#define LPF_PID_ALPHA     0.80f

#define KALMAN_Q_ANGLE    0.001f
#define KALMAN_Q_BIAS     0.003f
#define KALMAN_R_MEAS     0.030f

// ── MPU-6050 REGISTERS ───────────────────────────────────────────

#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_XOUT  0x3B     // first of 14 bytes: AX AY AZ TEMP GX GY GZ

// DLPF 21 Hz — must be ≤44 Hz to avoid aliasing at 250 Hz loop rate
#define MPU_DLPF_BW     0x04

// Gyro FS ±500°/s → 65.5 LSB/°/s; Accel FS ±8g → 4096 LSB/g
#define GYRO_SCALE      (1.0f / 65.5f)
#define DEG_PER_RAD     57.2957795f
#define ACCEL_SCALE     (1.0f / 4096.0f)
#define GRAVITY_MS2     9.80665f

// ── FILTER STRUCTURES ────────────────────────────────────────────

struct Kalman {
  float angle;
  float bias;
  float P[2][2];
};

static inline void kalmanInit(Kalman& k, float initAngle) {
  k.angle      = initAngle;
  k.bias       = 0.0f;
  k.P[0][0]    = 0.0f; k.P[0][1] = 0.0f;
  k.P[1][0]    = 0.0f; k.P[1][1] = 0.0f;
}

static float kalmanUpdate(Kalman& k, float newRate,
                          float accelAngle, float dt) {
  float rate  = newRate - k.bias;
  k.angle    += dt * rate;

  k.P[0][0]  += dt * (dt*k.P[1][1] - k.P[0][1] - k.P[1][0] + KALMAN_Q_ANGLE);
  k.P[0][1]  -= dt * k.P[1][1];
  k.P[1][0]  -= dt * k.P[1][1];
  k.P[1][1]  += KALMAN_Q_BIAS * dt;

  float S  = k.P[0][0] + KALMAN_R_MEAS;
  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;
  float y  = accelAngle - k.angle;
  k.angle  += K0 * y;
  k.bias   += K1 * y;

  float P00   = k.P[0][0];
  k.P[0][0]  -= K0 * P00;
  k.P[0][1]  -= K0 * k.P[0][1];
  k.P[1][0]  -= K1 * P00;
  k.P[1][1]  -= K1 * k.P[0][1];

  return k.angle;
}

struct LPF { float out; };

static inline float lpfUpdate(LPF& f, float in, float alpha) {
  f.out += alpha * (in - f.out);
  return f.out;
}

// ── FILTER INSTANCES ─────────────────────────────────────────────

static Kalman kfRoll,  kfPitch;
static LPF    lpfGyroX, lpfGyroY;
static LPF    lpfAccelRoll, lpfAccelPitch;
static LPF    lpfPidRoll, lpfPidPitch, lpfPidYaw;

// ── PID GAINS ────────────────────────────────────────────────────
// Yaw PID is ready but inactive — feed it a yaw-rate measurement to enable.

static float kp_r = 0.5f, ki_r = 0.02f, kd_r = 8.0f;
static float kp_p = 0.5f, ki_p = 0.02f, kd_p = 8.0f;
static float kp_y = 1.0f, ki_y = 0.0f,  kd_y = 0.5f;

// Setpoints written by processCmd(), read by loop().
// Single-core AVR: volatile is sufficient, no mutex needed.
volatile float sp_roll  = 0.0f;
volatile float sp_pitch = 0.0f;
volatile float sp_yaw   = 0.0f;
volatile int   sp_thr   = IDLE_THR;

// ── IMU + FLIGHT STATE ───────────────────────────────────────────

static float rollAngle  = 0.0f;
static float pitchAngle = 0.0f;
static bool  imuOk      = false;

struct CalOffsets {
  float gx, gy, gz;      // gyro bias (°/s)
  float ax, ay;          // accel bias (g)
  float az_scale;        // scale so az_corrected ≈ 1 g at rest
};
static CalOffsets cal = {0, 0, 0, 0, 0, 1.0f};

static float ri = 0, rp = 0;
static float pi_ = 0, pp = 0;
static float yi_ = 0, yp = 0;

static int mFL = 0, mFR = 0, mRR = 0, mRL = 0;

typedef enum : uint8_t { DISARMED = 0, ARMED = 1 } State;
volatile State flightState = DISARMED;

// ── TIMING STATE ─────────────────────────────────────────────────

static uint32_t loopTargetUs   = 0;
static uint32_t tickCount      = 0;
static uint32_t overrunCount   = 0;
static uint32_t worstOverrunUs = 0;
static uint32_t lastTickUs     = 0;

static unsigned long lastSensorMs = 0;
static unsigned long lastCmdMs    = 0;

static char    serBuf[64];
static uint8_t serLen   = 0;
static bool    serReady = false;

// ── MOTOR PWM ────────────────────────────────────────────────────
//
// Timer1 (pins 9,10): Fast PWM mode 14, TOP=ICR1=32767 → ~488 Hz
// Timer2 (pins 11,3): Fast PWM mode 3, prescaler 64    → ~977 Hz

static void motorSetup() {
  pinMode(PIN_FL, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13)  | _BV(WGM12)  | _BV(CS10);
  ICR1   = 32767;
  OCR1A  = 0;
  OCR1B  = 0;

  pinMode(PIN_RL, OUTPUT);
  pinMode(PIN_RR, OUTPUT);
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  OCR2A  = 0;
  OCR2B  = 0;
}

// Timer1 has a 15-bit TOP (32767); shift 8-bit demand left 7 to fill the range.
static inline void mWriteFL(uint8_t v) { OCR1A = (uint16_t)v << 7; }
static inline void mWriteFR(uint8_t v) { OCR1B = (uint16_t)v << 7; }
static inline void mWriteRL(uint8_t v) { OCR2A = v; }
static inline void mWriteRR(uint8_t v) { OCR2B = v; }

static void stopMotors() {
  mFL = mFR = mRR = mRL = MIN_THR;
  mWriteFL(0); mWriteFR(0); mWriteRL(0); mWriteRR(0);
}

static void writeMotors() {
  if (flightState != ARMED) { stopMotors(); return; }
  mWriteFL((uint8_t)mFL);
  mWriteFR((uint8_t)mFR);
  mWriteRL((uint8_t)mRL);
  mWriteRR((uint8_t)mRR);
}

static void mixMotors(float base, float pr, float pp_, float py) {
  pr  = constrain(pr,  -PID_LIMIT, PID_LIMIT);
  pp_ = constrain(pp_, -PID_LIMIT, PID_LIMIT);
  py  = constrain(py,  -PID_LIMIT, PID_LIMIT);
  // X-frame: Roll+ → FL↑ RL↑  Pitch+ → FL↑ FR↑  Yaw+ → FL↑ RR↑
  mFL = constrain((int)(base + pr + pp_ + py), MIN_THR, MAX_THR);
  mFR = constrain((int)(base - pr + pp_ - py), MIN_THR, MAX_THR);
  mRR = constrain((int)(base - pr - pp_ + py), MIN_THR, MAX_THR);
  mRL = constrain((int)(base + pr - pp_ - py), MIN_THR, MAX_THR);
}

// ── MPU-6050 DRIVER ──────────────────────────────────────────────

static void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool initIMU() {
  Wire.begin();
  Wire.setClock(400000UL);

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;

  mpuWriteReg(MPU_PWR_MGMT_1, 0x80);   // device reset
  delay(100);
  mpuWriteReg(MPU_PWR_MGMT_1, 0x01);   // wake, PLL clock
  delay(10);

  mpuWriteReg(MPU_SMPLRT_DIV, 0x00);   // sample rate 1 kHz
  mpuWriteReg(MPU_CONFIG,     MPU_DLPF_BW);
  mpuWriteReg(MPU_GYRO_CFG,   0x08);   // ±500°/s
  mpuWriteReg(MPU_ACCEL_CFG,  0x10);   // ±8 g
  delay(50);
  return true;
}

static bool pingIMU() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission() == 0;
}

// Burst-read 14 bytes (AX AY AZ TEMP GX GY GZ) in one I2C transaction.
static bool mpuRead(int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& gx, int16_t& gy, int16_t& gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read();   // temperature — discard
  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static void calibrateIMU() {
  Serial.println(F("[CAL] Keep STILL & LEVEL ..."));
  int32_t sgx=0, sgy=0, sgz=0, sax=0, say=0, saz=0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    if (mpuRead(ax, ay, az, gx, gy, gz)) {
      sgx += gx; sgy += gy; sgz += gz;
      sax += ax; say += ay; saz += az;
    }
    delayMicroseconds(1000);
  }

  cal.gx = (float)sgx / CAL_SAMPLES * GYRO_SCALE;
  cal.gy = (float)sgy / CAL_SAMPLES * GYRO_SCALE;
  cal.gz = (float)sgz / CAL_SAMPLES * GYRO_SCALE;

  cal.ax = (float)sax / CAL_SAMPLES * ACCEL_SCALE;
  cal.ay = (float)say / CAL_SAMPLES * ACCEL_SCALE;
  float az_mean = (float)saz / CAL_SAMPLES * ACCEL_SCALE;
  cal.az_scale  = (az_mean != 0.0f) ? (1.0f / az_mean) : 1.0f;

  Serial.print(F("[CAL] gx=")); Serial.print(cal.gx, 5);
  Serial.print(F(" gy="));      Serial.print(cal.gy, 5);
  Serial.print(F(" gz="));      Serial.println(cal.gz, 5);
  Serial.print(F("[CAL] ax=")); Serial.print(cal.ax, 5);
  Serial.print(F(" ay="));      Serial.print(cal.ay, 5);
  Serial.print(F(" azScale=")); Serial.println(cal.az_scale, 5);
  Serial.println(F("[CAL] Done"));
}

static void readIMU(float dt) {
  int16_t rax, ray, raz, rgx, rgy, rgz;
  if (!mpuRead(rax, ray, raz, rgx, rgy, rgz)) return;

  float ax = (float)rax * ACCEL_SCALE - cal.ax;
  float ay = (float)ray * ACCEL_SCALE - cal.ay;
  float az = (float)raz * ACCEL_SCALE * cal.az_scale;

  float gxDeg = (float)rgx * GYRO_SCALE - cal.gx;
  float gyDeg = (float)rgy * GYRO_SCALE - cal.gy;

  float gxF = lpfUpdate(lpfGyroX,  gxDeg, LPF_GYRO_ALPHA);
  float gyF = lpfUpdate(lpfGyroY,  gyDeg, LPF_GYRO_ALPHA);

  float aRoll  = atan2f(ay, az) * DEG_PER_RAD;
  float aPitch = atan2f(-ax, az) * DEG_PER_RAD;

  float aRollF  = lpfUpdate(lpfAccelRoll,  aRoll,  LPF_ACCEL_ALPHA);
  float aPitchF = lpfUpdate(lpfAccelPitch, aPitch, LPF_ACCEL_ALPHA);

  rollAngle  = kalmanUpdate(kfRoll,  gxF, aRollF,  dt);
  pitchAngle = kalmanUpdate(kfPitch, gyF, aPitchF, dt);
}

// ── PID CONTROLLER ───────────────────────────────────────────────
// Integral clamped to ±ilim; derivative clamped to ±500°/s² to
// suppress spikes on sensor glitch.

static float pidCalc(float sp, float meas,
                     float kp, float ki, float kd,
                     float& intg, float& prev,
                     float dt, float ilim) {
  float e = sp - meas;
  intg    = constrain(intg + e * dt, -ilim, ilim);
  float D = kd * constrain((e - prev) / dt, -500.0f, 500.0f);
  prev    = e;
  return kp * e + ki * intg + D;
}

// ── ARMING & FAILSAFES ───────────────────────────────────────────

static bool armingChecks() {
  bool ok = true;
  Serial.println(F("\n[PRE-ARM]"));

  if (!imuOk || !pingIMU()) {
    Serial.println(F("  FAIL IMU")); imuOk = false; ok = false;
  } else {
    Serial.println(F("  PASS IMU"));
  }

  if (fabsf(rollAngle) > ARM_ANGLE_LIM) {
    Serial.print(F("  FAIL Roll ")); Serial.println(rollAngle, 1); ok = false;
  } else {
    Serial.print(F("  PASS Roll ")); Serial.println(rollAngle, 1);
  }

  if (fabsf(pitchAngle) > ARM_ANGLE_LIM) {
    Serial.print(F("  FAIL Pitch ")); Serial.println(pitchAngle, 1); ok = false;
  } else {
    Serial.print(F("  PASS Pitch ")); Serial.println(pitchAngle, 1);
  }

  Serial.println(ok ? F(">> SAFE") : F(">> BLOCKED"));
  return ok;
}

static void armDrone() {
  flightState = ARMED;
  ri = rp = pi_ = pp = yi_ = yp = 0.0f;
  lpfPidRoll.out = lpfPidPitch.out = lpfPidYaw.out = 0.0f;
  lastSensorMs = lastCmdMs = millis();
  mWriteFL(IDLE_THR); mWriteFR(IDLE_THR);
  mWriteRL(IDLE_THR); mWriteRR(IDLE_THR);
  mFL = mFR = mRR = mRL = IDLE_THR;
  digitalWrite(PIN_STATUS_LED, HIGH);
  Serial.println(F("ARMED"));
}

static void disarm(const __FlashStringHelper* why) {
  flightState = DISARMED;
  stopMotors();
  digitalWrite(PIN_STATUS_LED, LOW);
  Serial.print(F("DISARMED — "));
  Serial.println(why);
}

static void checkFailsafes() {
  if (fabsf(rollAngle)  > SAFE_ANGLE)          { disarm(F("roll limit"));      return; }
  if (fabsf(pitchAngle) > SAFE_ANGLE)          { disarm(F("pitch limit"));     return; }
  if ((millis()-lastSensorMs) > FAILSAFE_MS)   { disarm(F("sensor timeout"));  return; }
  if (!imuOk)                                  { disarm(F("IMU error"));       return; }
  if ((millis()-lastCmdMs)    > CMD_TO_MS)     { disarm(F("cmd timeout"));     return; }
}

// ── COMMAND PROCESSOR ────────────────────────────────────────────

static void printKV(const __FlashStringHelper* key, float val, int decimals = 2) {
  Serial.print(key); Serial.print(val, decimals);
}

static void processCmd(const char* raw) {
  char cmd[64];
  uint8_t len = 0;
  while (raw[len] && len < 63) {
    cmd[len] = (raw[len] >= 'a' && raw[len] <= 'z')
               ? raw[len] - 32 : raw[len];
    len++;
  }
  cmd[len] = '\0';

  uint8_t start = 0;
  while (cmd[start] == ' ') start++;
  int8_t end = (int8_t)len - 1;
  while (end >= 0 && cmd[end] == ' ') end--;
  if (end < (int8_t)start) return;

  const char* c = cmd + start;

  lastCmdMs = millis();

  if (strcmp_P(c, PSTR("ARM")) == 0) {
    if (flightState == ARMED) { Serial.println(F("Already armed")); return; }
    if (armingChecks()) { armDrone(); Serial.println(F("ARMING SUCCESS")); }
    else                {            Serial.println(F("ARM DENIED"));     }
    return;
  }

  if (strcmp_P(c, PSTR("DISARM")) == 0) {
    disarm(F("pilot")); Serial.println(F("DISARMED")); return;
  }

  if (strcmp_P(c, PSTR("STATUS")) == 0) {
    Serial.print(F("{\"state\":\""));
    Serial.print(flightState == ARMED ? F("ARMED") : F("DISARMED"));
    Serial.print(F("\",\"roll\":")); Serial.print(rollAngle,  2);
    Serial.print(F(",\"pitch\":"));  Serial.print(pitchAngle, 2);
    Serial.print(F(",\"imu\":\""));  Serial.print(imuOk ? F("OK") : F("ERR"));
    Serial.print(F("\",\"thr\":"));  Serial.print(sp_thr);
    Serial.print(F(",\"FL\":"));     Serial.print(mFL);
    Serial.print(F(",\"FR\":"));     Serial.print(mFR);
    Serial.print(F(",\"RR\":"));     Serial.print(mRR);
    Serial.print(F(",\"RL\":"));     Serial.print(mRL);
    Serial.print(F(",\"hz\":"));     Serial.print((int)LOOP_HZ);
    Serial.print(F(",\"ovr\":"));    Serial.print(overrunCount);
    Serial.print(F(",\"worstUs\":")); Serial.print(worstOverrunUs);
    Serial.print(F(",\"kBR\":"));    Serial.print(kfRoll.bias,  4);
    Serial.print(F(",\"kBP\":"));    Serial.print(kfPitch.bias, 4);
    Serial.println('}');
    return;
  }

  if (strcmp_P(c, PSTR("RECAL")) == 0) {
    disarm(F("recal"));
    calibrateIMU();
    kalmanInit(kfRoll,  0.0f);
    kalmanInit(kfPitch, 0.0f);
    lpfGyroX.out = lpfGyroY.out = 0.0f;
    lpfAccelRoll.out = lpfAccelPitch.out = 0.0f;
    overrunCount = worstOverrunUs = 0;
    Serial.println(F("RECALIBRATED"));
    return;
  }

  if (strcmp_P(c, PSTR("RESETSTATS")) == 0) {
    overrunCount = worstOverrunUs = 0;
    Serial.println(F("Stats reset"));
    return;
  }

  if (strncmp_P(c, PSTR("THROTTLE "), 9) == 0) {
    int v = constrain(atoi(c + 9), MIN_THR, MAX_THR);
    sp_thr = v;
    Serial.print(F("THROTTLE=")); Serial.println(v);
    return;
  }

  if (strncmp_P(c, PSTR("ROLL "), 5) == 0) {
    float v = constrain((float)atof(c + 5), -MAX_SP, MAX_SP);
    sp_roll = v;
    Serial.print(F("ROLL=")); Serial.println(v, 2);
    return;
  }

  if (strncmp_P(c, PSTR("PITCH "), 6) == 0) {
    float v = constrain((float)atof(c + 6), -MAX_SP, MAX_SP);
    sp_pitch = v;
    Serial.print(F("PITCH=")); Serial.println(v, 2);
    return;
  }

  if (strncmp_P(c, PSTR("YAW "), 4) == 0) {
    float v = constrain((float)atof(c + 4), -MAX_SP, MAX_SP);
    sp_yaw = v;
    Serial.print(F("YAW=")); Serial.println(v, 2);
    return;
  }

  Serial.print(F("UNKNOWN: ")); Serial.println(raw);
}

// ── SERIAL HANDLER ───────────────────────────────────────────────

static void handleSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serLen > 0) serReady = true;
    } else if (serLen < 63) {
      serBuf[serLen++] = ch;
    }
  }
  if (serReady) {
    serBuf[serLen] = '\0';
    processCmd(serBuf);
    serLen   = 0;
    serReady = false;
  }
}

// ── SETUP ────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Drone FC Nano v1.0 ==="));
  Serial.print(F("[TIMING] "));
  Serial.print((int)LOOP_HZ);
  Serial.print(F(" Hz | period "));
  Serial.print((unsigned long)LOOP_PERIOD_US);
  Serial.print(F(" us | dt "));
  Serial.print(LOOP_DT_F, 6);
  Serial.println(F(" s"));

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  motorSetup();
  stopMotors();

  imuOk = initIMU();
  if (!imuOk) {
    Serial.println(F("[IMU] ERROR — check SDA(A4) SCL(A5)"));
  } else {
    Serial.println(F("[IMU] OK"));
    calibrateIMU();

    // Seed Kalman from first accel reading to avoid startup transient.
    int16_t ax0, ay0, az0, gx0, gy0, gz0;
    if (mpuRead(ax0, ay0, az0, gx0, gy0, gz0)) {
      float afX = (float)ax0 * ACCEL_SCALE - cal.ax;
      float afY = (float)ay0 * ACCEL_SCALE - cal.ay;
      float afZ = (float)az0 * ACCEL_SCALE * cal.az_scale;
      float initR =  atan2f(afY,  afZ) * DEG_PER_RAD;
      float initP =  atan2f(-afX, afZ) * DEG_PER_RAD;
      kalmanInit(kfRoll,  initR);
      kalmanInit(kfPitch, initP);
      lpfAccelRoll.out  = initR;
      lpfAccelPitch.out = initP;
      lpfGyroX.out = lpfGyroY.out = 0.0f;
      lpfPidRoll.out = lpfPidPitch.out = lpfPidYaw.out = 0.0f;
      Serial.print(F("[FILTER] Seeded roll="));
      Serial.print(initR, 2);
      Serial.print(F(" pitch="));
      Serial.println(initP, 2);
    }
  }

  loopTargetUs = micros();
  lastSensorMs = lastCmdMs = millis();
  Serial.println(F("[BOOT] Ready — send ARM\n"));
}

// ── MAIN LOOP ────────────────────────────────────────────────────
//
// 250 Hz spin-wait scheduler (4000 µs budget per tick):
//   Critical path every tick: readIMU → Kalman+LPF → PID → mixMotors → writeMotors
//   Sub-tasks rotated across 4 slots:
//     0: handleSerial   1: I2C health ping   2: reserved   3: telemetry

void loop() {

  // Spin-wait — unsigned wrap handles the ~71-min micros() rollover correctly.
  while ((micros() - loopTargetUs) > 0x80000000UL) { /* spin */ }

  uint32_t tickStartUs = micros();
  loopTargetUs        += (uint32_t)LOOP_PERIOD_US;
  tickCount++;

  // Critical path
  if (imuOk) {
    readIMU(LOOP_DT_F);
    lastSensorMs = millis();
  }

  if (flightState == ARMED) checkFailsafes();

  float lr = sp_roll;
  float lp = sp_pitch;
  float ly = sp_yaw;
  int   lt = sp_thr;

  float pidR = 0.0f, pidP = 0.0f, pidY = 0.0f;

  if (flightState == ARMED) {
    pidR = pidCalc(lr, rollAngle,
                   kp_r, ki_r, kd_r,
                   ri, rp, LOOP_DT_F, 100.0f);

    pidP = pidCalc(lp, pitchAngle,
                   kp_p, ki_p, kd_p,
                   pi_, pp, LOOP_DT_F, 100.0f);

    // Yaw: uncomment when yawRate is available
    // pidY = pidCalc(ly, yawRate, kp_y, ki_y, kd_y, yi_, yp, LOOP_DT_F, 100.0f);

    pidR = lpfUpdate(lpfPidRoll,  pidR, LPF_PID_ALPHA);
    pidP = lpfUpdate(lpfPidPitch, pidP, LPF_PID_ALPHA);
    pidY = lpfUpdate(lpfPidYaw,   pidY, LPF_PID_ALPHA);

    mixMotors((float)lt, pidR, pidP, pidY);
    writeMotors();
  } else {
    stopMotors();
    ri = rp = pi_ = pp = yi_ = yp = 0.0f;
  }

  // Overrun detection (measured after critical path, before sub-tasks)
  uint32_t criticalUs = micros() - tickStartUs;
  if (criticalUs > (uint32_t)LOOP_PERIOD_US) {
    overrunCount++;
    uint32_t excess = criticalUs - (uint32_t)LOOP_PERIOD_US;
    if (excess > worstOverrunUs) worstOverrunUs = excess;
  }

  // Sub-task slots
  switch (tickCount & 0x3U) {

    case 0:
      handleSerial();
      break;

    case 1:
      if ((tickCount % (uint32_t)IMU_PING_TICKS) == 1U) {
        if (imuOk && !pingIMU()) {
          imuOk = false;
          disarm(F("IMU disconnect"));
        }
      }
      break;

    case 2:
      // Reserved — battery ADC, RC receiver, barometer, etc.
      // Keep additions under ~500 µs.
      break;

    case 3:
      if ((tickCount % (uint32_t)TELEM_TICKS) == 3U) {
        uint32_t periodUs = tickStartUs - lastTickUs;

        Serial.print(flightState == ARMED ? F("[ARM]") : F("[DIS]"));
        Serial.print(F(" R:")); Serial.print(rollAngle,  2);
        Serial.print(F(" P:")); Serial.print(pitchAngle, 2);
        Serial.print(F(" rPID:")); Serial.print(pidR, 1);
        Serial.print(F(" pPID:")); Serial.print(pidP, 1);
        Serial.print(F(" FL:")); Serial.print(mFL);
        Serial.print(F(" FR:")); Serial.print(mFR);
        Serial.print(F(" RR:")); Serial.print(mRR);
        Serial.print(F(" RL:")); Serial.print(mRL);
        Serial.print(F(" thr:"));  Serial.print(lt);
        Serial.print(F(" kBR:")); Serial.print(kfRoll.bias,  4);
        Serial.print(F(" kBP:")); Serial.print(kfPitch.bias, 4);
        Serial.print(F(" crit:")); Serial.print(criticalUs);
        Serial.print(F("us per:"));Serial.print(periodUs);
        Serial.print(F("us ovr:")); Serial.print(overrunCount);
        Serial.print(F(" worst:")); Serial.print(worstOverrunUs);
        Serial.println(F("us"));
      }
      break;
  }

  lastTickUs = tickStartUs;
}
