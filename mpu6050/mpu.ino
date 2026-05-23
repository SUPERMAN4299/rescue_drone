#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// ── Gyro ────────────────────────────
float gyroBiasX = 0.0;

// ── PID gains ─────────────────────────────
float kp = 0.5;
float ki = 0.02;
float kd = 8.0;

// ── PID state ─────────────────────────────
float target     = 0.0;
float integral   = 0.0;
float prevError  = 0.0;
float prevRoll   = 0.0;

float rollAngle = 0.0;

// ── Timing ────────────────────────────────
unsigned long prevMicros  = 0;   // microseconds — for accurate dt
unsigned long lastPrint   = 0;   // milliseconds — for serial output throttle

// ── Peak detection (Ziegler–Nichols Tu) ───
unsigned long peakTime1       = 0;
unsigned long peakTime2       = 0;
unsigned long lastCrossing    = 0;          // debounce guard
bool          firstPeakFound  = false;
const unsigned long DEBOUNCE_MS = 200;      // ignore crossings < 200 ms apart

// ──────────────────────────────────────────
void printDivider() {
  Serial.println(F("────────────────────────────"));
}

void printAxes(const char* label, float x, float y, float z, const char* unit) {
  Serial.print(label);
  Serial.print(F("  X: ")); Serial.print(x, 3);
  Serial.print(F("  Y: ")); Serial.print(y, 3);
  Serial.print(F("  Z: ")); Serial.print(z, 3);
  Serial.print(F("  "));    Serial.println(unit);
}

// ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("\n=== MPU6050 Initializing ==="));

  if (!mpu.begin()) {
    Serial.println(F("ERROR: MPU6050 not found!"));
    Serial.println(F("Check wiring — SDA, SCL, VCC (3.3V), GND"));
    while (1) delay(10);
  }

  Serial.println(F("MPU6050 found!\n"));

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println(F("Accel range : ±8G"));

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println(F("Gyro range  : ±500 deg/s"));

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("DLPF        : 21 Hz"));

  Serial.println(F("\nRunning PID loop (serial prints every 500 ms)...\n"));
  delay(100);

  // ✅ Seed timer so first dt is valid (not a huge garbage value)
  prevMicros = micros();
}

// ──────────────────────────────────────────
void loop() {

    // ── dt in seconds ───────────────────────
  unsigned long now = micros();
  float dt = (now - prevMicros) / 1e6f;
  prevMicros = now;

  // Guard against nonsense dt on first iteration or timer overflow
  if (dt <= 0.0f || dt > 1.0f) dt = 0.005f;


  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // ── Roll angle ──────────────────────────
  float accelRoll = atan2(accel.acceleration.y,
                          accel.acceleration.z) * 180.0 / PI;

  float gyroRate =
    (gyro.gyro.x - gyroBiasX) * 180.0 / PI;

  rollAngle = 0.98f * (rollAngle + gyroRate * dt)
            + 0.02f * accelRoll;

  float roll = rollAngle;



  // ── PID ─────────────────────────────────
  float error = target - roll;

  float P = kp * error;

  integral += error * dt;                        // ✅ time-correct accumulation
  integral  = constrain(integral, -100.0f, 100.0f);
  float I   = ki * integral;

  float derivative = (error - prevError) / dt;

  derivative = constrain(derivative, -500, 500);
    
  float D = kd * derivative;  // derivative
  prevError = error;

  float pid = P + I + D;

  // ── Peak detection (upward zero-crossing) ──
  // ✅ previousRoll < 0 && roll > 0  →  upward crossing = actual peak cycle
  unsigned long nowMs = millis();
  if (prevRoll < 0.0f && roll > 0.0f &&
      (nowMs - lastCrossing) > DEBOUNCE_MS) {     // ✅ debounce

    lastCrossing = nowMs;

    if (!firstPeakFound) {
      peakTime1     = nowMs;
      firstPeakFound = true;
    } else {
      peakTime2 = nowMs;
      float Tu = (peakTime2 - peakTime1) / 1000.0f;
      Serial.print(F("Tu = "));
      Serial.print(Tu, 3);
      Serial.println(F(" s"));
      peakTime1 = peakTime2;   // slide window for continuous measurement
    }
  }

  prevRoll = roll;

  // ── Serial output (non-blocking, every 500 ms) ──
  // ✅ No delay() — loop runs freely for accurate PID timing
  if (nowMs - lastPrint >= 100) { // 500 ms for debugging
    lastPrint = nowMs;

    printDivider();
    printAxes("Accel (m/s²):",
              accel.acceleration.x,
              accel.acceleration.y,
              accel.acceleration.z, "m/s²");
    printAxes("Gyro (rad/s): ",
              gyro.gyro.x,
              gyro.gyro.y,
              gyro.gyro.z, "rad/s");

    Serial.print(F("Roll (°)   : ")); Serial.println(roll,  2);
    Serial.print(F("Error      : ")); Serial.println(error, 2);
    Serial.print(F("PID Output : ")); Serial.println(pid,   2);
    Serial.print(F("dt (ms)    : ")); Serial.println(dt * 1000.0f, 2);
    Serial.print(F("Temp (°C)  : ")); Serial.println(temp.temperature, 2);
  }
}