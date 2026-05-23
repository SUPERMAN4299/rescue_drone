// ============================================================
//  MPU6050 + Complementary Filter PID -- Arduino Nano
// ============================================================
//  WIRING:
//    MPU6050  -->  Nano
//    VCC      -->  3.3V  (NOT 5V -- sensor is 3.3V only)
//    GND      -->  GND
//    SDA      -->  A4
//    SCL      -->  A5
//    AD0      -->  GND   (I2C address = 0x68)
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// -- Gyro bias ----------------------------------
float gyroBiasX = 0.0;

// -- PID gains ----------------------------------
float kp = 0.5;
float ki = 0.02;
float kd = 8.0;

// -- PID state ----------------------------------
float target    = 0.0;
float integral  = 0.0;
float prevError = 0.0;
float prevRoll  = 0.0;
float rollAngle = 0.0;

// -- Timing -------------------------------------
unsigned long prevMicros = 0;  // for accurate dt
unsigned long lastPrint  = 0;  // for serial throttle

// -- Peak detection (Ziegler-Nichols Tu) --------
unsigned long peakTime1      = 0;
unsigned long peakTime2      = 0;
unsigned long lastCrossing   = 0;
bool firstPeakFound          = false;
const unsigned long DEBOUNCE_MS = 200;

// -----------------------------------------------
void printDivider() {
  Serial.println(F("----------------------------"));
}

void printAxes(const char* label, float x, float y, float z, const char* unit) {
  Serial.print(label);
  Serial.print(F("  X: ")); Serial.print(x, 3);
  Serial.print(F("  Y: ")); Serial.print(y, 3);
  Serial.print(F("  Z: ")); Serial.print(z, 3);
  Serial.print(F("  "));    Serial.println(unit);
}

// -----------------------------------------------
void setup() {
  // Nano has no native USB -- while(!Serial) hangs forever, so it's removed.
  // 9600 baud is safe for all Nano clones (CH340 / FTDI).
  Serial.begin(9600);

  Serial.println(F("\n=== MPU6050 Initializing ==="));

  if (!mpu.begin()) {
    Serial.println(F("ERROR: MPU6050 not found!"));
    Serial.println(F("Check: SDA->A4, SCL->A5, VCC->3.3V, GND->GND"));
    while (1) delay(10);
  }

  Serial.println(F("MPU6050 found!"));

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println(F("Accel : +-8G"));

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println(F("Gyro  : +-500 deg/s"));

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("DLPF  : 21 Hz"));

  Serial.println(F("\nPID loop running...\n"));
  delay(100);

  prevMicros = micros();  // seed dt timer
}

// -----------------------------------------------
void loop() {

  // -- dt -----------------------------------------
  unsigned long now = micros();
  float dt = (now - prevMicros) / 1e6f;
  prevMicros = now;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.005f;  // sanity guard

  // -- Read sensor --------------------------------
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // -- Complementary filter roll ------------------
  float accelRoll = atan2(accel.acceleration.y,
                          accel.acceleration.z) * 180.0 / PI;

  float gyroRate  = (gyro.gyro.x - gyroBiasX) * 180.0 / PI;

  rollAngle = 0.98f * (rollAngle + gyroRate * dt)
            + 0.02f * accelRoll;

  float roll = rollAngle;

  // -- PID ----------------------------------------
  float error = target - roll;

  float P = kp * error;

  integral += error * dt;
  integral  = constrain(integral, -100.0f, 100.0f);
  float I   = ki * integral;

  float derivative = constrain((error - prevError) / dt, -500.0f, 500.0f);
  float D          = kd * derivative;
  prevError        = error;

  float pid = P + I + D;

  // -- Peak detection (upward zero-crossing) ------
  unsigned long nowMs = millis();
  if (prevRoll < 0.0f && roll > 0.0f &&
      (nowMs - lastCrossing) > DEBOUNCE_MS) {

    lastCrossing = nowMs;

    if (!firstPeakFound) {
      peakTime1      = nowMs;
      firstPeakFound = true;
    } else {
      peakTime2  = nowMs;
      float Tu   = (peakTime2 - peakTime1) / 1000.0f;
      Serial.print(F("Tu = "));
      Serial.print(Tu, 3);
      Serial.println(F(" s"));
      peakTime1 = peakTime2;
    }
  }

  prevRoll = roll;

  // -- Serial output (non-blocking, every 100 ms) --
  if (nowMs - lastPrint >= 100) {
    lastPrint = nowMs;

    printDivider();
    printAxes("Accel(m/s2):",
              accel.acceleration.x,
              accel.acceleration.y,
              accel.acceleration.z, "m/s2");
    printAxes("Gyro(rad/s):",
              gyro.gyro.x,
              gyro.gyro.y,
              gyro.gyro.z, "rad/s");

    Serial.print(F("Roll (deg) : ")); Serial.println(roll,  2);
    Serial.print(F("Error      : ")); Serial.println(error, 2);
    Serial.print(F("PID Output : ")); Serial.println(pid,   2);
    Serial.print(F("dt (ms)    : ")); Serial.println(dt * 1000.0f, 2);
    Serial.print(F("Temp (C)   : ")); Serial.println(temp.temperature, 2);
  }
}
