/*
 * MPU6050 - 6-Axis IMU Sensor Reader
 * 
 * ACCELEROMETER measures linear acceleration (how fast velocity is changing)
 *   along 3 axes, in m/s². At rest, it detects gravity (~9.8 m/s² on Z-axis).
 *   Think of it as feeling "g-force" — like being pushed into your seat in a car.
 *   Use cases: tilt detection, step counting, vibration sensing.
 *
 * GYROSCOPE measures rotational speed (how fast something is spinning/rotating)
 *   around 3 axes, in rad/s. It detects angular velocity, NOT position.
 *   Think of it as a spin-o-meter — it tells you HOW FAST you're rotating.
 *   Use cases: orientation tracking, rotation rate, stabilization (drones, gimbals).
 *
 * AXES (same for both sensors):
 *   X — forward/backward tilt   (pitch axis)
 *   Y — left/right tilt         (roll axis)
 *   Z — up/down or yaw rotation (yaw axis)
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// ──────────────────────────────────────────
//  Print a divider line to Serial Monitor
// ──────────────────────────────────────────
void printDivider() {
  Serial.println(F("────────────────────────────"));
}

// ──────────────────────────────────────────
//  Print a labeled 3-axis value (X, Y, Z)
// ──────────────────────────────────────────
void printAxes(const char* label, float x, float y, float z, const char* unit) {
  Serial.print(label);
  Serial.print(F("  X: ")); Serial.print(x, 3);
  Serial.print(F("  Y: ")); Serial.print(y, 3);
  Serial.print(F("  Z: ")); Serial.print(z, 3);
  Serial.print(F("  "));    Serial.println(unit);
}

// ──────────────────────────────────────────
//  Setup — runs once on power-on/reset
// ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for Serial Monitor to open (USB boards only)

  Serial.println(F("\n=== MPU6050 Initializing ==="));

  // Try to find and initialize the sensor on I2C bus
  if (!mpu.begin()) {
    Serial.println(F("ERROR: MPU6050 not found!"));
    Serial.println(F("Check wiring — SDA, SCL, VCC (3.3V), GND"));
    while (1) delay(10);  // Halt forever; nothing to do without the sensor
  }

  Serial.println(F("MPU6050 found!\n"));

  // ── Accelerometer range ──────────────────────────────────────
  // Sets the max g-force it can measure before clipping/saturating.
  // Lower range = more sensitive for small movements.
  // Higher range = handles hard impacts without clipping.
  // Options: 2G, 4G, 8G, 16G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println(F("Accel range : ±8G"));

  // ── Gyroscope range ─────────────────────────────────────────
  // Sets the max rotation speed it can track before clipping.
  // Lower range = more precise for slow rotations.
  // Higher range = handles fast spins (drones, RC cars).
  // Options: 250, 500, 1000, 2000 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println(F("Gyro range  : ±500 deg/s"));

  // ── Digital Low-Pass Filter (DLPF) ──────────────────────────
  // Smooths noisy sensor data by filtering out high-freq vibrations.
  // Lower bandwidth = smoother but slower to respond.
  // Higher bandwidth = faster but noisier.
  // Options: 5, 10, 21, 44, 94, 184, 260 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("DLPF        : 21 Hz"));

  Serial.println(F("\nReading every 500ms...\n"));
  delay(100);
}

// ──────────────────────────────────────────
//  Loop — runs repeatedly after setup()
// ──────────────────────────────────────────
void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  printDivider();

  // ── Accelerometer data ──────────────────────────────────────
  // Values in m/s² (meters per second squared).
  // When lying flat and still:
  //   X ≈ 0 (no sideways force)
  //   Y ≈ 0 (no forward/back force)
  //   Z ≈ ±9.8 (gravity pulling straight down)
  // Tilting the board will shift gravity's effect across X and Y.
  printAxes("Accel (m/s²):",
            accel.acceleration.x,
            accel.acceleration.y,
            accel.acceleration.z,
            "m/s²");

  // ── Gyroscope data ──────────────────────────────────────────
  // Values in rad/s (radians per second).
  // When held still, all values should be near 0.
  // Rotating around X = pitching forward/back (nodding head)
  // Rotating around Y = rolling left/right  (tilting head)
  // Rotating around Z = yawing              (shaking head "no")
  printAxes("Gyro (rad/s): ",
            gyro.gyro.x,
            gyro.gyro.y,
            gyro.gyro.z,
            "rad/s");

  // ── Temperature ─────────────────────────────────────────────
  // Built-in die temperature sensor (chip's own heat, NOT ambient).
  // Useful for calibration drift compensation, not weather readings.
  Serial.print(F("Temp (°C)   :  "));
  Serial.println(temp.temperature, 2);

  delay(500);  // Wait 500ms before next reading
}
