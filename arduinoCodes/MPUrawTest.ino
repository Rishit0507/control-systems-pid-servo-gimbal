#include <Wire.h>        // I2C communication library
#include "MPU6050.h"    // MPU6050 driver (I2Cdevlib)

MPU6050 mpu;             // Create MPU6050 object

// ---------------- RAW SENSOR DATA ----------------
// These store raw values read directly from the MPU6050
int16_t ax, ay, az;      // Accelerometer raw data
int16_t gx, gy, gz;      // Gyroscope raw data

// ---------------- ANGLE VARIABLES ----------------
// Angles calculated only from accelerometer
float accPitch = 0.0;
float accRoll  = 0.0;

// Angles obtained by integrating gyroscope rates
float gyroPitch = 0.0;
float gyroRoll  = 0.0;

// Final filtered angles (these are what you will use for PID)
float pitch = 0.0;
float roll  = 0.0;

// ---------------- TIMING VARIABLES ----------------
// Used to maintain a fixed control loop timing
unsigned long prevTime = 0;

// Loop period = 2000 microseconds → 500 Hz update rate
const unsigned long loopTime = 2000;

void setup() {
  Serial.begin(115200);   // Start serial communication
  delay(1000);            // Short delay for stability

  // Initialize I2C on ESP32 using custom pins
  // SDA = GPIO 26, SCL = GPIO 27
  Wire.begin(26, 27);

  // Initialize MPU6050
  mpu.initialize();

  // Check if MPU6050 is responding
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection FAILED ❌");
    while (1);            // Stop execution if sensor fails
  }

  Serial.println("MPU6050 connected ✅");

  // ------------------------------------------------
  // IMPORTANT: COMPLEMENTARY FILTER INITIALIZATION
  // ------------------------------------------------
  // The filter MUST start with a correct angle.
  // If this is skipped, the output will appear stuck.

  // Read sensor once
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Compute initial angles using accelerometer only
  accRoll  = atan2(ay, az) * 180.0 / PI;
  accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Initialize gyro angles with accel angles
  // This aligns gyro and accel references
  gyroRoll  = accRoll;
  gyroPitch = accPitch;

  Serial.println("Complementary filter initialized ✅");
}

void loop() {

  // Get current time in microseconds
  unsigned long now = micros();

  // Run the control loop only at fixed intervals
  if (now - prevTime >= loopTime) {

    // Compute loop time in seconds (dt)
    float dt = (now - prevTime) * 1e-6;
    prevTime = now;

    // ---------------- READ MPU DATA ----------------
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // ---------------- ACCELEROMETER ANGLES ----------------
    // These give absolute angle reference using gravity
    accRoll  = atan2(ay, az) * 180.0 / PI;
    accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // ---------------- GYROSCOPE INTEGRATION ----------------
    // MPU6050 sensitivity: 131 LSB per degree/second
    // Gyro gives angular velocity → integrate to get angle
    gyroRoll  += (gx / 131.0) * dt;
    gyroPitch += (gy / 131.0) * dt;

    // ---------------- COMPLEMENTARY FILTER ----------------
    // Gyro = fast response, but drifts
    // Accel = stable reference, but noisy
    // Weighted combination gives best result
    roll  = 0.98 * gyroRoll  + 0.02 * accRoll;
    pitch = 0.98 * gyroPitch + 0.02 * accPitch;

    // ---------------- OUTPUT ----------------
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" deg | Roll: ");
    Serial.println(roll);
  }
}
