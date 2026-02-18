#include <Wire.h>        // I2C communication library for talking to the MPU6050
#include <MPU6050.h>     // MPU6050 IMU sensor library
#include <Servo.h>       // Servo motor control library
#include <PID_v1.h>      // PID controller library

MPU6050 mpu;             // Create MPU6050 sensor object
Servo pitchServo;        // Servo object for pitch control, wired to pin D9
Servo rollServo;         // Servo object for roll control, wired to pin D10

double pitchSetpoint = 0.0;  // Target pitch angle (0 = level)
double rollSetpoint  = 0.0;  // Target roll angle (0 = level)

double pitchInput, pitchOutput;  // PID input (measured angle) and output (correction) for pitch
double rollInput,  rollOutput;   // PID input (measured angle) and output (correction) for roll

double pitchKp = 1.8, pitchKi = 0.00, pitchKd = 1.2;  // PID tuning gains for pitch axis
double rollKp  = 1.2, rollKi  = 0.00, rollKd  = 1;    // PID tuning gains for roll axis

// Create PID controllers for pitch and roll, linking their input/output/setpoint variables
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID (&rollInput,  &rollOutput,  &rollSetpoint,  rollKp,  rollKi,  rollKd,  DIRECT);

float pitchAngle = 0.0, rollAngle = 0.0;  // Current estimated pitch and roll angles in degrees
float alpha = 0.96;                        // Complementary filter weight (favors gyro over accelerometer)
unsigned long prevTime;                    // Timestamp of the last loop iteration in milliseconds

void setup() {
  Serial.begin(115200);   // Start serial communication at 115200 baud for debugging/logging
  Wire.begin();           // Initialize I2C bus
  mpu.initialize();       // Power on and configure the MPU6050 with default settings

  if (!mpu.testConnection()) {         // Check if the MPU6050 responds over I2C
    Serial.println("MPU6050 connection failed!");  // Print error if sensor not found
    while (1);                         // Halt execution indefinitely if sensor is missing
  }
  Serial.println("MPU6050 connected.");  // Confirm successful sensor connection

  pitchServo.attach(9);   // Attach pitch servo signal wire to Arduino pin 9
  rollServo.attach(10);   // Attach roll servo signal wire to Arduino pin 10
  pitchServo.write(90);   // Move pitch servo to neutral center position (90°)
  rollServo.write(90);    // Move roll servo to neutral center position (90°)

  delay(2000);            // Wait 2 seconds for servos to reach center and system to settle

  int16_t ax, ay, az, gx, gy, gz;                      // Raw sensor reading variables (16-bit integers)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);        // Read initial accelerometer and gyroscope values
  pitchAngle = atan2((float)ay, (float)az) * 180.0 / PI;  // Calculate initial pitch angle from accelerometer (degrees)
  rollAngle  = atan2((float)ax, (float)az) * 180.0 / PI;  // Calculate initial roll angle from accelerometer (degrees)

  pitchPID.SetMode(AUTOMATIC);         // Enable the pitch PID controller (start computing outputs)
  rollPID.SetMode(AUTOMATIC);          // Enable the roll PID controller
  pitchPID.SetOutputLimits(-45, 45);   // Clamp pitch PID output to ±45° servo correction range
  rollPID.SetOutputLimits(-30, 30);    // Clamp roll PID output to ±30° servo correction range

  prevTime = millis();                 // Record the current time as the starting reference point
  Serial.println("System Ready.");     // Signal that setup is complete and the loop is about to begin
}

void loop() {
  unsigned long now = millis();        // Get the current time in milliseconds
  if (now - prevTime < 10) return;     // Skip this iteration if less than 10ms has passed (100Hz max loop rate)
  float dt = (now - prevTime) / 1000.0;  // Calculate elapsed time in seconds since last iteration
  prevTime = now;                      // Update the previous timestamp to current time

  int16_t ax, ay, az, gx, gy, gz;              // Declare variables to hold raw sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Read accelerometer (ax,ay,az) and gyroscope (gx,gy,gz) values

  float accPitch = atan2((float)ay, (float)az) * 180.0 / PI;  // Compute pitch angle from accelerometer using arctan (degrees)
  float accRoll  = atan2((float)ax, (float)az) * 180.0 / PI;  // Compute roll angle from accelerometer using arctan (degrees)

  float gyroX = (float)gx / 131.0;   // Convert raw gyro X reading to degrees/sec (131 LSB per °/s at ±250°/s range)
  float gyroY = (float)gy / 131.0;   // Convert raw gyro Y reading to degrees/sec

  // Complementary filter: blend gyro integration (fast, drifts) with accelerometer (slow, noisy)
  pitchAngle = alpha * (pitchAngle + gyroX * dt) + (1 - alpha) * accPitch;  // Update pitch estimate (96% gyro, 4% accel)
  rollAngle  = alpha * (rollAngle  + gyroY * dt) + (1 - alpha) * accRoll;   // Update roll estimate (96% gyro, 4% accel)

  pitchInput = pitchAngle;   // Feed the current pitch angle into the pitch PID as its measured input
  rollInput  = rollAngle;    // Feed the current roll angle into the roll PID as its measured input

  pitchPID.Compute();        // Run the pitch PID algorithm to calculate the required correction output
  rollPID.Compute();         // Run the roll PID algorithm to calculate the required correction output

  // Convert PID output to servo position: center (90°) plus correction, then clamp to safe range
  int pitchCmd = constrain(90 + (int)pitchOutput, 45, 135);  // Pitch servo command: 90° center ± correction, limited to 45–135°
  int rollCmd  = constrain(90 - (int)rollOutput,  60, 120);  // Roll servo command: subtracted to correct direction, limited to 60–120°

  pitchServo.write(pitchCmd);  // Send the computed angle command to the pitch servo
  rollServo.write(rollCmd);    // Send the computed angle command to the roll servo

  // Log data to Serial in CSV format for monitoring or plotting
  Serial.print(now);           Serial.print(",");  // Print current timestamp in milliseconds
  Serial.print(pitchAngle);    Serial.print(",");  // Print estimated pitch angle in degrees
  Serial.print(rollAngle);     Serial.print(",");  // Print estimated roll angle in degrees
  Serial.print(pitchCmd);      Serial.print(",");  // Print the pitch servo command sent (45–135)
  Serial.println(rollCmd);                          // Print the roll servo command sent (60–120) and end the line
}
