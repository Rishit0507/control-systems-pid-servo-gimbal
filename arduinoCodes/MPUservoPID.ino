#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <PID_v1.h>

MPU6050 mpu;
Servo pitchServo;  // D9
Servo rollServo;   // D10

double pitchSetpoint = 0.0;
double rollSetpoint  = 0.0;

double pitchInput, pitchOutput;
double rollInput,  rollOutput;

double pitchKp = 1.8, pitchKi = 0.00, pitchKd = 1.2;
double rollKp  = 1.2, rollKi  = 0.00, rollKd  = 1;

PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID (&rollInput,  &rollOutput,  &rollSetpoint,  rollKp,  rollKi,  rollKd,  DIRECT);

float pitchAngle = 0.0, rollAngle = 0.0;
float alpha = 0.96;
unsigned long prevTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected.");

  pitchServo.attach(9);
  rollServo.attach(10);
  pitchServo.write(90);
  rollServo.write(90);

  delay(2000);

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pitchAngle = atan2((float)ay, (float)az) * 180.0 / PI;
  rollAngle  = atan2((float)ax, (float)az) * 180.0 / PI;

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-45, 45);
  rollPID.SetOutputLimits(-30, 30);

  prevTime = millis();
  Serial.println("System Ready.");
}

void loop() {
  unsigned long now = millis();
  if (now - prevTime < 10) return;
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accPitch = atan2((float)ay, (float)az) * 180.0 / PI;
  float accRoll  = atan2((float)ax, (float)az) * 180.0 / PI;

  float gyroX = (float)gx / 131.0;
  float gyroY = (float)gy / 131.0;

  pitchAngle = alpha * (pitchAngle + gyroX * dt) + (1 - alpha) * accPitch;
  rollAngle  = alpha * (rollAngle  + gyroY * dt) + (1 - alpha) * accRoll;

  pitchInput = pitchAngle;
  rollInput  = rollAngle;

  pitchPID.Compute();
  rollPID.Compute();

  int pitchCmd = constrain(90 + (int)pitchOutput, 45, 135);
  int rollCmd  = constrain(90 - (int)rollOutput,  60, 120);

  pitchServo.write(pitchCmd);
  rollServo.write(rollCmd);
// CSV format: time, pitch, roll, pitchCmd, rollCmd
Serial.print(now);        Serial.print(",");
Serial.print(pitchAngle); Serial.print(",");
Serial.print(rollAngle);  Serial.print(",");
Serial.print(pitchCmd);   Serial.print(",");
Serial.println(rollCmd);
}