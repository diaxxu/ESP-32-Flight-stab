#include <Wire.h>
#include <ESP32Servo.h>

// MPU6050 Registers
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// Pins (change to your wiring)
const int servoPinAileron = 18;
const int servoPinElevator = 19;

const int rcInputPinPitch = 34;  // RC PWM input (elevator)
const int rcInputPinRoll  = 35;  // RC PWM input (aileron)

// PID parameters (tune these)
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

// Complementary filter constant (0 < alpha < 1)
const float alpha = 0.96;

Servo servoAileron;
Servo servoElevator;

// Variables for MPU data
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

float pitch = 0, roll = 0;         // filtered angles
float pitchRaw, rollRaw;           // from accel only

// PID control variables
float pitchError, rollError;
float pitchIntegral = 0, rollIntegral = 0;
float pitchPrevError = 0, rollPrevError = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  initMPU();

  servoAileron.attach(servoPinAileron);
  servoElevator.attach(servoPinElevator);

  pinMode(rcInputPinPitch, INPUT);
  pinMode(rcInputPinRoll, INPUT);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  readMPU();
  complementaryFilter(dt);
  
  // Read RC PWM inputs (pulse width in microseconds)
  int rcPitchPulse = pulseIn(rcInputPinPitch, HIGH, 25000);  // typical RC pulse ~1000-2000us
  int rcRollPulse = pulseIn(rcInputPinRoll, HIGH, 25000);

  // Map RC input from 1000-2000us to -30 to +30 degrees desired angle range (adjust as needed)
  float desiredPitch = map(rcPitchPulse, 1000, 2000, -30, 30);
  float desiredRoll = map(rcRollPulse, 1000, 2000, -30, 30);

  // PID for pitch
  pitchError = desiredPitch - pitch;
  pitchIntegral += pitchError * dt;
  float pitchDerivative = (pitchError - pitchPrevError) / dt;
  float pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  pitchPrevError = pitchError;

  // PID for roll
  rollError = desiredRoll - roll;
  rollIntegral += rollError * dt;
  float rollDerivative = (rollError - rollPrevError) / dt;
  float rollOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
  rollPrevError = rollError;

  // Convert PID output to servo angles (90 = neutral, 0-180 range)
  int servoPitchAngle = constrain(90 + pitchOutput, 0, 180);
  int servoRollAngle = constrain(90 + rollOutput, 0, 180);

  servoElevator.write(servoPitchAngle);
  servoAileron.write(servoRollAngle);

  // Debugging
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Desired: "); Serial.print(desiredPitch);
  Serial.print(" ServoPitch: "); Serial.print(servoPitchAngle);
  Serial.print(" | Roll: "); Serial.print(roll);
  Serial.print(" Desired: "); Serial.print(desiredRoll);
  Serial.print(" ServoRoll: "); Serial.println(servoRollAngle);

  delay(20);  // ~50Hz control loop
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void complementaryFilter(float dt) {
  // Convert raw accel to angles (in degrees)
  pitchRaw = atan2(accelY, accelZ) * 180 / PI;
  rollRaw = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  // Gyro rates in degrees/sec (assuming sensitivity scale = 131 LSB/(deg/s))
  float gyroPitchRate = gyroX / 131.0;
  float gyroRollRate = gyroY / 131.0;

  // Complementary filter to combine accel and gyro
  pitch = alpha * (pitch + gyroPitchRate * dt) + (1 - alpha) * pitchRaw;
  roll = alpha * (roll + gyroRollRate * dt) + (1 - alpha) * rollRaw;
}
