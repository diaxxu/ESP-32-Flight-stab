#include <Arduino.h>
#include <Wire.h>
#include "MPU6500.h"
#include "PID.h"

#define SERVO1_PIN 18
#define SERVO2_PIN 19
#define RC_CH1 34
#define RC_CH2 35

MPU6500 imu;
PID pid_pitch(1.0, 0.0, 0.0); // tune this

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.begin();
  pinMode(RC_CH1, INPUT);
  pinMode(RC_CH2, INPUT);
}

void loop() {
  imu.update();
  float pitch = imu.getPitch();
  int ch1_pwm = pulseIn(RC_CH1, HIGH, 25000);
  int ch2_pwm = pulseIn(RC_CH2, HIGH, 25000);

  float correction = pid_pitch.compute(0, pitch); // target is level (0)
  int servo1_output = ch1_pwm + correction;
  int servo2_output = ch2_pwm + correction;

  ledcWrite(0, servo1_output);
  ledcWrite(1, servo2_output);

  delay(20); // 50Hz servo refresh
}
