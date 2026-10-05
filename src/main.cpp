#include <Arduino.h>

#include "config.h"
#include "wiring.h"
#include "motors.h"
#include "encoders.h"
#include "ToF.h"
#include "pid.h"

// Pin assignments
static const int LEFT_FWD  = MTR_A_IN1;  // MX1508 IN1
static const int LEFT_REV  = MTR_A_IN2;  // MX1508 IN2
static const int RIGHT_FWD = MTR_B_IN1;  // MX1508 IN3
static const int RIGHT_REV = MTR_B_IN2;  // MX1508 IN4

// High-level controller for both motors
Motors motors;
ToF tof;
StraightPid straightPid(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT);
long previousLeftTicks = 0;
long previousRightTicks = 0;
unsigned long lastPidUpdate = 0;

void updateStraightDrive() {
  const long leftTicks = getEncoderLeft();
  const long rightTicks = getEncoderRight();
  const long leftDelta = leftTicks - previousLeftTicks;
  const long rightDelta = rightTicks - previousRightTicks;
  const float correction = straightPid.update(
      leftDelta, rightDelta, PID_PERIOD_MS / 1000.0f);

  previousLeftTicks = leftTicks;
  previousRightTicks = rightTicks;

  const int leftSpeed = constrain(
      static_cast<int>(BASE_PWM + correction), MIN_PWM, MAX_PWM);
  const int rightSpeed = constrain(
      static_cast<int>(BASE_PWM - correction), MIN_PWM, MAX_PWM);

  if (PID_DRIVE_ENABLED) {
    motors.setBoth(leftSpeed, rightSpeed);
  } else {
    // Keep the robot stationary until the physical PID test is enabled.
    motors.stop();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Let the encoder module configure its pins and interrupt handlers.
  initEncoders();

  // Initialize the front distance sensor before the bench test starts.
  tof.initSensors();
  Serial.println(tof.isReady() ? "ToF sensor ready" : "ToF sensor failed");

  // Configure the motors with the correct pins and PWM settings
  MotorConfig motorConfig;
  motorConfig.left.in1 = LEFT_FWD;
  motorConfig.left.in2 = LEFT_REV;
  motorConfig.left.ch1 = CH_L_FWD;
  motorConfig.left.ch2 = CH_L_REV;
  motorConfig.right.in1 = RIGHT_FWD;
  motorConfig.right.in2 = RIGHT_REV;
  motorConfig.right.ch1 = CH_R_FWD;
  motorConfig.right.ch2 = CH_R_REV;
  motorConfig.pwm_freq = PWM_FREQUENCY;
  motorConfig.pwm_res = PWM_RESOLUTION;
  motorConfig.max_pwm = MAX_PWM;
  motors.begin(motorConfig);

  // The application starts in a safe state until motion logic is added.
  motors.stop();
  lastPidUpdate = millis();
  Serial.println("Hardware initialization complete; motors stopped");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastPidUpdate >= PID_PERIOD_MS) {
    lastPidUpdate = now;
    updateStraightDrive();
  }

  static unsigned long lastDiagnostic = 0;
  if (now - lastDiagnostic >= 200) {
    lastDiagnostic = now;
    Serial.printf("encLeft=%ld  encRight=%ld\n",
                  getEncoderLeft(), getEncoderRight());
    Serial.printf("front distance=%d mm\n", tof.getFrontDistance());
  }
}