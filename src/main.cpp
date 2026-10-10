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
bool motorTestActive = false;
unsigned long motorTestStarted = 0;
bool pidTestActive = false;
unsigned long pidTestStarted = 0;

void startMotorTest(int leftSpeed, int rightSpeed) {
  motorTestActive = true;
  motorTestStarted = millis();
  motors.setBoth(leftSpeed, rightSpeed);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = Serial.read();
    if (command == 'l') {
      startMotorTest(MOTOR_TEST_PWM, 0);
      Serial.println("Testing left motor");
    } else if (command == 'r') {
      startMotorTest(0, MOTOR_TEST_PWM);
      Serial.println("Testing right motor");
    } else if (command == 'b') {
      startMotorTest(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
      Serial.println("Testing both motors");
    } else if (command == 'p') {
      resetEncoders();
      previousLeftTicks = 0;
      previousRightTicks = 0;
      pidTestActive = true;
      pidTestStarted = millis();
      lastPidUpdate = pidTestStarted;
      Serial.println("PID test: driving straight, auto-stops");
    } else if (command == 's') {
      motorTestActive = false;
      pidTestActive = false;     // ← add this line
      motors.stop();
      Serial.println("Motors stopped");
    }
  }
}

void updateStraightDrive(bool drive) {
  const long leftTicks = getEncoderLeft();
  const long rightTicks = getEncoderRight();
  const long leftDelta = leftTicks - previousLeftTicks;
  const long rightDelta = rightTicks - previousRightTicks;
  const float correction = straightPid.update(
      leftDelta, rightDelta, PID_PERIOD_MS / 1000.0f);
  previousLeftTicks = leftTicks;
  previousRightTicks = rightTicks;

  if (!drive) return;

  const int leftSpeed = constrain(
      static_cast<int>(BASE_PWM + correction), MIN_PWM, MAX_PWM);
  const int rightSpeed = constrain(
      static_cast<int>(BASE_PWM - correction), MIN_PWM, MAX_PWM);
  motors.setBoth(leftSpeed, rightSpeed);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Let the encoder module configure its pins and interrupt handlers.
  initEncoders();
  resetEncoders();

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
  Serial.println("Motor test commands: l=left, r=right, b=both, s=stop");
}

void loop() {
  const unsigned long now = millis();
  handleSerialCommands();

  if (motorTestActive && now - motorTestStarted >= MOTOR_TEST_DURATION_MS) {
    motorTestActive = false;
    motors.stop();
    Serial.println("Motor test complete");
  }

  if (pidTestActive && now - pidTestStarted >= MOTOR_TEST_DURATION_MS) {
    pidTestActive = false;
    motors.stop();
    Serial.printf("PID test complete: encLeft=%ld encRight=%ld (diff=%ld)\n",
                  getEncoderLeft(), getEncoderRight(),
                  getEncoderLeft() - getEncoderRight());
  }

  if (pidTestActive && now - lastPidUpdate >= PID_PERIOD_MS) {
    lastPidUpdate = now;
    updateStraightDrive(true);
  }

  static unsigned long lastDiagnostic = 0;
  if (now - lastDiagnostic >= 200) {
    lastDiagnostic = now;
    Serial.printf("encLeft=%ld  encRight=%ld\n",
                  getEncoderLeft(), getEncoderRight());
    Serial.printf("front distance=%d mm\n",
                  tof.getFilteredFrontDistance());
  }
}