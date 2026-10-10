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
int frontDistanceMm = -1;
unsigned long lastSensorUpdate = 0;

enum class MotionState {
  IDLE,
  DRIVE,
  OBSTACLE_DETECTED,
  BRAKE_APPROACH,
  TURN,
  WAIT_CLEAR
};
MotionState motionState = MotionState::IDLE;
unsigned long stateStartedAt = 0;
bool autonomousActive = false;

const char* stateName(MotionState state) {
  switch (state) {
    case MotionState::IDLE: return "IDLE";
    case MotionState::DRIVE: return "DRIVE";
    case MotionState::OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
    case MotionState::BRAKE_APPROACH: return "BRAKE_APPROACH";
    case MotionState::TURN: return "TURN";
    case MotionState::WAIT_CLEAR: return "WAIT_CLEAR";
  }
  return "UNKNOWN";
}

void changeMotionState(MotionState next) {
  motionState = next;
  stateStartedAt = millis();
  Serial.printf("State: %s\n", stateName(motionState));
}

void stopAllMotion() {
  motorTestActive = false;
  pidTestActive = false;
  autonomousActive = false;
  motors.stop();
  changeMotionState(MotionState::IDLE);
}

void startAutonomousMode() {
  resetEncoders();
  previousLeftTicks = 0;
  previousRightTicks = 0;
  straightPid.reset();
  autonomousActive = true;
  changeMotionState(MotionState::DRIVE);
  Serial.println("Autonomous mode started");
}

void startMotorTest(int leftSpeed, int rightSpeed) {
  motorTestActive = true;
  motorTestStarted = millis();
  motors.setBoth(leftSpeed, rightSpeed);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = Serial.read();
    if (command == 'l') {
      autonomousActive = false;
      startMotorTest(MOTOR_TEST_PWM, 0);
      Serial.println("Testing left motor");
    } else if (command == 'r') {
      autonomousActive = false;
      startMotorTest(0, MOTOR_TEST_PWM);
      Serial.println("Testing right motor");
    } else if (command == 'b') {
      autonomousActive = false;
      startMotorTest(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
      Serial.println("Testing both motors");
    } else if (command == 'p') {
      autonomousActive = false;
      motorTestActive = false;
      resetEncoders();
      previousLeftTicks = 0;
      previousRightTicks = 0;
      pidTestActive = true;
      pidTestStarted = millis();
      lastPidUpdate = pidTestStarted;
      Serial.println("PID test: driving straight, auto-stops");
    } else if (command == 'a') {
      motorTestActive = false;
      pidTestActive = false;
      startAutonomousMode();
    } else if (command == 's') {
      stopAllMotion();
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

void updateAutonomousMode() {
  if (!autonomousActive) return;

  const unsigned long elapsed = millis() - stateStartedAt;
  switch (motionState) {
    case MotionState::IDLE:
      motors.stop();
      break;

    case MotionState::DRIVE:
      if (frontDistanceMm < 0) {
        // Do not drive without a current valid distance measurement.
        motors.stop();
      } else if (frontDistanceMm <= OBSTACLE_DISTANCE_MM) {
        changeMotionState(MotionState::OBSTACLE_DETECTED);
      } else {
        updateStraightDrive(true);
      }
      break;

    case MotionState::OBSTACLE_DETECTED:
      motors.brake();
      changeMotionState(MotionState::BRAKE_APPROACH);
      break;

    case MotionState::BRAKE_APPROACH:
      if (elapsed < BRAKE_TIME_MS) {
        motors.brake();
      } else if (elapsed < BRAKE_TIME_MS + BACK_CORRECTION_MS) {
        motors.setBoth(-BACK_CORRECTION_PWM, -BACK_CORRECTION_PWM);
      } else {
        motors.stop();
        changeMotionState(MotionState::TURN);
      }
      break;

    case MotionState::TURN:
      if (elapsed < TURN_TIME_MS) {
        motors.setBoth(TURN_PWM, -TURN_PWM);
      } else {
        motors.stop();
        changeMotionState(MotionState::WAIT_CLEAR);
      }
      break;

    case MotionState::WAIT_CLEAR:
      motors.stop();
      if (frontDistanceMm >= OBSTACLE_CLEAR_DISTANCE_MM) {
        resetEncoders();
        previousLeftTicks = 0;
        previousRightTicks = 0;
        straightPid.reset();
        changeMotionState(MotionState::DRIVE);
      } else if (frontDistanceMm >= 0 &&
                 frontDistanceMm <= OBSTACLE_DISTANCE_MM) {
        changeMotionState(MotionState::OBSTACLE_DETECTED);
      }
      break;
  }
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
  lastSensorUpdate = millis();
  Serial.println("Hardware initialization complete; motors stopped");
  Serial.println("Commands: l=left, r=right, b=both, p=PID, a=auto, s=stop");
}

void loop() {
  const unsigned long now = millis();
  handleSerialCommands();

  if (now - lastSensorUpdate >= SENSOR_PERIOD_MS) {
    lastSensorUpdate = now;
    frontDistanceMm = tof.getFilteredFrontDistance();
  }

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

  if (autonomousActive && now - lastPidUpdate >= PID_PERIOD_MS) {
    lastPidUpdate = now;
    updateAutonomousMode();
  }

  static unsigned long lastDiagnostic = 0;
  if (now - lastDiagnostic >= 200) {
    lastDiagnostic = now;
    Serial.printf("encLeft=%ld  encRight=%ld\n",
                  getEncoderLeft(), getEncoderRight());
    Serial.printf("front distance=%d mm\n", frontDistanceMm);
  }
}