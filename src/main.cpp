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
float smoothedCorrection = 0.0f;
long previousLeftTicks = 0;
long previousRightTicks = 0;
unsigned long lastPidUpdate = 0;
int frontDistanceMm = -1;
unsigned long lastSensorUpdate = 0;
unsigned long lastDiagnosticUpdate = 0;

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
int turnDirection = 1;

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

void startTurn() {
  changeMotionState(MotionState::TURN);
}

void stopAllMotion() {
  autonomousActive = false;
  motors.stop();
  changeMotionState(MotionState::IDLE);
}

void startAutonomousMode() {
  resetEncoders();
  previousLeftTicks = 0;
  previousRightTicks = 0;
  straightPid.reset();
  smoothedCorrection = 0.0f;
  autonomousActive = true;
  changeMotionState(MotionState::DRIVE);
  Serial.println("Autonomous mode started");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = Serial.read();
    if (command == 'a') {
      startAutonomousMode();
    } else if (command == 's') {
      stopAllMotion();
      Serial.println("Motors stopped");
    }
  }
}

void updateStraightDrive(int basePwm = BASE_PWM) {
  const long leftTicks = getEncoderLeft();
  const long rightTicks = getEncoderRight();
  const long leftDelta = leftTicks - previousLeftTicks;
  const long rightDelta = rightTicks - previousRightTicks;
  const float rawCorrection = constrain(
      straightPid.update(leftDelta, rightDelta, PID_PERIOD_MS / 1000.0f),
      -PID_MAX_CORRECTION_PWM, PID_MAX_CORRECTION_PWM);
  smoothedCorrection += PID_CORRECTION_SMOOTHING *
                        (rawCorrection - smoothedCorrection);
  const float correction = smoothedCorrection;
  previousLeftTicks = leftTicks;
  previousRightTicks = rightTicks;

  const int leftSpeed = constrain(
      static_cast<int>(basePwm + correction), MIN_PWM, MAX_PWM);
  const int rightSpeed = constrain(
      static_cast<int>(basePwm - correction), MIN_PWM, MAX_PWM);
  motors.setBoth(leftSpeed, rightSpeed);
}

void updateAutonomousMode() {
  if (!autonomousActive) {
    return;
  }

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
        updateStraightDrive();
      }
      break;

    case MotionState::OBSTACLE_DETECTED:
      motors.brake();
      previousLeftTicks = getEncoderLeft();
      previousRightTicks = getEncoderRight();
      straightPid.reset();
      smoothedCorrection = 0.0f;
      changeMotionState(MotionState::BRAKE_APPROACH);
      break;

    case MotionState::BRAKE_APPROACH:
      if (frontDistanceMm < 0) {
        motors.brake();
      } else if (frontDistanceMm <= TARGET_STOP_DISTANCE_MM) {
        motors.brake();
        startTurn();
      } else if (elapsed < BRAKE_TIME_MS) {
        motors.brake();
      } else {
        // Creep forward under PID until the filtered sensor reaches the target.
        updateStraightDrive(APPROACH_PWM);
      }
      break;

    case MotionState::TURN:
      if (elapsed < TURN_TIME_MS) {
        motors.setBoth(turnDirection * TURN_PWM,
                       -turnDirection * TURN_PWM);
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
        smoothedCorrection = 0.0f;
        changeMotionState(MotionState::DRIVE);
      } else if (frontDistanceMm >= 0) {
        // Continue pivoting in the same direction until the sensor sees clear.
        startTurn();
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

  // Initialize the front distance sensor before autonomous mode starts.
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

  // Keep the motors stopped until hardware initialization is complete.
  motors.stop();
  lastPidUpdate = millis();
  lastSensorUpdate = millis();
  Serial.println("Hardware initialization complete");
  Serial.println("Autonomous mode starts automatically in 2 seconds; send 's' to stop");

  // Give the operator time to set the mouse down before it starts driving.
  delay(2000);
  startAutonomousMode();
}

void loop() {
  const unsigned long now = millis();
  handleSerialCommands();

  if (now - lastSensorUpdate >= SENSOR_PERIOD_MS) {
    lastSensorUpdate = now;
    frontDistanceMm = tof.getFilteredFrontDistance();
  }

  if (autonomousActive && now - lastPidUpdate >= PID_PERIOD_MS) {
    lastPidUpdate = now;
    updateAutonomousMode();
  }

  // Diagnostic heartbeat: prints only, changes no behavior.
  if (now - lastDiagnosticUpdate >= 200) {
    lastDiagnosticUpdate = now;
    Serial.printf("[%s] dist=%d encL=%ld encR=%ld\n",
                  stateName(motionState), frontDistanceMm,
                  getEncoderLeft(), getEncoderRight());
  }
}