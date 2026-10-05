#include "motors.h"
#include "config.h"

namespace {
// Saved hardware mapping used by all later motor commands.
MotorConfig motorConfig;
bool initialized = false;

// Keep signed motor commands inside the PWM range accepted by the driver.
int clampSpeed(int speed) {
  return constrain(speed, -motorConfig.max_pwm, motorConfig.max_pwm);
}

// Convert one signed speed into the two PWM signals required by the MX1508.
// Positive speed drives channel 1, negative speed drives channel 2, and zero
// drives neither channel so the motor coasts.
void writeMotor(const MotorPins& pins, int speed) {
  speed = clampSpeed(speed);
  if (speed > 0) {
    ledcWrite(pins.ch1, speed);
    ledcWrite(pins.ch2, 0);
  } else if (speed < 0) {
    ledcWrite(pins.ch1, 0);
    ledcWrite(pins.ch2, -speed);
  } else {
    ledcWrite(pins.ch1, 0);
    ledcWrite(pins.ch2, 0);
  }
}
}

bool Motors::begin(const MotorConfig& cfg) {
  // Configure one PWM channel for each direction of each motor.
  motorConfig = cfg;
  ledcSetup(cfg.left.ch1, cfg.pwm_freq, cfg.pwm_res);
  ledcSetup(cfg.left.ch2, cfg.pwm_freq, cfg.pwm_res);
  ledcSetup(cfg.right.ch1, cfg.pwm_freq, cfg.pwm_res);
  ledcSetup(cfg.right.ch2, cfg.pwm_freq, cfg.pwm_res);
  ledcAttachPin(cfg.left.in1, cfg.left.ch1);
  ledcAttachPin(cfg.left.in2, cfg.left.ch2);
  ledcAttachPin(cfg.right.in1, cfg.right.ch1);
  ledcAttachPin(cfg.right.in2, cfg.right.ch2);

  // Do not accept movement commands until the PWM hardware is ready.
  initialized = true;
  stop();
  return true;
}

void Motors::setLeft(int speed) {
  // The direction multiplier compensates for how the motor is mounted or wired.
  if (initialized) writeMotor(motorConfig.left, MOTOR_LEFT_DIRECTION * speed);
}

void Motors::setRight(int speed) {
  if (initialized) writeMotor(motorConfig.right, MOTOR_RIGHT_DIRECTION * speed);
}

void Motors::setBoth(int leftSpeed, int rightSpeed) {
  setLeft(leftSpeed);
  setRight(rightSpeed);
}

void Motors::stop() {
  // Zero on both channels means coast, not active braking.
  if (initialized) setBoth(0, 0);
}

void Motors::brake() {
  if (!initialized) return;

  // Driving both inputs high shorts the motor terminals through the MX1508,
  // producing active braking through the motor's back EMF.
  ledcWrite(motorConfig.left.ch1, motorConfig.max_pwm);
  ledcWrite(motorConfig.left.ch2, motorConfig.max_pwm);
  ledcWrite(motorConfig.right.ch1, motorConfig.max_pwm);
  ledcWrite(motorConfig.right.ch2, motorConfig.max_pwm);
}