#include "pid.h"

StraightPid::StraightPid(float kpValue, float kiValue, float kdValue,
                         float integralLimitValue)
    : kp(kpValue), ki(kiValue), kd(kdValue),
      integralLimit(integralLimitValue) {}

void StraightPid::reset() {
  integral = 0.0f;
  previousError = 0.0f;
  hasPreviousError = false;
}

float StraightPid::update(long leftTicks, long rightTicks, float dtSeconds) {
  const float error = static_cast<float>(rightTicks - leftTicks);
  integral += error * dtSeconds;
  integral = constrain(integral, -integralLimit, integralLimit);

  float derivative = 0.0f;
  if (hasPreviousError && dtSeconds > 0.0f) {
    derivative = (error - previousError) / dtSeconds;
  }
  previousError = error;
  hasPreviousError = true;
  return kp * error + ki * integral + kd * derivative;
}