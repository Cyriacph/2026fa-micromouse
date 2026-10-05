#pragma once

#include <Arduino.h>

class StraightPid {
public:
  StraightPid(float kp, float ki, float kd, float integralLimit);
  void reset();
  float update(long leftTicks, long rightTicks, float dtSeconds);

private:
  float kp;
  float ki;
  float kd;
  float integralLimit;
  float integral = 0.0f;
  float previousError = 0.0f;
  bool hasPreviousError = false;
};