#include "ToF.h"
#include "config.h"
#include "wiring.h"
#include <algorithm>

ToF::ToF() {}

void ToF::initSensors() {
  // XSHUT is used to hold the sensor in reset while the I2C bus starts.
  pinMode(TOF_CENTER_XSHUT, OUTPUT);
  digitalWrite(TOF_CENTER_XSHUT, LOW);
  delay(10);
  digitalWrite(TOF_CENTER_XSHUT, HIGH);
  delay(10);

  // Start I2C on the pins assigned to the ESP32 in wiring.h.
  Wire.begin(I2C_SDA, I2C_SCL);

  // A timeout prevents a missing or disconnected sensor from blocking forever.
  center_sensor.setTimeout(TOF_READ_TIMEOUT_MS);
  ready = center_sensor.init();
  if (!ready) {
    return;
  }

  // This address is useful if more ToF sensors are added later.
  center_sensor.setAddress(ADDR_CENTER);
  center_sensor.startContinuous(50);
}

int ToF::getFrontDistance() {
  // -1 tells the caller that the sensor is unavailable or timed out.
  lastReadTimedOut = false;
  if (!ready) {
    return -1;
  }

  lastRawDistance = center_sensor.read();
  lastReadTimedOut = center_sensor.timeoutOccurred();
  if (lastReadTimedOut) {
    return -1;
  }
  lastRangeStatus = center_sensor.ranging_data.range_status;

  // Ignore measurements the sensor itself marks as unreliable. If a target is
  // clipped by the minimum range, report it as very close for obstacle safety.
  if (lastRangeStatus == VL53L1X::RangeValidMinRangeClipped) {
    return 1;
  }
  if (lastRangeStatus != VL53L1X::RangeValid &&
      lastRangeStatus != VL53L1X::RangeValidNoWrapCheckFail) {
    return -1;
  }

  // Apply the linear calibration configured in config.h.
  return static_cast<int>(lastRawDistance * TOF_CENTER_SLOPE + TOF_CENTER_OFFSET);
}

int ToF::getFilteredFrontDistance() {
  const int currentDistance = getFrontDistance();
  // Reject impossible readings before they enter the rolling window.
  if (currentDistance <= 0 || currentDistance > TOF_MAX_VALID_DISTANCE_MM) {
    if (consecutiveInvalidReads < TOF_MAX_CONSECUTIVE_INVALID_READS) {
      ++consecutiveInvalidReads;
      return lastFilteredDistance;
    }
    return -1;
  }

  consecutiveInvalidReads = 0;
  distanceSamples[nextSampleIndex] = currentDistance;
  nextSampleIndex = (nextSampleIndex + 1) % FILTER_WINDOW_SIZE;
  if (distanceSampleCount < FILTER_WINDOW_SIZE) {
    ++distanceSampleCount;
  }

  if (distanceSampleCount == 0) {
    return -1;
  }

  int orderedSamples[FILTER_WINDOW_SIZE];
  for (uint8_t i = 0; i < distanceSampleCount; ++i) {
    orderedSamples[i] = distanceSamples[i];
  }
  std::sort(orderedSamples, orderedSamples + distanceSampleCount);
  lastFilteredDistance = orderedSamples[distanceSampleCount / 2];
  return lastFilteredDistance;
}

bool ToF::isReady() const {
  return ready;
}

uint16_t ToF::getLastRawDistance() const {
  return lastRawDistance;
}

VL53L1X::RangeStatus ToF::getLastRangeStatus() const {
  return lastRangeStatus;
}

bool ToF::didLastReadTimeout() const {
  return lastReadTimedOut;
}