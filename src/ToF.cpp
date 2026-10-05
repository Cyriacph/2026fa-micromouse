#include "ToF.h"
#include "config.h"
#include "wiring.h"

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
  center_sensor.setTimeout(50);
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
  if (!ready) return -1;

  const int rawDistance = center_sensor.read();
  if (center_sensor.timeoutOccurred()) return -1;

  // Apply the linear calibration configured in config.h.
  return static_cast<int>(rawDistance * TOF_CENTER_SLOPE + TOF_CENTER_OFFSET);
}

bool ToF::isReady() const {
  return ready;
}