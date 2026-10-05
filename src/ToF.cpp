#include "ToF.h"
#include "config.h"
#include "wiring.h"

ToF::ToF() {}

void ToF::initSensors() {
  pinMode(TOF_CENTER_XSHUT, OUTPUT);
  digitalWrite(TOF_CENTER_XSHUT, HIGH);
  Wire.begin(I2C_SDA, I2C_SCL);

  center_sensor.setTimeout(50);
  ready = center_sensor.init();
  if (ready) {
    center_sensor.setAddress(ADDR_CENTER);
    center_sensor.startContinuous(50);
  }
}

int ToF::getFrontDistance() {
  if (!ready) return -1;
  const int rawDistance = center_sensor.read();
  if (center_sensor.timeoutOccurred()) return -1;
  return static_cast<int>(rawDistance * TOF_CENTER_SLOPE + TOF_CENTER_OFFSET);
}

bool ToF::isReady() const {
  return ready;
}

void ToF::initCenterSensor() {
  initSensors();
}