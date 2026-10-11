#pragma once

#include <Arduino.h>
#include <Wire.h>

// Small I2C helpers.
static bool i2cWriteByte(TwoWire* bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus->beginTransmission(addr);
  bus->write(reg);
  bus->write(val);
  return bus->endTransmission() == 0;
}

static bool i2cReadBytes(TwoWire* bus, uint8_t addr, uint8_t reg, uint8_t* buf,
                         size_t n) {
  bus->beginTransmission(addr);
  bus->write(reg);
  if (bus->endTransmission(false) != 0) {
    return false;
  }

  const size_t received = bus->requestFrom(static_cast<int>(addr),
                                           static_cast<int>(n));
  if (received != n) {
    return false;
  }

  for (size_t i = 0; i < n; ++i) {
    buf[i] = bus->read();
  }
  return true;
}

static inline int16_t make16(uint8_t hi, uint8_t lo) {
  return static_cast<int16_t>((hi << 8) | lo);
}