/**
 * @file ToF.h
 * @brief Time-of-Flight distance sensor manager.
 *
 * Provides an abstraction for multiple VL53L1X Time-of-Flight sensors,
 * typically mounted on the left, center (front), and right sides of the robot.
 */
#pragma once

#include "VL53L1X.h"
#include <Wire.h>

/**
 * @brief Manages multiple VL53L1X Time-of-Flight distance sensors.
 */
class ToF {
public:
  /**
   * @brief Constructs a ToF sensor manager.
   */
  ToF();

  /**
    * @brief Initializes the front Time-of-Flight sensor.
   *
    * Configures I2C communication, resets the sensor through XSHUT,
    * initializes the VL53L1X, and starts continuous ranging.
   */
  void initSensors();

  /**
   * @brief Returns the distance measured by the front sensor.
   *
   * @return Distance value millimeters.
   */
  int getFrontDistance();

  /**
   * @brief Returns a median-filtered front distance.
   *
   * Keeps the latest five valid samples so one outlier does not immediately
   * change the distance used by navigation logic.
   *
   * @return Filtered distance in millimeters, or -1 if no valid sample exists.
   */
  int getFilteredFrontDistance();

  /**
   * @brief Reports whether sensor initialization succeeded.
   *
   * @return true when the sensor can be read, otherwise false.
   */
  bool isReady() const;

private:
  static constexpr uint8_t FILTER_WINDOW_SIZE = 5;
  VL53L1X center_sensor;  ///< Front-facing distance sensor
  bool ready = false;
  int distanceSamples[FILTER_WINDOW_SIZE] = {0};
  uint8_t distanceSampleCount = 0;
  uint8_t nextSampleIndex = 0;

};

