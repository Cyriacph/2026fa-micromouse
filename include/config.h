/**
 * @file config.h
 * @brief Robot geometry, motion, and maze configuration constants.
 *
 * Defines compile-time constants related to:
 *  - Physical robot geometry and kinematics
 *  - Motion control parameters (PWM limits and defaults)
 *  - Maze dimensions and start/goal configuration
 *  - Sensor thresholds and calibration values
 *  - Miscellaneous hardware-related constants
 *
 * These values are hardware- and build-specific and should be adjusted
 * to match the physical robot and competition setup.
 */
#pragma once
#include <Arduino.h>

/* ---------- geometry (measure yours!) ---------- */
constexpr float WHEEL_DIAMETER = 0;   // cm
constexpr int SPR = 0; // steps per revolution

/* ---------- motion constants ---------- */
constexpr int BASE_PWM = 150; // initial base PWM; tune on the bench
constexpr int TURN_PWM = 170; // initial turn PWM; tune on the bench
constexpr int MIN_PWM = 0;
constexpr int MAX_PWM = 255;

// Flip these values if motor wiring or encoder polarity is reversed.
constexpr int MOTOR_LEFT_DIRECTION = 1;
constexpr int MOTOR_RIGHT_DIRECTION = 1;
constexpr int ENCODER_LEFT_DIRECTION = 1;
constexpr int ENCODER_RIGHT_DIRECTION = 1;

/* ---------- motor PWM ---------- */
constexpr int CH_L_FWD = 0;
constexpr int CH_L_REV = 1;
constexpr int CH_R_FWD = 2;
constexpr int CH_R_REV = 3;
constexpr int PWM_FREQUENCY = 20000;
constexpr int PWM_RESOLUTION = 8;

// ---------- ToF sensor offsets (mm) ----------
// linear regression on sensors to calbrate them 
// CALIBRATE YOUR OWN!
constexpr float TOF_CENTER_OFFSET = 0.0f;
constexpr float TOF_CENTER_SLOPE = 1.0f;


/* ---------- misc ---------- */
const int LED_BUILTIN = 2;
