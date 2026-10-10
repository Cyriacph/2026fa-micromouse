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
constexpr float WHEEL_DIAMETER = 3.04f;   // cm
constexpr int SPR = 33; // steps per revolution: SPR = E * Nw/Nm (not accurate)

/* ---------- motion constants ---------- */
constexpr int BASE_PWM = 200; // initial base PWM; 200 moved the motors in healthcheck
constexpr int TURN_PWM = 170; // initial turn PWM; tune on the bench
constexpr int MIN_PWM = 0;
constexpr int MAX_PWM = 255;

// Flip these values if motor wiring or encoder polarity is reversed
constexpr int MOTOR_LEFT_DIRECTION = 1;
constexpr int MOTOR_RIGHT_DIRECTION = -1;
constexpr int ENCODER_LEFT_DIRECTION = 1;
constexpr int ENCODER_RIGHT_DIRECTION = -1;

/* ---------- straight-line PID ---------- */
constexpr float PID_KP = 5.0f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.0f;
constexpr float PID_INTEGRAL_LIMIT = 1000.0f;
constexpr unsigned long PID_PERIOD_MS = 10;
constexpr bool PID_DRIVE_ENABLED = true; // set to false to disable PID during bench testing

/* ---------- obstacle avoidance ---------- */
constexpr int OBSTACLE_DISTANCE_MM = 100;
constexpr int OBSTACLE_CLEAR_DISTANCE_MM = 130;
constexpr unsigned long SENSOR_PERIOD_MS = 50;
constexpr unsigned long BRAKE_TIME_MS = 100;
constexpr int BACK_CORRECTION_PWM = 100;
constexpr unsigned long BACK_CORRECTION_MS = 80;
constexpr unsigned long TURN_TIME_MS = 600;
constexpr int TOF_MAX_VALID_DISTANCE_MM = 4000;
// Keep using the last valid median through brief sensor timeouts (~1 second).
constexpr uint8_t TOF_MAX_CONSECUTIVE_INVALID_READS = 20;

/* ---------- serial motor test ---------- */
constexpr int MOTOR_TEST_PWM = 200; // match the known-working healthcheck test
constexpr unsigned long MOTOR_TEST_DURATION_MS = 500;

/* ---------- motor PWM ---------- */
constexpr int CH_L_FWD = 0;
constexpr int CH_L_REV = 1;
constexpr int CH_R_FWD = 2;
constexpr int CH_R_REV = 3;
constexpr int PWM_FREQUENCY = 20000;
constexpr int PWM_RESOLUTION = 8;

// ---------- ToF sensor offsets (mm) ----------
// linear regression on sensors to calbrate them
constexpr float TOF_CENTER_OFFSET = 0.0f;
constexpr float TOF_CENTER_SLOPE = 1.0f;


/* ---------- misc ---------- */
const int LED_BUILTIN = 2;
