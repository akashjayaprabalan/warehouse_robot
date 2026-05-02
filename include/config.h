/**
 * config.h
 * Hardware pin assignments and tunable constants for the ESP32 warehouse robot.
 *
 * Sensor assumptions (per spec):
 *   IR sensors     : 0 (dark/line) – 1027 (light/floor)  [ADC raw counts]
 *   Ultrasonic     : round-trip pulse time in microseconds (μs)
 *   Encoders       : interrupt-driven pulse count per revolution
 *
 * Motor control uses servo PWM pulse widths (μs):
 *   1500 μs = stopped, 1400 μs ≈ full reverse, 1600 μs ≈ full forward
 *   (continuous-rotation servo convention matching original Simulink model)
 */

#pragma once

#include <stdint.h>

// ─────────────────────────────────────────────
//  ESP32 pin assignments
// ─────────────────────────────────────────────

// Analog IR sensors (ADC1 channels, 12-bit: 0-4095, mapped to 0-1027 range)
constexpr uint8_t PIN_IR_RIGHT      = 34;   // GPIO34 / ADC1_CH6
constexpr uint8_t PIN_IR_LEFT       = 35;   // GPIO35 / ADC1_CH7

// Ultrasonic sensor (HC-SR04 or equivalent)
constexpr uint8_t PIN_ULTRASONIC_TRIG = 5;
constexpr uint8_t PIN_ULTRASONIC_ECHO = 18;

// Continuous-rotation servo motors (PWM)
constexpr uint8_t PIN_MOTOR_RIGHT   = 12;
constexpr uint8_t PIN_MOTOR_LEFT    = 13;

// Gripper servo (standard servo, angle-controlled)
constexpr uint8_t PIN_GRIPPER       = 14;

// Quadrature encoder inputs (interrupt-capable GPIO)
constexpr uint8_t PIN_ENC_RIGHT_A   = 25;
constexpr uint8_t PIN_ENC_LEFT_A    = 26;

// Status LED (on-board LED)
constexpr uint8_t PIN_LED           = 2;

// ─────────────────────────────────────────────
//  Motor PWM pulse widths (μs)
// ─────────────────────────────────────────────
constexpr int16_t MOTOR_STOP        = 1500;   // no movement
constexpr int16_t MOTOR_FWD_FULL    = 1600;   // full forward
constexpr int16_t MOTOR_REV_FULL    = 1400;   // full reverse
constexpr int16_t MOTOR_MIN_PULSE   = 1000;   // hardware absolute min (failsafe)
constexpr int16_t MOTOR_MAX_PULSE   = 2000;   // hardware absolute max (failsafe)

// ─────────────────────────────────────────────
//  Gripper servo angles (degrees, maps to 0–180 range)
// ─────────────────────────────────────────────
constexpr uint8_t GRIPPER_OPEN      = 180;   // open position (servo max; Simulink used 190 but servo range is 0-180)
constexpr uint8_t GRIPPER_CLOSE     = 115;   // closed / gripping

// ─────────────────────────────────────────────
//  IR sensor thresholds  (0 dark line, 1027 bright floor)
// ─────────────────────────────────────────────

// Below this value → sensor is over the dark line
constexpr int16_t IR_LINE_THRESHOLD     = 650;

// Above this value → sensor is fully on the light floor (used for start-line detection)
constexpr int16_t IR_FLOOR_THRESHOLD    = 775;

// ─────────────────────────────────────────────
//  Ultrasonic obstacle detection
// ─────────────────────────────────────────────

// Speed of sound at 20°C ≈ 343 m/s.
// Round-trip distance formula: distance_m = time_us * 343 / 2_000_000
// Original Simulink model triggers obstacle logic at 0.11 m (11 cm).
// → threshold_us = 0.11 * 2_000_000 / 343 ≈ 641 μs
// A slightly generous margin of 800 μs (~13.8 cm) is used here.
constexpr uint32_t OBSTACLE_THRESHOLD_US   = 800;

// Moving-average filter window (50 samples, matching the SLX FIR filter)
constexpr uint8_t  ULTRASONIC_FILTER_SIZE  = 50;

// Maximum plausible ultrasonic reading (≈ 4 m, 23 320 μs). Reject readings above this.
constexpr uint32_t ULTRASONIC_MAX_US       = 23320;

// ─────────────────────────────────────────────
//  Line-following control signal
// ─────────────────────────────────────────────

// Control signal = IRRight - IRLeft  (proportional error)
// A |controlSignal| below this is treated as "robot is centred" (no significant steering)
constexpr int16_t FORWARD_THRESHOLD = 100;   // matches Simulink forwardThreshold = 100

// ─────────────────────────────────────────────
//  Package / mission parameters
// ─────────────────────────────────────────────

// Number of objects the robot must collect before returning to deliver
constexpr uint8_t MAX_PICKUPS = 3;

// ─────────────────────────────────────────────
//  Timing constants (milliseconds)
// ─────────────────────────────────────────────

// Dwell times matching Simulink after(N, sec) transitions
constexpr uint32_t GRAB_DWELL_MS       =  20;   // close gripper settle time
constexpr uint32_t PIVOT_DWELL_MS      =  40;   // pivot/turn settle time
constexpr uint32_t OBSTACLE_CONFIRM_MS = 100;   // obstacle debounce (Simulink: after(0.1, sec))
constexpr uint32_t DELIVER_REVERSE_MS  =  30;   // reverse before pivot during delivery
constexpr uint32_t DELIVER_COOLDOWN_MS = 1000;  // minimum ms after a pickup/delivery before delivery can re-trigger

// ─────────────────────────────────────────────
//  Encoder / odometry
// ─────────────────────────────────────────────
constexpr uint16_t ENCODER_PPR = 64;    // pulses per revolution (wheel encoder)
