/**
 * sensors.h
 * Sensor initialisation and read functions.
 *
 * Provides:
 *   sensors_init()              – one-time hardware setup
 *   sensors_update()            – call every loop iteration to refresh cached values
 *   sensors_ir_right()          – latest filtered IR right value  (0-1027)
 *   sensors_ir_left()           – latest filtered IR left  value  (0-1027)
 *   sensors_ultrasonic_us()     – latest filtered ultrasonic round-trip time (μs)
 *   sensors_encoder_right()     – cumulative right encoder pulse count
 *   sensors_encoder_left()      – cumulative left  encoder pulse count
 *   sensors_encoder_reset()     – zero both encoder counters
 */

#pragma once

#include <stdint.h>

/**
 * Initialise sensor hardware (ADC pins, ultrasonic pins, encoder interrupts).
 * Must be called once inside setup().
 */
void sensors_init();

/**
 * Refresh all sensor readings and run filters.
 * Must be called once per main-loop iteration.
 */
void sensors_update();

/** @return Latest IR right reading (0=dark/line, 1027=light/floor). */
int16_t sensors_ir_right();

/** @return Latest IR left reading (0=dark/line, 1027=light/floor). */
int16_t sensors_ir_left();

/**
 * @return Moving-average-filtered ultrasonic round-trip time in μs.
 *         Returns ULTRASONIC_MAX_US if no obstacle is detected within range.
 */
uint32_t sensors_ultrasonic_us();

/** @return Cumulative right encoder pulse count since last reset. */
int32_t sensors_encoder_right();

/** @return Cumulative left encoder pulse count since last reset. */
int32_t sensors_encoder_left();

/** Zero both encoder counters. */
void sensors_encoder_reset();
