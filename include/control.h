/**
 * control.h
 * Line-following control signal computation and motor/gripper output.
 *
 * Provides:
 *   control_init()              – one-time servo setup
 *   control_signal()            – compute proportional error from IR readings
 *   motor_set(right, left)      – write clamped pulse widths to both wheel servos
 *   motor_stop()                – convenience: set both wheels to MOTOR_STOP
 *   gripper_set(angle)          – write gripper servo angle
 *   control_clamp_pulse(pulse)  – clamp a pulse width to safe hardware limits (failsafe)
 */

#pragma once

#include <stdint.h>

/**
 * Initialise servo objects for both motors and the gripper.
 * Must be called once inside setup().
 */
void control_init();

/**
 * Compute the proportional line-following control signal.
 * controlSignal = IRRight - IRLeft  (mirrors the Simulink Sum block +-).
 *
 * Positive value → robot drifting right  → steer left.
 * Negative value → robot drifting left   → steer right.
 *
 * @param ir_right  Current right IR reading (0-1027).
 * @param ir_left   Current left  IR reading (0-1027).
 * @return          Signed control signal in the range [-1027, +1027].
 */
int16_t control_signal(int16_t ir_right, int16_t ir_left);

/**
 * Write pulse widths to both wheel servos.
 * Values are clamped to [MOTOR_MIN_PULSE, MOTOR_MAX_PULSE] before writing
 * as a hardware failsafe – it is impossible to command an out-of-range speed.
 *
 * @param right_pulse  Desired right-wheel pulse width (μs).
 * @param left_pulse   Desired left-wheel  pulse width (μs).
 */
void motor_set(int16_t right_pulse, int16_t left_pulse);

/** Set both motors to MOTOR_STOP (1500 μs). */
void motor_stop();

/**
 * Write a position to the gripper servo.
 * @param angle  Angle in degrees, clamped to [0, 180].
 */
void gripper_set(uint8_t angle);

/**
 * Clamp a raw motor pulse width to safe hardware limits.
 * Used internally by motor_set() and publicly available for unit testing.
 *
 * @param pulse  Raw pulse width in μs.
 * @return       Clamped pulse width guaranteed to be within [MOTOR_MIN_PULSE, MOTOR_MAX_PULSE].
 */
int16_t control_clamp_pulse(int16_t pulse);
