/**
 * control.cpp
 * Control signal computation and servo-based motor / gripper output.
 *
 * The line-following control law is a simple proportional controller:
 *
 *   controlSignal = IRRight - IRLeft
 *
 *   RightWheel = 1400 - controlSignal / 2
 *   LeftWheel  = 1600 - controlSignal / 2
 *
 * This directly mirrors the Simulink model where:
 *   - 1400/1600 is the default forward differential (left slightly faster)
 *   - The control signal steers by slowing/speeding the appropriate wheel
 *
 * All motor outputs are hard-clamped to [MOTOR_MIN_PULSE, MOTOR_MAX_PULSE]
 * as a failsafe — impossible speeds can never be sent to hardware.
 */

#include "control.h"
#include "config.h"

#include <Arduino.h>
#include <ESP32Servo.h>    // ESP32-compatible servo library

// ─────────────────────────────────────────────
//  Servo objects
// ─────────────────────────────────────────────
static Servo s_servo_right;
static Servo s_servo_left;
static Servo s_servo_gripper;

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

void control_init() {
    // Allocate ESP32 hardware timers for each servo channel
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);

    // Continuous-rotation servos: 50 Hz, pulse range 1000–2000 μs
    s_servo_right.setPeriodHertz(50);
    s_servo_right.attach(PIN_MOTOR_RIGHT, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);

    s_servo_left.setPeriodHertz(50);
    s_servo_left.attach(PIN_MOTOR_LEFT, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);

    // Standard servo for gripper: 50 Hz, 500–2400 μs (broad range for angle mapping)
    s_servo_gripper.setPeriodHertz(50);
    s_servo_gripper.attach(PIN_GRIPPER, 500, 2400);

    // Start with motors stopped and gripper open
    motor_stop();
    gripper_set(GRIPPER_OPEN);
}

int16_t control_clamp_pulse(int16_t pulse) {
    if (pulse < MOTOR_MIN_PULSE) return MOTOR_MIN_PULSE;
    if (pulse > MOTOR_MAX_PULSE) return MOTOR_MAX_PULSE;
    return pulse;
}

int16_t control_signal(int16_t ir_right, int16_t ir_left) {
    // Proportional error: positive → robot drifting right
    return static_cast<int16_t>(ir_right - ir_left);
}

void motor_set(int16_t right_pulse, int16_t left_pulse) {
    int16_t r = control_clamp_pulse(right_pulse);
    int16_t l = control_clamp_pulse(left_pulse);
    s_servo_right.writeMicroseconds(r);
    s_servo_left.writeMicroseconds(l);
}

void motor_stop() {
    motor_set(MOTOR_STOP, MOTOR_STOP);
}

void gripper_set(uint8_t angle) {
    // Clamp to standard servo range
    if (angle > 180) angle = 180;
    s_servo_gripper.write(angle);
}
