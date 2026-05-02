/**
 * sensors.cpp
 * Sensor implementation: IR line sensors, ultrasonic distance, and wheel encoders.
 *
 * IR sensors are read raw via ESP32 ADC and rescaled to 0-1027 (matching spec).
 * Ultrasonic uses a non-blocking HC-SR04 trigger/echo scheme.
 * Encoders are counted in interrupt service routines (ISRs).
 * A 50-sample moving-average filter is applied to ultrasonic readings,
 * exactly mirroring the Simulink Discrete FIR Filter block (Coefficients = ones(1,50)/50).
 */

#include "sensors.h"
#include "config.h"

#include <Arduino.h>

// ─────────────────────────────────────────────
//  ESP32 ADC full-scale is 4095 (12-bit).
//  The spec quotes 0-1027 as the IR range.
//  Scale factor: 1027 / 4095 ≈ 0.2508
// ─────────────────────────────────────────────
static constexpr float IR_SCALE = 1027.0f / 4095.0f;

// ─────────────────────────────────────────────
//  Moving-average filter state for ultrasonic
// ─────────────────────────────────────────────
static uint32_t s_us_buf[ULTRASONIC_FILTER_SIZE] = {};
static uint8_t  s_us_idx   = 0;
static uint64_t s_us_sum   = 0;            // running sum to avoid recomputing each step
static bool     s_us_full  = false;        // true once the buffer has been filled once

// ─────────────────────────────────────────────
//  Latest filtered sensor values
// ─────────────────────────────────────────────
static int16_t  s_ir_right = 0;
static int16_t  s_ir_left  = 0;
static uint32_t s_ultrasonic_us = ULTRASONIC_MAX_US;

// ─────────────────────────────────────────────
//  Encoder counters (volatile: written in ISR)
// ─────────────────────────────────────────────
static volatile int32_t s_enc_right = 0;
static volatile int32_t s_enc_left  = 0;

static void IRAM_ATTR isr_enc_right() { s_enc_right++; }
static void IRAM_ATTR isr_enc_left()  { s_enc_left++;  }

// ─────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────

/**
 * Rescale 12-bit ESP32 ADC reading to 0-1027 as per spec.
 */
static inline int16_t adc_to_ir(int raw) {
    int16_t scaled = static_cast<int16_t>(raw * IR_SCALE);
    // Clamp to valid range
    if (scaled < 0)    scaled = 0;
    if (scaled > 1027) scaled = 1027;
    return scaled;
}

/**
 * Trigger an HC-SR04 measurement and return the raw round-trip time in μs.
 * This is a blocking call but takes at most ~24 ms (max range ≈ 4 m).
 * Returns ULTRASONIC_MAX_US if the echo times out.
 */
static uint32_t ultrasonic_measure_us() {
    // Send a 10-μs HIGH pulse on TRIG
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

    // Measure echo pulse width (timeout = ULTRASONIC_MAX_US μs)
    uint32_t duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH,
                                static_cast<unsigned long>(ULTRASONIC_MAX_US));

    // pulseIn returns 0 on timeout; treat as max range
    return (duration == 0) ? ULTRASONIC_MAX_US : duration;
}

/**
 * Push a new ultrasonic sample into the moving-average filter and return
 * the current average.  Implements the same 50-tap rectangular FIR filter
 * as the Simulink model: Coefficients = ones(1,50)/50.
 */
static uint32_t ultrasonic_filter_push(uint32_t new_sample) {
    // Subtract the oldest value from the running sum
    s_us_sum -= s_us_buf[s_us_idx];
    // Insert new sample
    s_us_buf[s_us_idx] = new_sample;
    s_us_sum += new_sample;
    // Advance index (ring buffer)
    s_us_idx = static_cast<uint8_t>((s_us_idx + 1) % ULTRASONIC_FILTER_SIZE);
    if (s_us_idx == 0) s_us_full = true;

    uint8_t count = s_us_full ? ULTRASONIC_FILTER_SIZE : s_us_idx;
    return static_cast<uint32_t>(s_us_sum / count);
}

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

void sensors_init() {
    // IR sensor pins are input-only ADC channels on ESP32 (no pinMode needed for ADC,
    // but setting INPUT explicitly for clarity)
    pinMode(PIN_IR_RIGHT, INPUT);
    pinMode(PIN_IR_LEFT,  INPUT);

    // Ultrasonic
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

    // Encoders with pull-up resistors; trigger on RISING edge of channel A
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_LEFT_A,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isr_enc_right, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A),  isr_enc_left,  RISING);

    // Warm up the ultrasonic filter with the first reading
    for (uint8_t i = 0; i < ULTRASONIC_FILTER_SIZE; ++i) {
        s_us_buf[i] = ULTRASONIC_MAX_US;
        s_us_sum   += ULTRASONIC_MAX_US;
    }
    s_us_full = true;
}

void sensors_update() {
    // --- IR sensors ---
    s_ir_right = adc_to_ir(analogRead(PIN_IR_RIGHT));
    s_ir_left  = adc_to_ir(analogRead(PIN_IR_LEFT));

    // --- Ultrasonic (filtered) ---
    uint32_t raw_us = ultrasonic_measure_us();
    s_ultrasonic_us = ultrasonic_filter_push(raw_us);
}

int16_t sensors_ir_right() { return s_ir_right; }
int16_t sensors_ir_left()  { return s_ir_left;  }

uint32_t sensors_ultrasonic_us() { return s_ultrasonic_us; }

int32_t sensors_encoder_right() { return s_enc_right; }
int32_t sensors_encoder_left()  { return s_enc_left;  }

void sensors_encoder_reset() {
    // Briefly disable interrupts to get a consistent zero
    noInterrupts();
    s_enc_right = 0;
    s_enc_left  = 0;
    interrupts();
}
