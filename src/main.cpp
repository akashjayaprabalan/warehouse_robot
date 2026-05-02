/**
 * main.cpp
 * Entry point for the ESP32 warehouse robot.
 *
 * Initialises all subsystems in the correct order then runs the
 * control loop at the maximum safe rate (rate-limited by the
 * blocking ultrasonic measurement, ~24 ms per cycle at max range).
 *
 * Boot sequence:
 *   1. Serial (debug output)
 *   2. LED pin
 *   3. Sensor hardware
 *   4. Motor / gripper servos
 *   5. FSM reset
 *   6. Loop forever: update sensors → tick FSM → debug print
 */

#include <Arduino.h>

#include "config.h"
#include "sensors.h"
#include "control.h"
#include "fsm.h"

// ─────────────────────────────────────────────
//  Debug print interval (ms) – print once per second to avoid flooding serial
// ─────────────────────────────────────────────
static constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 1000;
static uint32_t s_last_debug_print_ms = 0;

// ─────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("[BOOT] Warehouse Robot v1.0");

    // Status LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // Initialise subsystems in dependency order
    sensors_init();
    Serial.println("[INIT] Sensors OK");

    control_init();
    Serial.println("[INIT] Servos OK");

    fsm_init();
    Serial.println("[INIT] FSM OK");

    // Brief startup blink to signal readiness
    for (uint8_t i = 0; i < 3; ++i) {
        digitalWrite(PIN_LED, HIGH); delay(150);
        digitalWrite(PIN_LED, LOW);  delay(150);
    }

    Serial.println("[READY] Starting main loop");
}

// ─────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────
void loop() {
    // 1. Refresh all sensor readings (includes blocking ultrasonic pulse)
    sensors_update();

    // 2. Execute one FSM tick – reads sensors, commands actuators
    fsm_update();

    // 3. LED mirrors gripper state (on = holding an object)
    digitalWrite(PIN_LED, fsm_is_gripping() ? HIGH : LOW);

    // 4. Periodic debug output over Serial
    uint32_t now = millis();
    if (now - s_last_debug_print_ms >= DEBUG_PRINT_INTERVAL_MS) {
        s_last_debug_print_ms = now;
        Serial.printf("[DBG] IR_R=%4d  IR_L=%4d  US=%6lu us  "
                      "pickups=%d  gripping=%d  enc_R=%ld  enc_L=%ld\n",
                      sensors_ir_right(),
                      sensors_ir_left(),
                      (unsigned long)sensors_ultrasonic_us(),
                      fsm_pickup_count(),
                      fsm_is_gripping() ? 1 : 0,
                      (long)sensors_encoder_right(),
                      (long)sensors_encoder_left());
    }
}
