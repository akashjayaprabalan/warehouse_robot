/**
 * fsm.cpp
 * Finite State Machine implementation.
 *
 * This file is a faithful C++ translation of the Simulink Stateflow chart
 * (chart_95.xml / chart_85.xml extracted from WS11_10_G14.slx), adapted to:
 *   - ESP32 hardware (millis() timing instead of Simulink clock)
 *   - New sensor spec: IR 0-1027, Ultrasonic in μs
 *   - Encoder-based odometry (additional to original model)
 *   - Hard failsafes on all motor commands
 *
 * ─────────────────────────────────────────────────────────────────────────
 * STATE MACHINE OVERVIEW
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Flat enum of all active states (superstates decomposed into sub-states):
 *
 *   DRIVE             — line following (root initial state)
 *   PICKUP_GRAB       — stop + close gripper (PickupObject superstate, step 1)
 *   PICKUP_PIVOT      — both wheels forward = pivot/turn (step 2)
 *   PICKUP_EXIT       — flag object as held, increment counter (step 3)
 *   DELIVER_GRAB      — open gripper + stop (DeliverObject superstate, step 1)
 *   DELIVER_REVERSE   — reverse briefly to clear the start line (step 2)
 *   DELIVER_PIVOT     — pivot to face track again (step 3)
 *   DELIVER_EXIT      — release grip flag, reset delivery timer (step 4)
 *   STOP_GRAB         — close gripper while stopped (Stop superstate, step 1)
 *   STOP_DRIVE        — continue line-following inside Stop superstate
 *   STOP_STOP         — full stop, open gripper (deposit objects)
 *
 * ─────────────────────────────────────────────────────────────────────────
 * TRANSITION CONDITIONS (directly from Simulink labels)
 * ─────────────────────────────────────────────────────────────────────────
 *
 * DRIVE → PICKUP_OBJECT :
 *   after(100 ms) [ ultrasonic < OBSTACLE_THRESHOLD_US
 *                   && !isGripping
 *                   && countPickup != MAX_PICKUPS ]
 *
 * DRIVE → STOP_OBJECT :
 *   after(100 ms) [ ultrasonic < OBSTACLE_THRESHOLD_US
 *                   && !isGripping
 *                   && countPickup == MAX_PICKUPS ]
 *
 * DRIVE → DELIVER :
 *   [ IRLeft < IR_LINE_THRESHOLD
 *     && IRRight < IR_LINE_THRESHOLD
 *     && abs(controlSignal) < FORWARD_THRESHOLD
 *     && isGripping
 *     && clock - deltaTime >= deltaTime - 100 ms ]
 *
 * (All after(N, ms) timers are implemented with millis() in C++.)
 */

#include "fsm.h"
#include "config.h"
#include "sensors.h"
#include "control.h"

#include <Arduino.h>

// ─────────────────────────────────────────────
//  State enum
// ─────────────────────────────────────────────
enum class State : uint8_t {
    DRIVE,
    PICKUP_GRAB,
    PICKUP_PIVOT,
    PICKUP_EXIT,
    DELIVER_GRAB,
    DELIVER_REVERSE,
    DELIVER_PIVOT,
    DELIVER_EXIT,
    STOP_GRAB,
    STOP_DRIVE,
    STOP_STOP
};

// ─────────────────────────────────────────────
//  FSM persistent variables
// ─────────────────────────────────────────────
static State    s_state        = State::DRIVE;
static bool     s_is_gripping  = false;
static bool     s_is_turning   = false;
static uint8_t  s_count_pickup = 0;

// Timer: records the millis() timestamp when we entered the current state
static uint32_t s_state_entry_ms = 0;

// Delivery cooldown timer (mirrors DeltaTime in Simulink).
// Set to millis() when an object is picked up; prevents immediately
// re-triggering a delivery action right after a pickup.
static uint32_t s_delta_time_ms = 0;

// Obstacle debounce: records when we first saw a close obstacle reading
static uint32_t s_obstacle_first_seen_ms = 0;
static bool     s_obstacle_debouncing    = false;

// ─────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────

/** How many milliseconds since we entered the current state. */
static inline uint32_t ms_in_state() {
    return millis() - s_state_entry_ms;
}

/** Transition to a new state and record the entry timestamp. */
static void transition_to(State next) {
    s_state          = next;
    s_state_entry_ms = millis();
}

/**
 * Compute drive wheel speeds from the proportional control signal.
 * Mirrors Simulink:
 *   RightWheel = 1400 - controlSignal / 2
 *   LeftWheel  = 1600 - controlSignal / 2
 */
static void apply_drive_control(int16_t ctrl) {
    int16_t right_pw = static_cast<int16_t>(MOTOR_REV_FULL  - ctrl / 2);   // 1400 - ctrl/2
    int16_t left_pw  = static_cast<int16_t>(MOTOR_FWD_FULL  - ctrl / 2);   // 1600 - ctrl/2
    motor_set(right_pw, left_pw);
}

/**
 * Check whether the obstacle debounce condition has been met.
 * Mirrors Simulink: after(0.1, sec)[Ultrasonic < OBSTACLE_THRESHOLD_US].
 * Returns true only once per confirmed obstacle event; caller must reset
 * s_obstacle_debouncing when the event is consumed.
 */
static bool obstacle_confirmed(uint32_t ultrasonic_us) {
    bool close = (ultrasonic_us < OBSTACLE_THRESHOLD_US);

    if (!close) {
        // No obstacle – reset debounce
        s_obstacle_debouncing    = false;
        s_obstacle_first_seen_ms = 0;
        return false;
    }

    if (!s_obstacle_debouncing) {
        // First sample within threshold – start timer
        s_obstacle_debouncing    = true;
        s_obstacle_first_seen_ms = millis();
        return false;
    }

    // Obstacle has been continuously detected for the debounce window
    return (millis() - s_obstacle_first_seen_ms >= OBSTACLE_CONFIRM_MS);
}

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

void fsm_init() {
    s_state                  = State::DRIVE;
    s_is_gripping            = false;
    s_is_turning             = false;
    s_count_pickup           = 0;
    s_state_entry_ms         = millis();
    s_delta_time_ms          = millis();
    s_obstacle_debouncing    = false;
    s_obstacle_first_seen_ms = 0;

    // Ensure safe initial hardware state
    motor_stop();
    gripper_set(GRIPPER_OPEN);
}

void fsm_update() {
    // --- Gather sensor data ------------------------------------------------
    int16_t  ir_right   = sensors_ir_right();
    int16_t  ir_left    = sensors_ir_left();
    uint32_t ultra_us   = sensors_ultrasonic_us();
    uint32_t now_ms     = millis();

    // Proportional control error (line-following)
    int16_t ctrl = control_signal(ir_right, ir_left);

    // --- State machine -------------------------------------------------------
    switch (s_state) {

    // ════════════════════════════════════════════════════════════════════
    //  DRIVE  –  normal line following
    // ════════════════════════════════════════════════════════════════════
    case State::DRIVE: {
        // Drive outputs (mirrors Simulink DRIVE state)
        apply_drive_control(ctrl);

        // ── Transition 1: Obstacle detected, not at capacity → pick up ──
        if (obstacle_confirmed(ultra_us) && !s_is_gripping
                && s_count_pickup != MAX_PICKUPS) {
            s_obstacle_debouncing = false;   // consume event
            s_is_turning = true;
            transition_to(State::PICKUP_GRAB);
            break;
        }

        // ── Transition 2: Obstacle detected, at capacity → deliver remaining ──
        if (obstacle_confirmed(ultra_us) && !s_is_gripping
                && s_count_pickup == MAX_PICKUPS) {
            s_obstacle_debouncing = false;
            transition_to(State::STOP_GRAB);
            break;
        }

        // ── Transition 3: Start-line detected while carrying objects ──
        // Conditions (Simulink):
        //   IRLeft < 650 && IRRight < 650                → both sensors on dark strip
        //   abs(controlSignal) < FORWARD_THRESHOLD       → robot heading straight
        //   isGripping == 1                              → holding an object
        //   clock - deltaTime >= deltaTime - 100 ms      → cooldown elapsed
        bool on_dark = (ir_left < IR_LINE_THRESHOLD && ir_right < IR_LINE_THRESHOLD);
        bool heading_straight = (ctrl > -FORWARD_THRESHOLD && ctrl < FORWARD_THRESHOLD);
        bool cooldown_ok = (now_ms - s_delta_time_ms >= DELIVER_COOLDOWN_MS);

        if (on_dark && heading_straight && s_is_gripping && cooldown_ok) {
            s_is_turning = true;
            transition_to(State::DELIVER_GRAB);
            break;
        }

        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  PICKUP_GRAB  –  stop and close gripper around the detected object
    //  Simulink: LeftWheel=1500, RightWheel=1500, GripperAngle=115
    // ════════════════════════════════════════════════════════════════════
    case State::PICKUP_GRAB: {
        motor_stop();
        gripper_set(GRIPPER_CLOSE);

        // Record delta time (Simulink: DeltaTime = Clock - DeltaTime)
        // Here we record the entry timestamp for cooldown use.

        // after(0.02, sec) → advance to pivot
        if (ms_in_state() >= GRAB_DWELL_MS) {
            transition_to(State::PICKUP_PIVOT);
        }
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  PICKUP_PIVOT  –  pivot until back on track
    //  Simulink: RightWheel=1600, LeftWheel=1600
    // ════════════════════════════════════════════════════════════════════
    case State::PICKUP_PIVOT: {
        // Both wheels forward = spin/pivot (differential is zero → rotate in place)
        motor_set(MOTOR_FWD_FULL, MOTOR_FWD_FULL);

        // after(0.04, sec) [IRRight < 650] → confirm we're back on line, exit
        if (ms_in_state() >= PIVOT_DWELL_MS && ir_right < IR_LINE_THRESHOLD) {
            transition_to(State::PICKUP_EXIT);
        }
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  PICKUP_EXIT  –  update flags; return to DRIVE when turn complete
    //  Simulink: isTurning=0, isGripping=1, CountPickup++
    // ════════════════════════════════════════════════════════════════════
    case State::PICKUP_EXIT: {
        s_is_turning   = false;
        s_is_gripping  = true;
        s_count_pickup = static_cast<uint8_t>(s_count_pickup + 1);
        // Record pickup time for delivery cooldown
        s_delta_time_ms = millis();

        // Transition immediately back to DRIVE
        transition_to(State::DRIVE);
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  DELIVER_GRAB  –  stop and open gripper to release objects
    //  Simulink: LeftWheel=1500, RightWheel=1500, GripperAngle=190
    // ════════════════════════════════════════════════════════════════════
    case State::DELIVER_GRAB: {
        motor_stop();
        gripper_set(GRIPPER_OPEN);

        // after(0.02, sec) → reverse
        if (ms_in_state() >= GRAB_DWELL_MS) {
            transition_to(State::DELIVER_REVERSE);
        }
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  DELIVER_REVERSE  –  reverse briefly to clear the starting line
    //  Simulink: RightWheel=1600, LeftWheel=1400
    // ════════════════════════════════════════════════════════════════════
    case State::DELIVER_REVERSE: {
        motor_set(MOTOR_FWD_FULL, MOTOR_REV_FULL);   // 1600 R, 1400 L → reverse-right arc

        // after(0.03, sec) → pivot
        if (ms_in_state() >= DELIVER_REVERSE_MS) {
            transition_to(State::DELIVER_PIVOT);
        }
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  DELIVER_PIVOT  –  pivot to face the track again
    //  Simulink: RightWheel=1600, LeftWheel=1600
    // ════════════════════════════════════════════════════════════════════
    case State::DELIVER_PIVOT: {
        motor_set(MOTOR_FWD_FULL, MOTOR_FWD_FULL);

        // after(0.04, sec) [IRRight < 650] → confirm re-alignment
        if (ms_in_state() >= PIVOT_DWELL_MS && ir_right < IR_LINE_THRESHOLD) {
            transition_to(State::DELIVER_EXIT);
        }
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  DELIVER_EXIT  –  clear gripping flag; update delivery timer
    //  Simulink: isTurning=0, isGripping=0, DeltaTime=Clock
    // ════════════════════════════════════════════════════════════════════
    case State::DELIVER_EXIT: {
        s_is_turning    = false;
        s_is_gripping   = false;
        s_delta_time_ms = millis();   // reset cooldown (DeltaTime = Clock)

        // [isTurning == 0] → DRIVE  (condition is trivially satisfied here)
        transition_to(State::DRIVE);
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  STOP_GRAB  –  Stop superstate, step 1: close gripper
    //  (All MAX_PICKUPS objects collected; robot approaches final deposit)
    //  Simulink Stop.Grab: GripperAngle=115, isTurning=1
    // ════════════════════════════════════════════════════════════════════
    case State::STOP_GRAB: {
        motor_stop();
        gripper_set(GRIPPER_CLOSE);
        s_is_turning = true;

        // after(0.02, sec) → continue line following inside Stop superstate
        if (ms_in_state() >= GRAB_DWELL_MS) {
            transition_to(State::STOP_DRIVE);
        }
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  STOP_DRIVE  –  Stop superstate, step 2: line follow toward deposit
    //  Simulink Stop.DRIVE1: same control law as DRIVE
    // ════════════════════════════════════════════════════════════════════
    case State::STOP_DRIVE: {
        apply_drive_control(ctrl);

        // [IRLeft > 775 && IRRight > 775 && abs(controlSignal) < FORWARD_THRESHOLD]
        // → both sensors off the dark line → robot has crossed the start/deposit line
        bool over_floor = (ir_left  > IR_FLOOR_THRESHOLD &&
                           ir_right > IR_FLOOR_THRESHOLD);
        bool centred    = (ctrl > -FORWARD_THRESHOLD && ctrl < FORWARD_THRESHOLD);

        if (over_floor && centred) {
            transition_to(State::STOP_STOP);
        }
        break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  STOP_STOP  –  Stop superstate, step 3: full stop + open gripper
    //  Simulink Stop.Stop: LeftWheel=1500, RightWheel=1500, GripperAngle=190
    //  After deposit, reset counter and return to DRIVE for next mission.
    // ════════════════════════════════════════════════════════════════════
    case State::STOP_STOP: {
        motor_stop();
        gripper_set(GRIPPER_OPEN);

        // Dwell long enough for gripper to fully open (reuse GRAB_DWELL_MS)
        if (ms_in_state() >= GRAB_DWELL_MS) {
            // Reset for next pickup cycle
            s_is_gripping   = false;
            s_is_turning    = false;
            s_count_pickup  = 0;
            s_delta_time_ms = millis();
            transition_to(State::DRIVE);
        }
        break;
    }

    default:
        // Should never reach here – failsafe: halt robot
        motor_stop();
        break;
    }
}

uint8_t fsm_pickup_count()  { return s_count_pickup; }
bool    fsm_is_gripping()   { return s_is_gripping;  }
