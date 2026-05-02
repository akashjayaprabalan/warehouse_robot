/**
 * fsm.h
 * Finite State Machine for warehouse robot navigation and package handling.
 *
 * Top-level states mirror the Simulink Stateflow chart exactly:
 *
 *   DRIVE          – line following, obstacle scanning
 *   PICKUP_OBJECT  – superstate: stop → grab → pivot → exit
 *   DELIVER_OBJECT – superstate: grab → reverse → pivot → exit  (triggered at start line)
 *   STOP           – superstate: final delivery sequence (after MAX_PICKUPS collected)
 *
 * State transition diagram (simplified):
 *
 *   [init] ──→ DRIVE
 *                │
 *      obstacle detected (not gripping, pickups < MAX) ──→ PICKUP_OBJECT
 *      obstacle detected (not gripping, pickups == MAX) ──→ STOP
 *      both IR on line + gripping + timer ──→ DELIVER_OBJECT
 *                │
 *   PICKUP_OBJECT ──[isTurning == 0]──→ DRIVE
 *   DELIVER_OBJECT ──[isTurning == 0]──→ DRIVE
 *   STOP ──[isTurning == 0]──→ DRIVE  (continues patrolling after deposit)
 *
 * Provides:
 *   fsm_init()    – reset state to DRIVE and clear counters
 *   fsm_update()  – call every loop iteration with fresh sensor values;
 *                   internally commands motors and gripper
 */

#pragma once

#include <stdint.h>

/**
 * Reset the FSM to its initial DRIVE state.
 * Must be called once from setup() after control_init() and sensors_init().
 */
void fsm_init();

/**
 * Execute one FSM tick.
 * Reads pre-updated sensor values via sensors_* functions, computes control
 * outputs, and writes motor/gripper commands via control_* functions.
 *
 * Must be called every loop iteration after sensors_update().
 */
void fsm_update();

/** @return Current number of objects successfully picked up. */
uint8_t fsm_pickup_count();

/** @return true if the robot is currently holding at least one object. */
bool fsm_is_gripping();
