/**
 * DESIGN_RATIONALE.md
 * (Under 300 words – to be exported as PDF for submission)
 *
 * ──────────────────────────────────────────────────────────
 * DESIGN RATIONALE – Warehouse Robot Navigation System
 * ──────────────────────────────────────────────────────────
 *
 * ARCHITECTURE OVERVIEW
 * The system is structured into four clearly separated modules:
 * config.h (constants), sensors.cpp (HAL), control.cpp (actuators),
 * and fsm.cpp (behavioral logic). This separation minimises coupling
 * and makes each layer independently testable.
 *
 * STATE MACHINE
 * The navigation algorithm is a finite state machine ported directly
 * from the Simulink Stateflow model. It contains four top-level
 * superstates (DRIVE, PICKUP_OBJECT, DELIVER_OBJECT, STOP) decomposed
 * into 11 flat sub-states in C++ using a switch-case enum pattern.
 * Millis()-based timers replace Simulink's after(N, sec) transitions.
 *
 * LINE FOLLOWING
 * A proportional controller computes a steering correction as the
 * difference of the two IR sensor readings (controlSignal = IR_R − IR_L).
 * This is applied asymmetrically to the two continuous-rotation servos,
 * replicating the original model's 1400/1600 differential drive.
 *
 * OBSTACLE / OBJECT DETECTION
 * A 50-sample moving-average FIR filter (matching the Simulink Discrete
 * FIR block) smooths ultrasonic readings. A 100 ms debounce window
 * prevents single noisy samples from triggering state transitions.
 *
 * FAILSAFES
 * All motor pulse widths are hard-clamped to [1000, 2000] μs in
 * control_clamp_pulse() before being written to hardware. Only one FSM
 * state is active at any time (no parallel task conflict). The obstacle
 * debounce flag is consumed on each state transition to prevent
 * double-firing.
 *
 * ENCODERS
 * Wheel encoders are read via IRAM_ATTR ISRs (interrupt-safe) and
 * exposed for odometry, future closed-loop speed control, or dead-reckoning.
 *
 * STRUGGLES / INCOMPLETE AREAS
 * The gripper angle mapping (115°/180°) was carried from the Simulink
 * model (which used 190° – clamped to the servo maximum of 180°) and
 * may need physical calibration. The DELIVER_COOLDOWN_MS constant
 * (1000 ms) prevents re-triggering delivery immediately after a pickup
 * or delivery; its value may need empirical tuning on hardware.
 */
