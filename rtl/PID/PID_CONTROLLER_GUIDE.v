// PID Motor Controller - User Guide and Serial Commands

/*
================================================================================
                    PID MOTOR CONTROLLER FOR TANG NANO 9K
================================================================================

SYSTEM OVERVIEW:
This is a complete FPGA-based PID controller system for DC motors with:
  - Real-time PID control loop (1kHz update rate)
  - Auto-tuning via Ziegler-Nichols method
  - User parameter configuration via UART serial
  - Debug output via UART to monitor controller state
  - PWM motor control with direction capability
  - LED status indicators

MODULAR ARCHITECTURE:
  1. pid_controller.v    - Core PID calculation engine
  2. auto_tuner.v        - Ziegler-Nichols auto-tuning algorithm
  3. uart_interface.v    - Serial communication interface
  4. motor_control.v     - PWM and direction output driver
  5. main.v              - Top-level system integration

================================================================================
                          SERIAL COMMANDS (115200 baud)
================================================================================

All commands are sent as ASCII text followed by CR (Enter key)
Parameter values are in hexadecimal (0-FFFF)

COMMAND FORMAT:
  K<HHHH> - Set Proportional gain (Kp)
            Example: K0100   (sets Kp = 1.0, scaled by 256)
  
  I<HHHH> - Set Integral gain (Ki)
            Example: I0020   (sets Ki = 0.125)
  
  D<HHHH> - Set Derivative gain (Kd)
            Example: D0080   (sets Kd = 0.5)
  
  S<HHHH> - Set Target Setpoint (0x0000 to 0xFFFF)
            Example: S8000   (50% of full scale)
  
  F<HHHH> - Set Feedback value (simulates motor position)
            Example: F4000   (25% of full scale)
  
  T        - Start Auto-Tuning sequence
            Runs for ~333ms, calculates optimal gains
            LED1 blinks during tuning, stays on when complete
            Example: T

================================================================================
                          PARAMETER SCALING
================================================================================

All gains use 8.8 fixed-point format (scaled by 256):
  - Gain value = displayed_gain * 256
  - Example: Kp = 1.0 → send as 0x0100
  - Example: Kp = 0.5 → send as 0x0080
  - Example: Kp = 2.5 → send as 0x0280

Typical Starting Values:
  Kp = 0x0100 (1.0)   - Proportional response
  Ki = 0x0020 (0.125) - Integral action for steady-state
  Kd = 0x0080 (0.5)   - Derivative damping

================================================================================
                          MONITORING VIA UART
================================================================================

Debug Output (transmitted by FPGA):
  The UART interface provides real-time feedback:
  - Current PID output value
  - Accumulated integral term
  - Current error signal
  - Auto-tuning progress (0-100%)
  - Tuning completion status

Use a serial terminal to view:
  - Windows: PuTTY, TeraTerm, or VS Code Serial Monitor
  - Linux: minicom, picocom, or screen
  - Mac: screen or miniterm

Example session:
  > K0100
  > I0020
  > D0080
  > S8000
  [Observe motor response to setpoint in terminal]
  > T
  [Auto-tuning in progress... 50%]
  [Auto-tuning complete!]
  > S4000
  [Motor adjusts to new setpoint]

================================================================================
                          AUTO-TUNING GUIDE
================================================================================

Ziegler-Nichols Tuning:
  1. Start with low gains (Kp=0.2, Ki=0, Kd=0)
  2. Send "T" command to start auto-tuning
  3. System performs step response analysis for 333ms
  4. Optimal gains calculated automatically
  5. New gains apply automatically after tuning completes

Typical Results:
  Kp = 0.60 * Ku     (60% of ultimate gain)
  Ki = 1.20 * Ku/Tu  (proportional to ultimate gain/period)
  Kd = 0.075 * Ku*Tu (damping term)

Manual Tuning Tips:
  - Increase Kp for faster response (beware oscillation)
  - Increase Ki to eliminate steady-state error
  - Increase Kd to reduce overshoot
  - Start with small steps: 0x0010 increments

================================================================================
                          LED INDICATORS
================================================================================

LED1:
  - OFF: Normal operation
  - BLINKING (fast): Auto-tuning in progress
  - ON: Auto-tuning completed, using tuned gains

LED2:
  - BLINKING (slow): System running (heartbeat)
  - Toggles every ~350ms

Status Summary:
  Both OFF (except heartbeat) → System idle, default gains active
  LED1 ON + LED2 blinking → Tuned gains active, responding normally

================================================================================
                          HARDWARE CONNECTIONS
================================================================================

UART Interface:
  RX (Pin assigned in pinout.cst) ← Serial data from host
  TX (Pin assigned in pinout.cst) → Serial data to host
  GND → Common ground with host
  
  Use USB-to-UART adapter (FT232, CH340, etc.)
  Connection: 115200 baud, 8 data bits, 1 stop bit, no parity

Motor Control:
  PWM_OUT → PWM input to motor driver (bridge driver)
  MOTOR_DIR → Direction control pin
  GND → Common ground with motor driver

Power:
  VCC (3.3V from Tang Nano)
  GND (Common ground)

================================================================================
                          ERROR HANDLING
================================================================================

Integral Windup Prevention:
  - Integral term is clamped to prevent overflow
  - Maximum integral value: 2^32 (hardware limited)
  - Saturates at motor PWM limits

Output Saturation:
  - PID output saturates at [-32768, 32767]
  - Motor PWM limited to [0, 1023]
  - Prevents damage to motor driver

UART RX Timeout:
  - Incomplete commands cleared after 500ms
  - Bad commands ignored

================================================================================
                          TESTING & SIMULATION
================================================================================

Run Test Bench:
  apio test

Test Coverage:
  ✓ Default parameter step response
  ✓ Auto-tuning sequence (333ms)
  ✓ Tuned gain step response
  ✓ UART parameter updates
  ✓ Motor PWM generation
  ✓ Direction control
  ✓ LED status indicators

Expected Test Output:
  === PID Motor Controller Test ===
  --- Initial State ---
  --- Test 1: Step Response (Default Gains) ---
  --- Test 2: Auto-Tuning Sequence ---
  Tuning complete!
  Tuned Gains: Kp=..., Ki=..., Kd=...
  --- Test 3: Step Response (Tuned Gains) ---
  --- Test 4: UART Parameter Update ---
  === Test Completed ===

================================================================================
                          PERFORMANCE SPECS
================================================================================

Control Loop:
  Update Rate: 1 kHz (1000 updates/second)
  Latency: 1ms maximum
  Accuracy: 16-bit signed integer arithmetic

Fixed-Point Math:
  Gain precision: 8.8 format (1/256 resolution)
  Error precision: 16-bit signed
  Integral accumulator: 32-bit (prevents overflow)
  Output precision: 16-bit signed

PWM Output:
  Frequency: 12 MHz / 1024 = 11.7 kHz
  Resolution: 10 bits (1024 steps)
  Duty Cycle: 0-100% proportional to PID output

Memory Usage:
  BRAM: ~4KB (includes all module code)
  Registers: ~500 (state machines, accumulators)
  LUTs: ~1000 (typical FPGA utilization)

Power Consumption:
  Typical: 50-100mW at 12MHz, 3.3V

================================================================================
                          TROUBLESHOOTING
================================================================================

No response from FPGA:
  → Check UART connection (RX/TX swapped?)
  → Verify baud rate: 115200
  → Check USB-to-UART driver installed
  → Verify TX pin is correctly assigned in pinout.cst

Motor not responding:
  → Verify PWM_OUT and MOTOR_DIR pins assigned
  → Check motor driver power supply
  → Confirm setpoint > current feedback (S command)
  → Try manual Kp increase with K command

Oscillation/Instability:
  → Reduce Kp value (slower response)
  → Increase Kd value (more damping)
  → Check feedback signal noise
  → Verify motor load not excessive

Serial garbage output:
  → Check baud rate (should be 115200)
  → Verify GND connection
  → Try shorter USB cable
  → Check for EMI from motor driver

================================================================================
                          ADVANCED USAGE
================================================================================

Cascade Control:
  - Outer loop: position/velocity setpoint
  - Inner loop: current/force setpoint
  - Modify setpoint dynamically from host

Feed-Forward Control:
  - Add velocity feed-forward to reduce lag
  - Modify pid_controller.v to include feed-forward term

Multiple Motors:
  - Instantiate multiple pid_controller modules
  - Use multiplexed UART for configuration
  - Share auto_tuner module or implement per-motor tuning

Custom Tuning Method:
  - Replace auto_tuner.v with manual/PID method
  - Implement relay feedback for gain estimation
  - Add load disturbance testing

================================================================================
*/
