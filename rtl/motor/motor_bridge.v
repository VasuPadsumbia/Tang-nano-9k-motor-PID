// =============================================================================
// Module  : motor_bridge
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/motor_bridge.v
//
// Purpose : Maps the signed 16-bit PID output to PWM duty cycle and direction
//           signals suitable for an external H-bridge (L298N, DRV8833, TB6612).
//
//   PID output > +DEADBAND  → motor_dir=1 (forward),  pwm_duty = |pid_out|
//   PID output < -DEADBAND  → motor_dir=0 (reverse),  pwm_duty = |pid_out|
//   |PID output| ≤ DEADBAND → motor_en=0, coast/brake, pwm_duty = 0
//
// Dead-band eliminates low-level jitter/oscillation near zero.
//
// Parameters
//   DEADBAND  – Minimum absolute PID output to activate motor (default 256/65535)
//
// Interface
//   clk       – System clock
//   rst       – Synchronous reset
//   pid_out   – Signed 16-bit PID output
//   pwm_duty  – 16-bit unsigned PWM duty (to pwm_gen)
//   motor_dir – Direction bit:  1=forward (IN1 high), 0=reverse (IN2 high)
//   motor_en  – Motor enable:   1=drive,  0=coast (both INx low)
//
// H-bridge wiring:
//   PWM_OUT   → H-bridge EN/PWM pin
//   motor_dir → H-bridge IN1 pin
//   ~motor_dir → H-bridge IN2 pin  (must invert externally or in your board)
// =============================================================================

`default_nettype none

module motor_bridge #(
    parameter [15:0] DEADBAND = 16'd256    // ~0.4 % of full scale
)(
    input  wire              clk,
    input  wire              rst,
    input  wire signed [15:0] pid_out,

    output reg  [15:0]       pwm_duty,
    output reg               motor_dir,
    output reg               motor_en
);

    // Absolute value of pid_out (unsigned 16-bit)
    wire        neg      = pid_out[15];           // 1 if negative
    wire [15:0] abs_out  = neg ? (~pid_out + 1'b1) : pid_out[15:0];

    always @(posedge clk) begin
        if (rst) begin
            pwm_duty  <= 16'h0;
            motor_dir <= 1'b0;
            motor_en  <= 1'b0;
        end else begin
            if (abs_out <= DEADBAND) begin
                // Dead-band: coast motor
                pwm_duty  <= 16'h0;
                motor_dir <= 1'b0;
                motor_en  <= 1'b0;
            end else begin
                motor_en  <= 1'b1;
                motor_dir <= ~neg;          // positive PID → forward
                pwm_duty  <= abs_out;       // 16-bit duty cycle magnitude
            end
        end
    end

endmodule

`default_nettype wire
