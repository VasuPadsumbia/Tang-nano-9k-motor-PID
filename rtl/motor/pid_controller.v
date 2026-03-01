// =============================================================================
// Module  : pid_controller
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/pid_controller.v
//
// Purpose : Fixed-point discrete PID controller.
//
//   Output = Kp*e + Ki*∑e + Kd*(e - e_prev)       [per sample tick]
//
// Fixed-point format (8.8): gain_register / 256 = actual real gain
//   Example: kp = 16'h0100 (256) → Kp_real = 1.0
//            ki = 16'h0020 (32)  → Ki_real = 0.125
//            kd = 16'h0080 (128) → Kd_real = 0.5
//
// Anti-windup: integral accumulator is clamped to ±ICLAMP to prevent
//              saturation from very long step errors.
//
// Output saturation: signed 16-bit, i.e. ±32767.
//
// Parameters
//   ICLAMP  – Integral windup clamp (32-bit signed magnitude, default ±10M)
//
// Interface
//   clk         – System clock
//   rst         – Synchronous active-high reset
//   sample_tick – 1-cycle pulse that triggers one PID computation
//   enable      – PID active when HIGH; output holds when LOW
//   kp/ki/kd    – Gains (8.8 fixed-point, unsigned 16-bit)
//   setpoint    – Target value (signed 16-bit)
//   feedback    – Measured value (signed 16-bit, e.g. encoder position>>16)
//   pid_out     – PID output (signed 16-bit, drives motor_bridge)
//   error_out   – Current error for telemetry (signed 16-bit)
//   integral_dbg– Raw integral accumulator for debug/telemetry
// =============================================================================

`default_nettype none

module pid_controller #(
    parameter signed [31:0] ICLAMP = 32'sh00989680  // ±10 000 000 (prevents windup)
)(
    input  wire              clk,
    input  wire              rst,
    input  wire              sample_tick,
    input  wire              enable,

    input  wire [15:0]       kp,
    input  wire [15:0]       ki,
    input  wire [15:0]       kd,

    input  wire signed [15:0] setpoint,
    input  wire signed [15:0] feedback,

    output reg  signed [15:0] pid_out,
    output reg  signed [15:0] error_out,
    output wire signed [31:0] integral_dbg
);

    // -------------------------------------------------------------------------
    // Internal registers
    // -------------------------------------------------------------------------
    reg  signed [31:0] integrator;
    reg  signed [15:0] prev_error;

    assign integral_dbg = integrator;

    // -------------------------------------------------------------------------
    // Combinatorial terms (computed every cycle, registered on sample_tick)
    // -------------------------------------------------------------------------
    wire signed [15:0] error      = setpoint - feedback;
    wire signed [31:0] p_term     = ($signed({1'b0, kp}) * error) >>> 8;
    wire signed [31:0] d_term     = ($signed({1'b0, kd}) * (error - prev_error)) >>> 8;
    wire signed [31:0] i_contrib  = integrator >>> 8;

    // Raw sum before saturation (33-bit to catch overflow)
    wire signed [33:0] raw_sum    = $signed(p_term) + $signed(i_contrib) + $signed(d_term);

    // Next integrator value (for anti-windup logic)
    wire signed [31:0] i_next     = integrator + ($signed({1'b0, ki}) * error);

    // -------------------------------------------------------------------------
    // Sequential logic
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            integrator <= 32'sd0;
            prev_error <= 16'sd0;
            pid_out    <= 16'sd0;
            error_out  <= 16'sd0;
        end else if (sample_tick && enable) begin

            // -- Error output (for telemetry) ---------------------------------
            error_out <= error;

            // -- Anti-windup integral clamp -----------------------------------
            if ($signed(i_next) > $signed(ICLAMP))
                integrator <= $signed(ICLAMP);
            else if ($signed(i_next) < $signed(-ICLAMP))
                integrator <= $signed(-ICLAMP);
            else
                integrator <= i_next;

            // -- Output saturation to ±32767 ----------------------------------
            if ($signed(raw_sum) > 34'sh0000000_7FFF)
                pid_out <= 16'sh7FFF;
            else if ($signed(raw_sum) < 34'shFFFFFF_8000)
                pid_out <= 16'sh8000;
            else
                pid_out <= raw_sum[15:0];

            // -- Store previous error for derivative --------------------------
            prev_error <= error;
        end
    end

endmodule

`default_nettype wire
