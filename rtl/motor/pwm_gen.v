// =============================================================================
// Module  : pwm_gen
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/pwm_gen.v
//
// Purpose : Generates a PWM output signal with 16-bit duty-cycle resolution.
//           A free-running counter (0..65535) is compared against 'duty'.
//           When duty=0 output is always LOW; duty=65535 is always HIGH.
//
//           PWM Frequency = CLK_FREQ / 65536
//   At 50 MHz  → ~762 Hz  (suitable for DC motor H-bridge control)
//   At 27 MHz  → ~412 Hz
//
// Parameters
//   CLK_FREQ  – Only informational, not synthesised into logic here.
//
// Interface
//   clk       – System clock
//   rst       – Synchronous active-high reset (output LOW, counter zeroed)
//   duty      – 16-bit duty cycle  (0=0 %, 65535=100 %)
//   pwm_out   – PWM output pin (connect to H-bridge PWM input)
// =============================================================================

`default_nettype none

module pwm_gen (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] duty,     // 0 = 0 %, 65535 ≈ 100 %
    output reg         pwm_out
);

    reg [15:0] cnt;

    always @(posedge clk) begin
        if (rst) begin
            cnt     <= 16'h0000;
            pwm_out <= 1'b0;
        end else begin
            cnt     <= cnt + 1'b1;       // free-running 16-bit counter
            pwm_out <= (cnt < duty);     // output HIGH while counter < duty
        end
    end

endmodule

`default_nettype wire
