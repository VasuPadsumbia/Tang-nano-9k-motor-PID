// =============================================================================
// Module  : clk_div
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/clk_div.v
//
// Purpose : Generates a single-cycle sample_tick pulse at TICK_FREQ Hz from
//           a CLK_FREQ input clock.  Used to drive the PID computation loop
//           at a deterministic and configurable sample rate (default 1 kHz).
//
// Parameters
//   CLK_FREQ   – Frequency of 'clk' in Hz (default 50 MHz from LAN8720 REF_CLK)
//   TICK_FREQ  – Desired output pulse rate in Hz  (default 1 kHz)
//
// Interface
//   clk        – System clock (50 MHz)
//   rst        – Synchronous active-high reset
//   tick       – 1-cycle pulse every (CLK_FREQ / TICK_FREQ) clock cycles
//
// Timing    : Fully synchronous to clk.  tick is a registered output, so the
//             consumer can use it directly in a posedge clk always block.
// =============================================================================

`default_nettype none

module clk_div #(
    parameter integer CLK_FREQ  = 50_000_000,  // 50 MHz
    parameter integer TICK_FREQ = 1_000         // 1 kHz PID sample
)(
    input  wire clk,
    input  wire rst,
    output reg  tick
);

    // Calculate counter top value.  E.g. 50000000/1000 = 50000 counts per tick.
    localparam integer DIVIDE  = CLK_FREQ / TICK_FREQ;  // 50 000
    localparam integer CNT_W   = 16;                    // 2^16=65536 > 50000 ✓

    reg [CNT_W-1:0] cnt;

    always @(posedge clk) begin
        if (rst) begin
            cnt  <= {CNT_W{1'b0}};
            tick <= 1'b0;
        end else begin
            tick <= 1'b0;                         // default: no tick
            if (cnt == DIVIDE[CNT_W-1:0] - 1) begin
                cnt  <= {CNT_W{1'b0}};
                tick <= 1'b1;                     // one-cycle pulse
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
