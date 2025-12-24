module uart_baud_gen #(
    parameter integer CLK_FREQ_HZ = 27_000_000,
    parameter integer BAUD        = 115200,
    parameter integer OVERSAMPLE  = 16
)(
    input  wire clk,
    input  wire reset,
    output reg  tick  // 1-cycle pulse at BAUD*OVERSAMPLE
);
    // NCO: phase accumulator creates accurate average tick even when not divisible.
    localparam integer TICK_RATE = BAUD * OVERSAMPLE;

    // 32-bit accumulator
    reg [31:0] acc;

    // step = round(2^32 * TICK_RATE / CLK_FREQ)
    // We compute using integer math; add half divisor for rounding.
    localparam [31:0] STEP = ( (64'd4294967296 * TICK_RATE) + (CLK_FREQ_HZ/2) ) / CLK_FREQ_HZ;

    always @(posedge clk) begin
        if (reset) begin
            acc  <= 32'd0;
            tick <= 1'b0;
        end else begin
            {tick, acc} <= acc + STEP; // tick = carry out
        end
    end
endmodule
