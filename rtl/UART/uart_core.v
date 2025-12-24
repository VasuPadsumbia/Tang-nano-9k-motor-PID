module uart_core #(
    parameter integer CLK_FREQ_HZ = 27_000_000,
    parameter integer BAUD        = 115200
)(
    input  wire clk,
    input  wire reset,
    input  wire rx,
    output wire tx,

    // RX output
    output wire [7:0] rx_data,
    output wire       rx_valid,
    output wire       rx_busy,
    output wire       rx_framing_error,

    // TX input
    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output wire       tx_ready,
    output wire       tx_busy
);
    // 16x tick for RX
    wire tick16;
    uart_baud_gen #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD(BAUD),
        .OVERSAMPLE(16)
    ) gen16 (
        .clk(clk),
        .reset(reset),
        .tick(tick16)
    );

    // 1x tick for TX: derive from tick16 (every 16 pulses)
    reg [3:0] div16 = 0;
    reg tick1 = 0;
    always @(posedge clk) begin
        if (reset) begin
            div16 <= 0;
            tick1 <= 0;
        end else begin
            tick1 <= 0;
            if (tick16) begin
                if (div16 == 4'd15) begin
                    div16 <= 0;
                    tick1 <= 1;
                end else begin
                    div16 <= div16 + 1;
                end
            end
        end
    end

    uart_rx #(.OVERSAMPLE(16)) rx_i (
        .clk(clk),
        .reset(reset),
        .tick16(tick16),
        .rx(rx),
        .data(rx_data),
        .valid(rx_valid),
        .busy(rx_busy),
        .framing_error(rx_framing_error)
    );

    uart_tx tx_i (
        .clk(clk),
        .reset(reset),
        .tick(tick1),
        .tx(tx),
        .data(tx_data),
        .start(tx_start),
        .ready(tx_ready),
        .busy(tx_busy)
    );
endmodule
