module uart_rx #(
    parameter integer OVERSAMPLE = 16
)(
    input  wire clk,
    input  wire reset,
    input  wire tick16,   // BAUD*16 tick
    input  wire rx,

    output reg  [7:0] data,
    output reg        valid,    // 1 clk pulse when a byte received
    output reg        busy,
    output reg        framing_error
);
    // Synchronize RX to clk
    reg rx_ff1, rx_ff2;
    always @(posedge clk) begin
        rx_ff1 <= rx;
        rx_ff2 <= rx_ff1;
    end
    wire rx_s = rx_ff2;

    reg [3:0] sub;       // 0..15
    reg [2:0] bit_idx;   // 0..7
    reg [7:0] shreg;

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;
    reg [1:0] state;

    always @(posedge clk) begin
        if (reset) begin
            state         <= IDLE;
            sub           <= 4'd0;
            bit_idx       <= 3'd0;
            shreg         <= 8'd0;
            data          <= 8'd0;
            valid         <= 1'b0;
            busy          <= 1'b0;
            framing_error <= 1'b0;
        end else begin
            valid <= 1'b0;

            if (tick16) begin
                case (state)
                    IDLE: begin
                        busy <= 1'b0;
                        framing_error <= 1'b0;
                        if (rx_s == 1'b0) begin
                            // potential start bit
                            state <= START;
                            sub   <= 4'd0;
                            busy  <= 1'b1;
                        end
                    end

                    START: begin
                        // sample middle of start bit at sub==7
                        sub <= sub + 1;
                        if (sub == 4'd7) begin
                            if (rx_s == 1'b0) begin
                                // confirmed start
                                state   <= DATA;
                                sub     <= 4'd0;
                                bit_idx <= 3'd0;
                            end else begin
                                // false start
                                state <= IDLE;
                            end
                        end
                    end

                    DATA: begin
                        sub <= sub + 1;
                        if (sub == 4'd15) begin
                            // sample at end of bit period (centered due to start alignment)
                            shreg <= {rx_s, shreg[7:1]}; // LSB first -> shift right
                            sub   <= 4'd0;

                            if (bit_idx == 3'd7) begin
                                state <= STOP;
                            end
                            bit_idx <= bit_idx + 1;
                        end
                    end

                    STOP: begin
                        sub <= sub + 1;
                        if (sub == 4'd15) begin
                            sub <= 4'd0;
                            // stop bit should be 1
                            if (rx_s == 1'b1) begin
                                data  <= shreg;
                                valid <= 1'b1;
                            end else begin
                                framing_error <= 1'b1;
                            end
                            state <= IDLE;
                        end
                    end
                endcase
            end
        end
    end
endmodule
