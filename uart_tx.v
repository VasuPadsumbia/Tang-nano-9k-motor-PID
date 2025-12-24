module uart_tx(
    input  wire clk,
    input  wire reset,
    input  wire tick,        // BAUD tick (1x)
    output reg  tx,

    input  wire [7:0] data,
    input  wire       start, // pulse to start sending
    output reg        ready,
    output reg        busy
);
    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_idx;
    reg [7:0] shreg;

    always @(posedge clk) begin
        if (reset) begin
            state  <= IDLE;
            tx     <= 1'b1;
            ready  <= 1'b1;
            busy   <= 1'b0;
            bit_idx<= 3'd0;
            shreg  <= 8'd0;
        end else begin
            if (state == IDLE) begin
                tx    <= 1'b1;
                busy  <= 1'b0;
                ready <= 1'b1;

                if (start) begin
                    shreg   <= data;
                    state   <= START;
                    busy    <= 1'b1;
                    ready   <= 1'b0;
                    bit_idx <= 3'd0;
                end
            end

            if (tick) begin
                case (state)
                    START: begin
                        tx    <= 1'b0; // start bit
                        state <= DATA;
                    end

                    DATA: begin
                        tx    <= shreg[0];           // LSB first
                        shreg <= {1'b0, shreg[7:1]}; // shift right
                        if (bit_idx == 3'd7) begin
                            state <= STOP;
                        end
                        bit_idx <= bit_idx + 1;
                    end

                    STOP: begin
                        tx    <= 1'b1; // stop bit
                        state <= IDLE;
                    end
                endcase
            end
        end
    end
endmodule
