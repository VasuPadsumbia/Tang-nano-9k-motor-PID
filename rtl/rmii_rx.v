module rmii_rx(
    input  wire       clk50,
    input  wire       rst,
    input  wire       crs_dv,
    input  wire [1:0] rxd,
    input  wire       rx_er,

    output reg        sof,
    output reg        eof,
    output reg        vld,
    output reg [7:0]  byte_out
);

    reg dv_d;

    // 2-bit -> 8-bit assembly
    reg [1:0] p2;
    reg [7:0] sh;
    reg [7:0] assembled;

    // preamble stripper
    localparam ST_IDLE = 2'd0;
    localparam ST_PREAM = 2'd1;
    localparam ST_DATA = 2'd2;
    reg [1:0] st;
    reg       sof_pending;

    always @(posedge clk50) begin
        if (rst) begin
            dv_d <= 1'b0;
            p2 <= 2'd0;
            sh <= 8'd0;

            sof <= 1'b0;
            eof <= 1'b0;
            vld <= 1'b0;
            byte_out <= 8'd0;

            st <= ST_IDLE;
            sof_pending <= 1'b0;
        end else begin
            sof <= 1'b0;
            eof <= 1'b0;
            vld <= 1'b0;

            dv_d <= crs_dv;

            if (!dv_d && crs_dv) begin
                // new frame begins (preamble starts)
                st <= ST_PREAM;
                p2 <= 2'd0;
                sof_pending <= 1'b0;
            end

            if (dv_d && !crs_dv) begin
                // end of frame
                eof <= 1'b1;
                st <= ST_IDLE;
                p2 <= 2'd0;
                sof_pending <= 1'b0;
            end

            if (crs_dv) begin
                // Assemble 1 byte from 4 cycles
                case (p2)
                    2'd0: begin sh[1:0] <= rxd; p2 <= 2'd1; end
                    2'd1: begin sh[3:2] <= rxd; p2 <= 2'd2; end
                    2'd2: begin sh[5:4] <= rxd; p2 <= 2'd3; end
                    2'd3: begin
                        sh[7:6] <= rxd;
                        p2 <= 2'd0;
                        assembled <= {rxd, sh[5:0]};

                        // Now process the assembled byte
                        if (st == ST_PREAM) begin
                            // ignore 0x55 bytes, wait for SFD 0xD5
                            if ({rxd, sh[5:0]} == 8'hD5) begin
                                st <= ST_DATA;
                                sof_pending <= 1'b1; // next DATA byte is sof
                            end
                        end else if (st == ST_DATA) begin
                            // output real frame bytes (dest mac starts here)
                            vld <= 1'b1;
                            byte_out <= {rxd, sh[5:0]};
                            if (sof_pending) begin
                                sof <= 1'b1;
                                sof_pending <= 1'b0;
                            end
                        end
                    end
                endcase
            end
        end
    end

endmodule
