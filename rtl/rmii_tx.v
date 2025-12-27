module rmii_tx(
    input  wire        clk50,
    input  wire        rst,
    input  wire        start,
    input  wire [15:0] length,
    input  wire [7:0]  data_in,
    output reg  [15:0] rd_idx,

    output reg         tx_en,
    output reg  [1:0]  txd,
    output reg         busy,
    output reg         done
);
    reg [1:0] pair;
    reg [15:0] left;
    reg sending;

    always @(posedge clk50) begin
        if (rst) begin
            tx_en<=0; txd<=0; busy<=0; done<=0;
            rd_idx<=0; pair<=0; left<=0; sending<=0;
        end else begin
            done <= 1'b0;
            if (!sending) begin
                tx_en  <= 1'b0;
                txd    <= 2'b00;
                rd_idx <= 16'd0;
                pair   <= 2'd0;
                if (start) begin
                    sending <= 1'b1;
                    busy    <= 1'b1;
                    tx_en   <= 1'b1;
                    left    <= length;
                end
            end else begin
                txd <= data_in[pair*2 +: 2];
                if (pair == 2'd3) begin
                    pair <= 2'd0;
                    if (left == 16'd1) begin
                        sending <= 1'b0;
                        busy    <= 1'b0;
                        tx_en   <= 1'b0;
                        done    <= 1'b1;
                        rd_idx  <= 16'd0;
                    end else begin
                        left   <= left - 16'd1;
                        rd_idx <= rd_idx + 16'd1;
                    end
                end else begin
                    pair <= pair + 2'd1;
                end
            end
        end
    end
endmodule
