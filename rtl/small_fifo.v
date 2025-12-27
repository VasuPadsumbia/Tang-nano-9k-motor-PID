module small_fifo #(
    parameter DEPTH = 512,
    parameter AW = 9
)(
    input  wire clk,
    input  wire rst,

    input  wire wr_en,
    input  wire [7:0] wr_data,
    output wire full,

    input  wire rd_en,
    output reg  [7:0] rd_data,
    output wire empty,

    output reg  [AW:0] count
);
    reg [7:0] mem [0:DEPTH-1];
    reg [AW-1:0] wptr, rptr;

    assign full  = (count == DEPTH);
    assign empty = (count == 0);

    always @(posedge clk) begin
        if (rst) begin
            wptr <= 0; rptr <= 0; count <= 0; rd_data <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[wptr] <= wr_data;
                wptr <= wptr + 1'b1;
                count <= count + 1'b1;
            end
            if (rd_en && !empty) begin
                rd_data <= mem[rptr];
                rptr <= rptr + 1'b1;
                count <= count - 1'b1;
            end
        end
    end
endmodule
