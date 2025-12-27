module eth_fcs32(
    input  wire       clk,
    input  wire       rst,
    input  wire       init,
    input  wire       en,
    input  wire [7:0] data,
    output reg  [31:0] crc
);
    integer k;
    reg [31:0] c;
    reg [7:0] d;

    always @(posedge clk) begin
        if (rst) begin
            crc <= 32'hFFFF_FFFF;
        end else begin
            if (init) crc <= 32'hFFFF_FFFF;
            else if (en) begin
                c = crc;
                d = data;
                for (k=0; k<8; k=k+1) begin
                    if ((c[0] ^ d[0]) == 1'b1)
                        c = (c >> 1) ^ 32'hEDB88320;
                    else
                        c = (c >> 1);
                    d = d >> 1;
                end
                crc <= c;
            end
        end
    end
endmodule
