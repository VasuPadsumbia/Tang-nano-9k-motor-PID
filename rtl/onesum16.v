module onesum16(
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [15:0] sum
);
    wire [32:0] s = {1'b0,a} + {1'b0,b};
    wire [16:0] hi = s[32:16];
    wire [16:0] lo = s[15:0];
    wire [16:0] t  = hi + lo;
    assign sum = ~(t[15:0] + t[16]);
endmodule
