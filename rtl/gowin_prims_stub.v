// Simple Verilator stub for Gowin primitives.
// For lint/sim only; synthesis will use the real primitive.

module GOWIN_CLKDIV (
  input  wire hclkin,
  input  wire resetn,
  output wire clkout
);
  assign clkout = hclkin;
endmodule
