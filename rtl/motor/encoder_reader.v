// =============================================================================
// Module  : encoder_reader
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/encoder_reader.v
//
// Purpose : Decodes a 2-channel quadrature (incremental) encoder into a
//           signed 32-bit position counter.  Supports all four gray-code
//           transitions (full 4x decoding).
//
//           A 2-stage synchroniser on each input provides protection against
//           metastability; the effective debounce is 2 clock cycles.
//
// Interface
//   clk       – System clock (50 MHz)
//   rst       – Synchronous active-high reset (position zeroed)
//   enc_a     – Encoder channel A (connect via 100 Ω series + 3.3 V Schmitt)
//   enc_b     – Encoder channel B
//   position  – Signed 32-bit position count (forward = +1, reverse = -1)
//   enc_dir   – Current direction: 1 = forward, 0 = reverse
//
// Wiring note: Use 100-ohm series resistors between encoder and FPGA pins to
//              limit inrush current.  If encoder is open-collector, add 10k
//              pull-up to 3V3.
// =============================================================================

`default_nettype none

module encoder_reader (
    input  wire        clk,
    input  wire        rst,
    input  wire        enc_a,
    input  wire        enc_b,
    output reg  [31:0] position,
    output reg         enc_dir     // 1 = forward (CW), 0 = reverse (CCW)
);

    // -------------------------------------------------------------------------
    // 2-stage synchroniser (metastability protection)
    // -------------------------------------------------------------------------
    reg [1:0] a_ff, b_ff;

    always @(posedge clk) begin
        a_ff <= {a_ff[0], enc_a};
        b_ff <= {b_ff[0], enc_b};
    end

    wire a_s = a_ff[1];
    wire b_s = b_ff[1];

    // -------------------------------------------------------------------------
    // 4-bit Gray-code state machine: {prev_a, prev_b, cur_a, cur_b}
    // Transition table (all forward and reverse transitions):
    //   Forward:  0001, 0111, 1110, 1000  → +1
    //   Reverse:  0010, 0100, 1101, 1011  → -1
    //   Any other transition = error / glitch, ignored
    // -------------------------------------------------------------------------
    reg [1:0] prev_ab;

    always @(posedge clk) begin
        if (rst) begin
            prev_ab  <= 2'b00;
            position <= 32'h0;
            enc_dir  <= 1'b0;
        end else begin
            prev_ab <= {a_s, b_s};

            case ({prev_ab, a_s, b_s})
                // Forward transitions
                4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                    position <= position + 1;
                    enc_dir  <= 1'b1;
                end
                // Reverse transitions
                4'b0010, 4'b0100, 4'b1101, 4'b1011: begin
                    position <= position - 1;
                    enc_dir  <= 1'b0;
                end
                // No change or glitch – do nothing
                default: ;
            endcase
        end
    end

endmodule

`default_nettype wire
