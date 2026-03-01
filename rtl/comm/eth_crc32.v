// =============================================================================
// Module  : eth_crc32
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/eth_crc32.v
//
// Purpose : Ethernet CRC-32 (FCS) calculator.  Processes one byte per clock
//           using the standard CRC-32 polynomial 0xEDB88320 (bit-reversed).
//
//   Usage (TX path):
//     1. Assert init=1 for one cycle to reset CRC to 0xFFFFFFFF.
//     2. Feed each frame byte (Eth header through payload) with vld=1.
//     3. After the last payload byte, read fcs[7:0], fcs[15:8],
//        fcs[23:16], fcs[31:24] and transmit those 4 bytes.
//
//   Usage (RX path):
//     Feed all frame bytes including the received FCS; a valid frame produces
//     crc == 32'hDEBB20E3 (Ethernet residue constant).
//
// Interface
//   clk    – System clock
//   rst    – Synchronous reset
//   init   – Reinitialise CRC to 0xFFFFFFFF (can pulse at start of frame)
//   vld    – Input byte strobe
//   data   – Input byte
//   crc    – Running CRC residue
//   fcs    – Inverted CRC ready to transmit (= ~crc, the Ethernet FCS)
// =============================================================================

`default_nettype none

module eth_crc32 (
    input  wire        clk,
    input  wire        rst,
    input  wire        init,
    input  wire        vld,
    input  wire [7:0]  data,
    output reg  [31:0] crc,
    output wire [31:0] fcs
);

    // -------------------------------------------------------------------------
    // Byte-by-byte CRC update function (unrolled loop → pure combinatorial)
    // Polynomial: 0xEDB88320  (reversed representation of 0x04C11DB7)
    // -------------------------------------------------------------------------
    function [31:0] crc32_next;
        input [31:0] crc_in;
        input [7:0]  byte_in;
        reg   [31:0] c;
        integer i;
        begin
            c = crc_in;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0] ^ byte_in[i])
                    c = (c >> 1) ^ 32'hEDB88320;
                else
                    c = c >> 1;
            end
            crc32_next = c;
        end
    endfunction

    always @(posedge clk) begin
        if (rst || init)
            crc <= 32'hFFFF_FFFF;
        else if (vld)
            crc <= crc32_next(crc, data);
    end

    // FCS = invert CRC.  Transmitted LSB-first per byte in wire order.
    assign fcs = ~crc;

endmodule

`default_nettype wire
