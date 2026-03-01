// =============================================================================
// Module  : rmii_tx
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/rmii_tx.v
//
// Purpose : RMII transmit path.  Accepts an Ethernet frame as a byte stream
//           (caller provides all bytes from dest-MAC through FCS), and
//           outputs the complete frame on RMII with preamble + SFD prepended.
//
//   TX sequence:
//     1. Preamble: 7× 0x55 = 28 dibits of 2'b01
//     2. SFD     : 0xD5    =  3 dibits of 2'b01 + 1 dibit of 2'b11
//     3. Data    : caller's bytes, 4 dibits each (LSB first)
//
// Interface
//   clk50      – 50 MHz RMII reference clock
//   rst        – Synchronous active-high reset
//   start      – Pulse HIGH for 1 cycle to begin transmission
//   tx_byte    – Byte to transmit (must be stable when tx_req is HIGH)
//   tx_req     – HIGH when module needs the NEXT byte (sample tx_byte on
//                the following rising edge)
//   tx_last    – Assert HIGH along with the last byte to indicate EoF
//   tx_busy    – HIGH while a frame is being transmitted
//   rmii_tx_en – RMII TX_EN pin
//   rmii_txd   – RMII TXD[1:0] pins
//
// Important: The caller is responsible for appending the CRC/FCS bytes.
//            Use eth_crc32 to calculate FCS, then send as final 4 bytes.
// =============================================================================

`default_nettype none

module rmii_tx (
    input  wire        clk50,
    input  wire        rst,

    // Frame source interface
    input  wire        start,
    input  wire [7:0]  tx_byte,
    output reg         tx_req,      // request next byte
    input  wire        tx_last,     // HIGH with last valid byte
    output reg         tx_busy,

    // RMII pins (to LAN8720)
    output reg         rmii_tx_en,
    output reg  [1:0]  rmii_txd
);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam S_IDLE     = 3'd0;
    localparam S_PREAMBLE = 3'd1;
    localparam S_SFD      = 3'd2;
    localparam S_DATA     = 3'd3;
    localparam S_DONE     = 3'd4;

    reg  [2:0] state;
    reg  [4:0] pre_cnt;    // preamble dibit counter (28 dibits = 5'd27)
    reg  [1:0] sfd_cnt;    // SFD dibit index (0..3)
    reg  [1:0] dcnt;       // dibit within current byte (0..3)
    reg  [7:0] shift;      // byte being shifted out
    reg         is_last;   // latched tx_last

    always @(posedge clk50) begin
        tx_req <= 1'b0;   // default de-assert

        if (rst) begin
            state      <= S_IDLE;
            pre_cnt    <= 5'd0;
            sfd_cnt    <= 2'd0;
            dcnt       <= 2'd0;
            shift      <= 8'd0;
            is_last    <= 1'b0;
            rmii_tx_en <= 1'b0;
            rmii_txd   <= 2'b00;
            tx_busy    <= 1'b0;
        end else begin
            case (state)

                // --------------------------------------------------------------
                // IDLE: wait for start pulse
                // --------------------------------------------------------------
                S_IDLE: begin
                    rmii_tx_en <= 1'b0;
                    rmii_txd   <= 2'b00;
                    tx_busy    <= 1'b0;
                    if (start) begin
                        state      <= S_PREAMBLE;
                        pre_cnt    <= 5'd0;
                        tx_busy    <= 1'b1;
                        rmii_tx_en <= 1'b1;
                    end
                end

                // --------------------------------------------------------------
                // PREAMBLE: transmit 28 × 2'b01 dibits (7 bytes of 0x55)
                // --------------------------------------------------------------
                S_PREAMBLE: begin
                    rmii_txd <= 2'b01;
                    pre_cnt  <= pre_cnt + 1'b1;
                    if (pre_cnt == 5'd27) begin
                        state   <= S_SFD;
                        sfd_cnt <= 2'd0;
                    end
                end

                // --------------------------------------------------------------
                // SFD: 0xD5 = dibits 01, 01, 01, 11  (LSB first)
                // --------------------------------------------------------------
                S_SFD: begin
                    rmii_txd <= (sfd_cnt == 2'd3) ? 2'b11 : 2'b01;
                    sfd_cnt  <= sfd_cnt + 1'b1;
                    if (sfd_cnt == 2'd3) begin
                        state   <= S_DATA;
                        dcnt    <= 2'd0;
                        shift   <= tx_byte;  // latch first byte
                        is_last <= tx_last;
                        tx_req  <= 1'b1;     // request second byte early
                    end
                end

                // --------------------------------------------------------------
                // DATA: shift out current byte, 2 bits per cycle (LSB first)
                // --------------------------------------------------------------
                S_DATA: begin
                    rmii_txd <= shift[1:0];
                    shift    <= {2'b00, shift[7:2]};   // right-shift by 2
                    dcnt     <= dcnt + 1'b1;

                    if (dcnt == 2'd3) begin
                        // Just sent the last dibit of this byte
                        if (is_last) begin
                            // Frame complete
                            state      <= S_DONE;
                        end else begin
                            // Load next byte (must be stable on tx_byte now)
                            shift   <= tx_byte;
                            is_last <= tx_last;
                            tx_req  <= 1'b1;   // request byte after next
                        end
                    end
                end

                // --------------------------------------------------------------
                // DONE: de-assert TX_EN, return to IDLE
                // --------------------------------------------------------------
                S_DONE: begin
                    rmii_tx_en <= 1'b0;
                    rmii_txd   <= 2'b00;
                    tx_busy    <= 1'b0;
                    state      <= S_IDLE;
                end

            endcase
        end
    end

endmodule

`default_nettype wire
