// =============================================================================
// Module  : eth_rx_parser
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/eth_rx_parser.v
//
// Purpose : Parses incoming Ethernet frames from the rmii_rx byte stream.
//           Decodes:
//             - ARP requests  (ethertype 0x0806)
//             - IPv4/UDP frames directed to LOCAL_IP:LOCAL_PORT (0x0800 / 0x11)
//           Any other frames are silently discarded.
//
//           Outputs:
//             arp_req    – Pulse: ARP who-has received for our IP
//             arp_src_mac, arp_src_ip – ARP sender fields (for reply)
//             udp_payload_vld – Byte valid for UDP payload
//             udp_payload_byte – Current UDP payload byte
//             udp_payload_idx  – Byte index within payload (0-based)
//             udp_payload_last – HIGH with last payload byte
//
// Parameters
//   LOCAL_MAC  – Our MAC address  (48-bit)
//   LOCAL_IP   – Our IPv4 address (32-bit)
//   LOCAL_PORT – Our UDP control port (16-bit, default 5005)
//
// Notes
//   - IP options are NOT supported (IHL must be 5 = 20-byte header)
//   - UDP checksum is NOT verified (set to 0 by PC sender for simplicity)
//   - Frame FCS is NOT checked here (add eth_crc32 separately if needed)
// =============================================================================

`default_nettype none

module eth_rx_parser #(
    parameter [47:0] LOCAL_MAC  = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP   = {8'd10, 8'd10, 8'd10, 8'd100},
    parameter [15:0] LOCAL_PORT = 16'd5005
)(
    input  wire        clk,
    input  wire        rst,

    // From rmii_rx
    input  wire        rx_sof,
    input  wire        rx_eof,
    input  wire        rx_vld,
    input  wire [7:0]  rx_byte,

    // ARP outputs
    output reg         arp_req,       // 1-cycle pulse when ARP for us
    output reg  [47:0] arp_src_mac,
    output reg  [31:0] arp_src_ip,

    // UDP payload outputs
    output reg         udp_payload_vld,
    output reg  [7:0]  udp_payload_byte,
    output reg  [10:0] udp_payload_idx,
    output reg         udp_payload_last
);

    // -------------------------------------------------------------------------
    // State machine: walks through byte offsets in the Ethernet frame
    // -------------------------------------------------------------------------
    localparam [5:0]
        S_ETH_DST   = 6'd0,     // bytes  0-5   : destination MAC (6 bytes)
        S_ETH_SRC   = 6'd6,     // bytes  6-11  : source MAC     (6 bytes)
        S_ETH_TYPE  = 6'd12,    // bytes 12-13  : EtherType
        S_ARP_BODY  = 6'd14,    // bytes 14+    : ARP payload
        S_IP_HDR    = 6'd20,    // bytes 14-33  : IPv4 header (20 bytes)
        S_UDP_HDR   = 6'd34,    // bytes 34-41  : UDP header  (8 bytes)
        S_UDP_PAY   = 6'd42,    // bytes 42+    : UDP payload
        S_DISCARD   = 6'd63;    // absorb remainder of unwanted frames

    // Frame type discriminator
    localparam T_NONE = 2'd0;
    localparam T_ARP  = 2'd1;
    localparam T_UDP  = 2'd2;

    reg [5:0]  byte_cnt;       // byte offset within frame
    reg [1:0]  frame_type;
    reg        for_us_mac;     // destination MAC matches ours (or broadcast)
    reg        ip_ok;          // IPv4, no options, UDP protocol, dst=us
    reg        port_ok;        // dst port matches LOCAL_PORT

    // Shift registers to collect multi-byte fields
    reg [47:0] arp_sha_buf;    // ARP sender hardware address
    reg [31:0] arp_spa_buf;    // ARP sender protocol address
    reg [15:0] eth_type_buf;
    reg [15:0] ip_proto_buf;   // holds {IHL, protocol} fields
    reg [31:0] ip_dst_buf;
    reg [15:0] udp_dst_port_buf;
    reg [15:0] udp_len_buf;
    reg [10:0] payload_cnt;

    // Convenience alias
    wire [5:0] bc = byte_cnt;

    always @(posedge clk) begin
        // Default de-assert every cycle
        arp_req          <= 1'b0;
        udp_payload_vld  <= 1'b0;
        udp_payload_last <= 1'b0;

        if (rst) begin
            byte_cnt        <= 6'd0;
            frame_type      <= T_NONE;
            for_us_mac      <= 1'b0;
            ip_ok           <= 1'b0;
            port_ok         <= 1'b0;
            payload_cnt     <= 11'd0;
            arp_src_mac     <= 48'd0;
            arp_src_ip      <= 32'd0;
        end else begin

            // -----------------------------------------------------------------
            // Start of frame: reset all state
            // -----------------------------------------------------------------
            if (rx_sof) begin
                byte_cnt   <= 6'd0;
                frame_type <= T_NONE;
                for_us_mac <= 1'b0;
                ip_ok      <= 1'b0;
                port_ok    <= 1'b0;
                payload_cnt<= 11'd0;
            end

            // -----------------------------------------------------------------
            // End of frame: flush any pending outputs
            // -----------------------------------------------------------------
            if (rx_eof) begin
                byte_cnt <= 6'd0;
                if (frame_type == T_ARP && for_us_mac) begin
                    arp_req     <= 1'b1;
                    arp_src_mac <= arp_sha_buf;
                    arp_src_ip  <= arp_spa_buf;
                end
            end

            // -----------------------------------------------------------------
            // Process incoming byte
            // -----------------------------------------------------------------
            if (rx_vld) begin
                byte_cnt <= byte_cnt + 1'b1;

                // --- Ethernet destination MAC (bytes 0-5) --------------------
                if (bc < 6) begin
                    // Accept broadcast (FF:FF:FF:FF:FF:FF) or our MAC
                    case (bc)
                        6'd0: for_us_mac <= (rx_byte == LOCAL_MAC[47:40]) || (rx_byte == 8'hFF);
                        6'd1: for_us_mac <= for_us_mac && ((rx_byte == LOCAL_MAC[39:32]) || (rx_byte == 8'hFF));
                        6'd2: for_us_mac <= for_us_mac && ((rx_byte == LOCAL_MAC[31:24]) || (rx_byte == 8'hFF));
                        6'd3: for_us_mac <= for_us_mac && ((rx_byte == LOCAL_MAC[23:16]) || (rx_byte == 8'hFF));
                        6'd4: for_us_mac <= for_us_mac && ((rx_byte == LOCAL_MAC[15:8])  || (rx_byte == 8'hFF));
                        6'd5: for_us_mac <= for_us_mac && ((rx_byte == LOCAL_MAC[7:0])   || (rx_byte == 8'hFF));
                        default: ;
                    endcase
                end

                // --- EtherType (bytes 12-13) ---------------------------------
                if (bc == 6'd12) eth_type_buf[15:8] <= rx_byte;
                if (bc == 6'd13) begin
                    eth_type_buf[7:0] <= rx_byte;
                    if ({eth_type_buf[15:8], rx_byte} == 16'h0806)
                        frame_type <= T_ARP;
                    else if ({eth_type_buf[15:8], rx_byte} == 16'h0800)
                        frame_type <= T_NONE; // will confirm after IP hdr
                end

                // --- ARP payload (bytes 14–41 for standard ARP) --------------
                // We only need: SHA (bytes 22-27), SPA (bytes 28-31),
                // TPA (bytes 38-41 – to verify it's for us)
                // ARP byte offsets from frame start:
                //   14-15: HTYPE, 16-17: PTYPE, 18: HLEN, 19: PLEN
                //   20-21: OPER  (1=request, 2=reply)
                //   22-27: SHA, 28-31: SPA, 32-37: THA, 38-41: TPA
                if (frame_type == T_ARP) begin
                    case (bc)
                        6'd22: arp_sha_buf[47:40] <= rx_byte;
                        6'd23: arp_sha_buf[39:32] <= rx_byte;
                        6'd24: arp_sha_buf[31:24] <= rx_byte;
                        6'd25: arp_sha_buf[23:16] <= rx_byte;
                        6'd26: arp_sha_buf[15:8]  <= rx_byte;
                        6'd27: arp_sha_buf[7:0]   <= rx_byte;
                        6'd28: arp_spa_buf[31:24]  <= rx_byte;
                        6'd29: arp_spa_buf[23:16]  <= rx_byte;
                        6'd30: arp_spa_buf[15:8]   <= rx_byte;
                        6'd31: arp_spa_buf[7:0]    <= rx_byte;
                        // TPA check (bytes 38-41): verify request is for us
                        6'd38: for_us_mac <= for_us_mac && (rx_byte == LOCAL_IP[31:24]);
                        6'd39: for_us_mac <= for_us_mac && (rx_byte == LOCAL_IP[23:16]);
                        6'd40: for_us_mac <= for_us_mac && (rx_byte == LOCAL_IP[15:8]);
                        6'd41: for_us_mac <= for_us_mac && (rx_byte == LOCAL_IP[7:0]);
                        default: ;
                    endcase
                end

                // --- IPv4 Header (bytes 14-33, IHL must be 20 bytes) ---------
                if (eth_type_buf == 16'h0800) begin
                    case (bc)
                        // Byte 14: Version(4) + IHL(4) → must be 0x45
                        6'd14: ip_ok <= (rx_byte == 8'h45);
                        // Byte 23: Protocol → must be 0x11 (UDP)
                        6'd23: ip_ok <= ip_ok && (rx_byte == 8'h11);
                        // Bytes 30-33: Destination IP
                        6'd30: ip_ok <= ip_ok && (rx_byte == LOCAL_IP[31:24]);
                        6'd31: ip_ok <= ip_ok && (rx_byte == LOCAL_IP[23:16]);
                        6'd32: ip_ok <= ip_ok && (rx_byte == LOCAL_IP[15:8]);
                        6'd33: begin
                            ip_ok <= ip_ok && (rx_byte == LOCAL_IP[7:0]);
                            if (ip_ok && (rx_byte == LOCAL_IP[7:0]))
                                frame_type <= T_UDP;  // confirmed IPv4/UDP to us
                        end
                        default: ;
                    endcase
                end

                // --- UDP Header (bytes 34-41) ---------------------------------
                if (frame_type == T_UDP) begin
                    case (bc)
                        6'd34: udp_dst_port_buf[15:8] <= rx_byte;
                        6'd35: begin
                            udp_dst_port_buf[7:0] <= rx_byte;
                            port_ok <= ({udp_dst_port_buf[15:8], rx_byte} == LOCAL_PORT);
                        end
                        6'd38: udp_len_buf[15:8] <= rx_byte;
                        6'd39: udp_len_buf[7:0]  <= rx_byte;
                        default: ;
                    endcase
                end

                // --- UDP Payload (bytes 42+) ----------------------------------
                if (frame_type == T_UDP && port_ok && bc >= 6'd42) begin
                    // UDP payload length = udp_len - 8 (UDP header)
                    // To avoid divide, we track and compare payload_cnt
                    udp_payload_vld  <= 1'b1;
                    udp_payload_byte <= rx_byte;
                    udp_payload_idx  <= payload_cnt[10:0];
                    payload_cnt      <= payload_cnt + 1'b1;
                    // Mark last payload byte on EOF (rx_eof handled above)
                    if (rx_eof)
                        udp_payload_last <= 1'b1;
                end

            end // rx_vld
        end
    end

endmodule

`default_nettype wire
