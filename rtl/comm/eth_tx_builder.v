// =============================================================================
// Module  : eth_tx_builder
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/eth_tx_builder.v
//
// Purpose : Builds a complete Ethernet + IPv4 + UDP frame from a payload
//           buffer supplied by the caller (arp_handler or udp_telem_tx).
//           Automatically:
//             - Prepends the 14-byte Ethernet header
//             - Prepends the 20-byte IPv4 header (no options)
//             - Prepends the  8-byte UDP header
//             - Feeds all bytes through eth_crc32 and appends 4-byte FCS
//             - Streams resulting bytes to rmii_tx
//
//   ARP frames bypass the IP/UDP header wrapper (use ETH_ONLY mode).
//
// Interface
//   clk/rst          – Clock and reset
//   -- Payload source (one of: arp_handler or udp_telem_tx) --
//   src_sel          – 0=ARP (raw Eth frame), 1=UDP payload
//   payload_req      – Pulse to begin building & sending a frame
//   payload_busy     – HIGH while building (source must hold buf_byte stable)
//   arp_buf_idx      – Index driven into arp_handler.buf_idx
//   arp_buf_byte     – Byte from arp_handler buffer
//   arp_buf_len      – Frame length from arp_handler
//   pay_buf_idx      – Index into udp_telem_tx payload buffer
//   pay_buf_byte     – Byte from udp_telem_tx
//   pay_buf_len      – Payload byte count from udp_telem_tx
//   dst_mac          – Ethernet destination MAC (for UDP path)
//   dst_ip           – IPv4 destination address
//   dst_port         – UDP destination port
//   local_mac        – Our MAC
//   local_ip         – Our IP
//   local_port       – Our source UDP port
//   -- To rmii_tx --
//   tx_start         – Pulse to start RMII TX of assembled frame
//   tx_byte          – Frame byte streamed out
//   tx_req_in        – rmii_tx requests next byte
//   tx_last          – HIGH with last byte
//   tx_busy_in       – rmii_tx busy flag
// =============================================================================

`default_nettype none

module eth_tx_builder #(
    parameter [47:0] LOCAL_MAC   = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP    = {8'd10, 8'd10, 8'd10, 8'd100},
    parameter [15:0] LOCAL_PORT  = 16'd5005,
    parameter [7:0]  TTL         = 8'd64,
    parameter integer MAX_PAYLOAD = 64
)(
    input  wire        clk,
    input  wire        rst,

    // Arbiter: which source wants to TX?
    input  wire        arp_tx_req,    // from arp_handler
    input  wire        udp_tx_req,    // from udp_telem_tx
    output reg         tx_busy,       // to both sources

    // ARP frame buffer (pre-built full Eth frame)
    output reg  [5:0]  arp_buf_idx,
    input  wire [7:0]  arp_buf_byte,
    input  wire [5:0]  arp_buf_len,

    // UDP payload buffer
    output reg  [4:0]  pay_buf_idx,
    input  wire [7:0]  pay_buf_byte,
    input  wire [4:0]  pay_buf_len,

    // UDP destination (from udp_ctrl_rx, learned from incoming command)
    input  wire [47:0] dst_mac,
    input  wire [31:0] dst_ip,
    input  wire [15:0] dst_port,

    // To rmii_tx
    output reg         rmii_start,
    output reg  [7:0]  rmii_byte,
    input  wire        rmii_req,
    output reg         rmii_last,
    input  wire        rmii_busy
);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam S_IDLE    = 3'd0;
    localparam S_ARP     = 3'd1;
    localparam S_ETH_HDR = 3'd2;   // Ethernet header (14 bytes)
    localparam S_IP_HDR  = 3'd3;   // IP header      (20 bytes)
    localparam S_UDP_HDR = 3'd4;   // UDP header     ( 8 bytes)
    localparam S_UDP_PAY = 3'd5;   // UDP payload
    localparam S_FCS     = 3'd6;   // Append 4 FCS bytes

    reg [2:0]  state;
    reg [5:0]  hdr_cnt;    // byte counter within current header
    reg [4:0]  pay_cnt;    // payload byte counter
    reg [1:0]  fcs_cnt;    // FCS byte counter

    // Latched destination for this frame
    reg [47:0] l_dst_mac;
    reg [31:0] l_dst_ip;
    reg [15:0] l_dst_port;
    reg [15:0] l_pay_len;   // payload byte count for this frame

    // CRC32 interface
    reg         crc_init;
    reg         crc_vld;
    reg  [7:0]  crc_data;
    wire [31:0] crc_fcs;

    eth_crc32 u_crc (
        .clk  (clk),
        .rst  (rst),
        .init (crc_init),
        .vld  (crc_vld),
        .data (crc_data),
        .crc  (),           // unused internal
        .fcs  (crc_fcs)
    );

    // IP total length = 20 (IP hdr) + 8 (UDP hdr) + payload
    wire [15:0] ip_total_len = 16'd28 + {11'd0, l_pay_len};
    // UDP length = 8 (UDP hdr) + payload
    wire [15:0] udp_len      = 16'd8  + {11'd0, l_pay_len};

    // Helper: IP header checksum (ones-complement sum of IP header words)
    // Simplified: set to 0x0000 (TX path will compute, or use software trick)
    // For correct operation a real IP checksum is required.
    // We compute it here as a 32-bit accumulation then fold the carry.
    // IP header (20 bytes = 10 words) with checksum field = 0:
    //   word0: 0x4500
    //   word1: ip_total_len
    //   word2: 0x0000 (ID)
    //   word3: 0x4000 (flags/offset: don't-fragment)
    //   word4: { TTL, 0x11 (UDP) }
    //   word5: 0x0000 (checksum placeholder)
    //   word6-7: LOCAL_IP
    //   word8-9: dst_ip
    wire [31:0] cksum_acc =
        32'h0000_4500 +
        {16'd0, ip_total_len} +
        32'h0000_0000 +   // ID
        32'h0000_4000 +   // DF flag
        {24'd0, TTL, 8'h11} +
        32'h0000_0000 +   // checksum field = 0
        {16'd0, LOCAL_IP[31:16]} +
        {16'd0, LOCAL_IP[15:0]} +
        {16'd0, l_dst_ip[31:16]} +
        {16'd0, l_dst_ip[15:0]};

    // Fold 32→16 (add carries)
    wire [16:0] cksum_fold = {1'b0, cksum_acc[31:16]} + {1'b0, cksum_acc[15:0]};
    wire [15:0] ip_checksum = ~(cksum_fold[16] ? cksum_fold[15:0] + 1 : cksum_fold[15:0]);

    // -------------------------------------------------------------------------
    // Internal byte output register
    // -------------------------------------------------------------------------
    reg [7:0] out_byte;
    reg       out_last;

    // Send a byte: feed to CRC and to rmii
    task send_byte;
        input [7:0] b;
        input       last;
        begin
            out_byte  = b;
            out_last  = last;
            crc_data  = b;
            crc_vld   = 1'b1;
        end
    endtask

    always @(posedge clk) begin
        crc_init   <= 1'b0;
        crc_vld    <= 1'b0;
        rmii_start <= 1'b0;
        rmii_last  <= 1'b0;

        if (rst) begin
            state      <= S_IDLE;
            tx_busy    <= 1'b0;
            hdr_cnt    <= 6'd0;
            pay_cnt    <= 5'd0;
            fcs_cnt    <= 2'd0;
            arp_buf_idx<= 6'd0;
            pay_buf_idx<= 5'd0;
        end else begin
            case (state)

                // --------------------------------------------------------------
                // Arbitrate: ARP has priority over telemetry
                // --------------------------------------------------------------
                S_IDLE: begin
                    tx_busy <= 1'b0;
                    if (!rmii_busy) begin
                        if (arp_tx_req) begin
                            state       <= S_ARP;
                            tx_busy     <= 1'b1;
                            arp_buf_idx <= 6'd0;
                            crc_init    <= 1'b1;
                        end else if (udp_tx_req) begin
                            state        <= S_ETH_HDR;
                            tx_busy      <= 1'b1;
                            hdr_cnt      <= 6'd0;
                            l_dst_mac    <= dst_mac;
                            l_dst_ip     <= dst_ip;
                            l_dst_port   <= dst_port;
                            l_pay_len    <= {11'd0, pay_buf_len};
                            pay_buf_idx  <= 5'd0;
                            crc_init     <= 1'b1;
                        end
                    end
                end

                // --------------------------------------------------------------
                // ARP: stream pre-built frame buffer through RMII TX + CRC
                // --------------------------------------------------------------
                S_ARP: begin
                    if (rmii_req || hdr_cnt == 0) begin
                        rmii_byte   <= arp_buf_byte;
                        crc_vld     <= 1'b1;
                        crc_data    <= arp_buf_byte;
                        if (hdr_cnt == 0)
                            rmii_start <= 1'b1;

                        if (arp_buf_idx == arp_buf_len - 1) begin
                            rmii_last   <= 1'b0;  // still FCS to append
                            state       <= S_FCS;
                            fcs_cnt     <= 2'd0;
                        end else begin
                            arp_buf_idx <= arp_buf_idx + 1'b1;
                            hdr_cnt     <= hdr_cnt + 1'b1;
                        end
                    end
                end

                // --------------------------------------------------------------
                // ETH Header (14 bytes)
                // Dst MAC[0-5], Src MAC[6-11], EtherType[12-13] = 0x0800
                // --------------------------------------------------------------
                S_ETH_HDR: begin
                    if (rmii_req || hdr_cnt == 0) begin
                        if (hdr_cnt == 0) rmii_start <= 1'b1;
                        case (hdr_cnt)
                            6'd0:  rmii_byte <= l_dst_mac[47:40];
                            6'd1:  rmii_byte <= l_dst_mac[39:32];
                            6'd2:  rmii_byte <= l_dst_mac[31:24];
                            6'd3:  rmii_byte <= l_dst_mac[23:16];
                            6'd4:  rmii_byte <= l_dst_mac[15:8];
                            6'd5:  rmii_byte <= l_dst_mac[7:0];
                            6'd6:  rmii_byte <= LOCAL_MAC[47:40];
                            6'd7:  rmii_byte <= LOCAL_MAC[39:32];
                            6'd8:  rmii_byte <= LOCAL_MAC[31:24];
                            6'd9:  rmii_byte <= LOCAL_MAC[23:16];
                            6'd10: rmii_byte <= LOCAL_MAC[15:8];
                            6'd11: rmii_byte <= LOCAL_MAC[7:0];
                            6'd12: rmii_byte <= 8'h08;
                            6'd13: rmii_byte <= 8'h00;
                            default: rmii_byte <= 8'h00;
                        endcase
                        crc_vld  <= 1'b1;
                        crc_data <= rmii_byte;
                        hdr_cnt  <= hdr_cnt + 1'b1;
                        if (hdr_cnt == 6'd13) begin
                            state   <= S_IP_HDR;
                            hdr_cnt <= 6'd0;
                        end
                    end
                end

                // --------------------------------------------------------------
                // IP Header (20 bytes)
                // --------------------------------------------------------------
                S_IP_HDR: begin
                    if (rmii_req) begin
                        case (hdr_cnt)
                            6'd0:  rmii_byte <= 8'h45;                    // Ver=4, IHL=5
                            6'd1:  rmii_byte <= 8'h00;                    // DSCP/ECN
                            6'd2:  rmii_byte <= ip_total_len[15:8];
                            6'd3:  rmii_byte <= ip_total_len[7:0];
                            6'd4:  rmii_byte <= 8'h00;                    // ID hi
                            6'd5:  rmii_byte <= 8'h00;                    // ID lo
                            6'd6:  rmii_byte <= 8'h40;                    // DF flag
                            6'd7:  rmii_byte <= 8'h00;                    // frag offset
                            6'd8:  rmii_byte <= TTL;
                            6'd9:  rmii_byte <= 8'h11;                    // Protocol: UDP
                            6'd10: rmii_byte <= ip_checksum[15:8];
                            6'd11: rmii_byte <= ip_checksum[7:0];
                            6'd12: rmii_byte <= LOCAL_IP[31:24];
                            6'd13: rmii_byte <= LOCAL_IP[23:16];
                            6'd14: rmii_byte <= LOCAL_IP[15:8];
                            6'd15: rmii_byte <= LOCAL_IP[7:0];
                            6'd16: rmii_byte <= l_dst_ip[31:24];
                            6'd17: rmii_byte <= l_dst_ip[23:16];
                            6'd18: rmii_byte <= l_dst_ip[15:8];
                            6'd19: rmii_byte <= l_dst_ip[7:0];
                            default: rmii_byte <= 8'h00;
                        endcase
                        crc_vld  <= 1'b1;
                        crc_data <= rmii_byte;
                        hdr_cnt  <= hdr_cnt + 1'b1;
                        if (hdr_cnt == 6'd19) begin
                            state   <= S_UDP_HDR;
                            hdr_cnt <= 6'd0;
                        end
                    end
                end

                // --------------------------------------------------------------
                // UDP Header (8 bytes)
                // --------------------------------------------------------------
                S_UDP_HDR: begin
                    if (rmii_req) begin
                        case (hdr_cnt)
                            6'd0: rmii_byte <= LOCAL_PORT[15:8];
                            6'd1: rmii_byte <= LOCAL_PORT[7:0];
                            6'd2: rmii_byte <= l_dst_port[15:8];
                            6'd3: rmii_byte <= l_dst_port[7:0];
                            6'd4: rmii_byte <= udp_len[15:8];
                            6'd5: rmii_byte <= udp_len[7:0];
                            6'd6: rmii_byte <= 8'h00;  // checksum = 0 (disabled)
                            6'd7: rmii_byte <= 8'h00;
                            default: rmii_byte <= 8'h00;
                        endcase
                        crc_vld  <= 1'b1;
                        crc_data <= rmii_byte;
                        hdr_cnt  <= hdr_cnt + 1'b1;
                        if (hdr_cnt == 6'd7) begin
                            state   <= S_UDP_PAY;
                            pay_cnt <= 5'd0;
                        end
                    end
                end

                // --------------------------------------------------------------
                // UDP Payload – read from buffer
                // --------------------------------------------------------------
                S_UDP_PAY: begin
                    if (rmii_req) begin
                        rmii_byte   <= pay_buf_byte;
                        crc_vld     <= 1'b1;
                        crc_data    <= pay_buf_byte;
                        pay_buf_idx <= pay_cnt;
                        pay_cnt     <= pay_cnt + 1'b1;
                        if (pay_cnt == l_pay_len[4:0] - 1) begin
                            state   <= S_FCS;
                            fcs_cnt <= 2'd0;
                        end
                    end
                end

                // --------------------------------------------------------------
                // FCS (4 bytes, LSB byte first)
                // --------------------------------------------------------------
                S_FCS: begin
                    if (rmii_req) begin
                        case (fcs_cnt)
                            2'd0: rmii_byte <= crc_fcs[7:0];
                            2'd1: rmii_byte <= crc_fcs[15:8];
                            2'd2: rmii_byte <= crc_fcs[23:16];
                            2'd3: rmii_byte <= crc_fcs[31:24];
                        endcase
                        rmii_last <= (fcs_cnt == 2'd3);
                        fcs_cnt   <= fcs_cnt + 1'b1;
                        if (fcs_cnt == 2'd3) begin
                            state   <= S_IDLE;
                            tx_busy <= 1'b0;
                        end
                    end
                end

            endcase
        end
    end

endmodule

`default_nettype wire
