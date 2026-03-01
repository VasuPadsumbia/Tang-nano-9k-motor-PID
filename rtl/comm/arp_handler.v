// =============================================================================
// Module  : arp_handler
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/arp_handler.v
//
// Purpose : Receives an ARP who-has request (from eth_rx_parser) and
//           queues a pre-built ARP reply frame for transmission.
//
//   ARP Reply frame layout (60 bytes without FCS):
//     [0-5]   Dst MAC  = arp_src_mac (the requester)
//     [6-11]  Src MAC  = LOCAL_MAC
//     [12-13] EtherType= 0x0806
//     [14-15] HTYPE    = 0x0001 (Ethernet)
//     [16-17] PTYPE    = 0x0800 (IPv4)
//     [18]    HLEN     = 0x06
//     [19]    PLEN     = 0x04
//     [20-21] OPER     = 0x0002 (reply)
//     [22-27] SHA      = LOCAL_MAC
//     [28-31] SPA      = LOCAL_IP
//     [32-37] THA      = arp_src_mac
//     [38-41] TPA      = arp_src_ip
//     [42-59] padding  = 0x00 (18 bytes to reach minimum 60-byte frame)
//
// Interface
//   clk/rst       – System clock and reset
//   arp_req       – Pulse from eth_rx_parser (ARP for us received)
//   arp_src_mac   – Sender MAC from the ARP request
//   arp_src_ip    – Sender IP from the ARP request
//   tx_req        – Pulse: assert to request TX of arp_reply_buf
//   tx_busy       – HIGH when TX is in progress (from eth_tx_builder)
//   buf_byte      – Current byte to transmit (addressed by buf_idx)
//   buf_idx       – ETH TX builder drives this to walk through our buffer
//   buf_len       – Total frame length (60 bytes)
//   buf_last      – HIGH when buf_idx == buf_len - 1
// =============================================================================

`default_nettype none

module arp_handler #(
    parameter [47:0] LOCAL_MAC = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP  = {8'd10, 8'd10, 8'd10, 8'd100}
)(
    input  wire        clk,
    input  wire        rst,

    // From eth_rx_parser
    input  wire        arp_req,
    input  wire [47:0] arp_src_mac,
    input  wire [31:0] arp_src_ip,

    // To eth_tx_builder / arbiter
    output reg         tx_req,
    input  wire        tx_busy,

    // Frame buffer interface
    input  wire [5:0]  buf_idx,
    output reg  [7:0]  buf_byte,
    output wire [5:0]  buf_len,
    output wire        buf_last
);

    localparam FRAME_LEN = 6'd60;
    assign buf_len  = FRAME_LEN;
    assign buf_last = (buf_idx == FRAME_LEN - 1);

    // Latch requester details
    reg [47:0] req_mac;
    reg [31:0] req_ip;

    always @(posedge clk) begin
        if (rst) begin
            tx_req  <= 1'b0;
            req_mac <= 48'd0;
            req_ip  <= 32'd0;
        end else begin
            tx_req <= 1'b0;
            if (arp_req && !tx_busy) begin
                req_mac <= arp_src_mac;
                req_ip  <= arp_src_ip;
                tx_req  <= 1'b1;
            end
        end
    end

    // Combinatorial frame byte mux
    always @(*) begin
        case (buf_idx)
            // Destination MAC (requester)
            6'd0:  buf_byte = req_mac[47:40];
            6'd1:  buf_byte = req_mac[39:32];
            6'd2:  buf_byte = req_mac[31:24];
            6'd3:  buf_byte = req_mac[23:16];
            6'd4:  buf_byte = req_mac[15:8];
            6'd5:  buf_byte = req_mac[7:0];
            // Source MAC (us)
            6'd6:  buf_byte = LOCAL_MAC[47:40];
            6'd7:  buf_byte = LOCAL_MAC[39:32];
            6'd8:  buf_byte = LOCAL_MAC[31:24];
            6'd9:  buf_byte = LOCAL_MAC[23:16];
            6'd10: buf_byte = LOCAL_MAC[15:8];
            6'd11: buf_byte = LOCAL_MAC[7:0];
            // EtherType = ARP (0x0806)
            6'd12: buf_byte = 8'h08;
            6'd13: buf_byte = 8'h06;
            // HTYPE = Ethernet (0x0001)
            6'd14: buf_byte = 8'h00;
            6'd15: buf_byte = 8'h01;
            // PTYPE = IPv4 (0x0800)
            6'd16: buf_byte = 8'h08;
            6'd17: buf_byte = 8'h00;
            // HLEN = 6, PLEN = 4
            6'd18: buf_byte = 8'h06;
            6'd19: buf_byte = 8'h04;
            // OPER = Reply (0x0002)
            6'd20: buf_byte = 8'h00;
            6'd21: buf_byte = 8'h02;
            // SHA = our MAC
            6'd22: buf_byte = LOCAL_MAC[47:40];
            6'd23: buf_byte = LOCAL_MAC[39:32];
            6'd24: buf_byte = LOCAL_MAC[31:24];
            6'd25: buf_byte = LOCAL_MAC[23:16];
            6'd26: buf_byte = LOCAL_MAC[15:8];
            6'd27: buf_byte = LOCAL_MAC[7:0];
            // SPA = our IP
            6'd28: buf_byte = LOCAL_IP[31:24];
            6'd29: buf_byte = LOCAL_IP[23:16];
            6'd30: buf_byte = LOCAL_IP[15:8];
            6'd31: buf_byte = LOCAL_IP[7:0];
            // THA = requester MAC
            6'd32: buf_byte = req_mac[47:40];
            6'd33: buf_byte = req_mac[39:32];
            6'd34: buf_byte = req_mac[31:24];
            6'd35: buf_byte = req_mac[23:16];
            6'd36: buf_byte = req_mac[15:8];
            6'd37: buf_byte = req_mac[7:0];
            // TPA = requester IP
            6'd38: buf_byte = req_ip[31:24];
            6'd39: buf_byte = req_ip[23:16];
            6'd40: buf_byte = req_ip[15:8];
            6'd41: buf_byte = req_ip[7:0];
            // Padding (bytes 42-59)
            default: buf_byte = 8'h00;
        endcase
    end

endmodule

`default_nettype wire
