module tcp_echo_server #(
    parameter [47:0] LOCAL_MAC  = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP   = {8'd10,8'd10,8'd10,8'd100},
    parameter [15:0] LOCAL_PORT = 16'd5005,
    parameter [15:0] MAX_PAY    = 16'd512
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        arp_req,
    input  wire [47:0] arp_src_mac,
    input  wire [31:0] arp_src_ip,

    input  wire        udp_done,
    input  wire [47:0] udp_src_mac,
    input  wire [31:0] udp_src_ip,
    input  wire [15:0] udp_src_port,
    input  wire [15:0] udp_dst_port,

    input  wire        pay_vld,
    input  wire [7:0]  pay_byte,
    input  wire [15:0] pay_idx,
    input  wire        pay_last,

    output reg         tx_req,
    output reg  [15:0] tx_len,
    input  wire        tx_busy,

    output reg         tx_byte_vld,
    output reg  [7:0]  tx_byte,
    input  wire        tx_done,

    input  wire [15:0] tx_rd_idx,
    output wire [7:0]  tx_rd_byte
);

    // Payload buffer (internal only!)
    reg [7:0] mem [0:511];
    reg [15:0] pay_count;

    always @(posedge clk) begin
        if (rst) begin
            pay_count <= 16'd0;
        end else begin
            if (pay_vld && (pay_idx < MAX_PAY)) begin
                mem[pay_idx[8:0]] <= pay_byte;
            end
            if (pay_last) begin
                pay_count <= pay_idx + 16'd1;
            end
        end
    end

    // Remember who we reply to
    reg mode_arp;
    reg [47:0] peer_mac;
    reg [31:0] peer_ip;
    reg [15:0] peer_port;

    // TX streaming control
    reg sending;
    reg [15:0] out_idx;

    wire [15:0] ip_total_len_w = 16'd28 + pay_count; // 20+8+payload
    wire [15:0] udp_len_w      = 16'd8  + pay_count;

    // IPv4 header checksum for our fixed header (no options)
    reg [31:0] sum32;
    reg [15:0] ip_chk;
    assign tx_rd_byte = build_byte(mode_arp, tx_rd_idx);

    always @(*) begin
        sum32 = 32'd0;
        sum32 = sum32 + 32'h00004500;
        sum32 = sum32 + {16'd0, ip_total_len_w};
        sum32 = sum32 + 32'h00001234;
        sum32 = sum32 + 32'h00004000;
        sum32 = sum32 + 32'h00004011;
        sum32 = sum32 + 32'h00000000;
        sum32 = sum32 + {16'd0, LOCAL_IP[31:16]};
        sum32 = sum32 + {16'd0, LOCAL_IP[15:0]};
        sum32 = sum32 + {16'd0, peer_ip[31:16]};
        sum32 = sum32 + {16'd0, peer_ip[15:0]};
        sum32 = (sum32 & 32'h0000FFFF) + (sum32 >> 16);
        sum32 = (sum32 & 32'h0000FFFF) + (sum32 >> 16);
        ip_chk = ~sum32[15:0];
    end

    // Start when idle
    always @(posedge clk) begin
        if (rst) begin
            tx_req <= 1'b0;
            tx_len <= 16'd0;
            mode_arp <= 1'b0;
            peer_mac <= 48'd0;
            peer_ip <= 32'd0;
            peer_port <= 16'd0;
            sending <= 1'b0;
            out_idx <= 16'd0;
            tx_byte_vld <= 1'b0;
            tx_byte <= 8'd0;
        end else begin
            tx_req <= 1'b0;
            tx_byte_vld <= 1'b0;

            // Launch new send if idle
            if (!sending && !tx_busy) begin
                if (arp_req) begin
                    mode_arp <= 1'b1;
                    peer_mac <= arp_src_mac;
                    peer_ip  <= arp_src_ip;
                    peer_port <= 16'd0;
                    tx_len <= 16'd42;          // ETH+ARP
                    tx_req <= 1'b1;
                    sending <= 1'b1;
                    out_idx <= 16'd0;
                end else if (udp_done) begin
                    mode_arp <= 1'b0;
                    peer_mac <= udp_src_mac;
                    peer_ip  <= udp_src_ip;
                    peer_port <= udp_src_port;
                    tx_len <= 16'd42 + pay_count;  // ETH+IP+UDP+payload
                    tx_req <= 1'b1;
                    sending <= 1'b1;
                    out_idx <= 16'd0;
                end
            end

            // Stream bytes while framer is busy
            if (sending && tx_busy) begin
                tx_byte_vld <= 1'b1;
                tx_byte <= build_byte(mode_arp, out_idx);
                out_idx <= out_idx + 16'd1;
            end

            if (tx_done) begin
                sending <= 1'b0;
            end
        end
    end

    // Build one byte of the outgoing Ethernet frame.
    // Uses internal mem[] directly (Yosys-safe).
    function [7:0] build_byte;
        input mode_arp_f;
        input [15:0] i;
        reg [7:0] b;
        reg [15:0] pi;
        begin
            b = 8'h00;

            if (mode_arp_f) begin
                // Ethernet header
                if (i==16'd0)  b=peer_mac[47:40];
                if (i==16'd1)  b=peer_mac[39:32];
                if (i==16'd2)  b=peer_mac[31:24];
                if (i==16'd3)  b=peer_mac[23:16];
                if (i==16'd4)  b=peer_mac[15:8];
                if (i==16'd5)  b=peer_mac[7:0];

                if (i==16'd6)  b=LOCAL_MAC[47:40];
                if (i==16'd7)  b=LOCAL_MAC[39:32];
                if (i==16'd8)  b=LOCAL_MAC[31:24];
                if (i==16'd9)  b=LOCAL_MAC[23:16];
                if (i==16'd10) b=LOCAL_MAC[15:8];
                if (i==16'd11) b=LOCAL_MAC[7:0];

                if (i==16'd12) b=8'h08;
                if (i==16'd13) b=8'h06;

                // ARP reply fields
                if (i==16'd14) b=8'h00; if (i==16'd15) b=8'h01;
                if (i==16'd16) b=8'h08; if (i==16'd17) b=8'h00;
                if (i==16'd18) b=8'h06; if (i==16'd19) b=8'h04;
                if (i==16'd20) b=8'h00; if (i==16'd21) b=8'h02;

                // SHA
                if (i==16'd22) b=LOCAL_MAC[47:40];
                if (i==16'd23) b=LOCAL_MAC[39:32];
                if (i==16'd24) b=LOCAL_MAC[31:24];
                if (i==16'd25) b=LOCAL_MAC[23:16];
                if (i==16'd26) b=LOCAL_MAC[15:8];
                if (i==16'd27) b=LOCAL_MAC[7:0];

                // SPA
                if (i==16'd28) b=LOCAL_IP[31:24];
                if (i==16'd29) b=LOCAL_IP[23:16];
                if (i==16'd30) b=LOCAL_IP[15:8];
                if (i==16'd31) b=LOCAL_IP[7:0];

                // THA
                if (i==16'd32) b=peer_mac[47:40];
                if (i==16'd33) b=peer_mac[39:32];
                if (i==16'd34) b=peer_mac[31:24];
                if (i==16'd35) b=peer_mac[23:16];
                if (i==16'd36) b=peer_mac[15:8];
                if (i==16'd37) b=peer_mac[7:0];

                // TPA
                if (i==16'd38) b=peer_ip[31:24];
                if (i==16'd39) b=peer_ip[23:16];
                if (i==16'd40) b=peer_ip[15:8];
                if (i==16'd41) b=peer_ip[7:0];

            end else begin
                // Ethernet header
                if (i==16'd0)  b=peer_mac[47:40];
                if (i==16'd1)  b=peer_mac[39:32];
                if (i==16'd2)  b=peer_mac[31:24];
                if (i==16'd3)  b=peer_mac[23:16];
                if (i==16'd4)  b=peer_mac[15:8];
                if (i==16'd5)  b=peer_mac[7:0];

                if (i==16'd6)  b=LOCAL_MAC[47:40];
                if (i==16'd7)  b=LOCAL_MAC[39:32];
                if (i==16'd8)  b=LOCAL_MAC[31:24];
                if (i==16'd9)  b=LOCAL_MAC[23:16];
                if (i==16'd10) b=LOCAL_MAC[15:8];
                if (i==16'd11) b=LOCAL_MAC[7:0];

                if (i==16'd12) b=8'h08;
                if (i==16'd13) b=8'h00;

                // IPv4 header
                if (i==16'd14) b=8'h45;
                if (i==16'd15) b=8'h00;
                if (i==16'd16) b=ip_total_len_w[15:8];
                if (i==16'd17) b=ip_total_len_w[7:0];
                if (i==16'd18) b=8'h12;
                if (i==16'd19) b=8'h34;
                if (i==16'd20) b=8'h40;
                if (i==16'd21) b=8'h00;
                if (i==16'd22) b=8'h40;
                if (i==16'd23) b=8'h11;
                if (i==16'd24) b=ip_chk[15:8];
                if (i==16'd25) b=ip_chk[7:0];

                if (i==16'd26) b=LOCAL_IP[31:24];
                if (i==16'd27) b=LOCAL_IP[23:16];
                if (i==16'd28) b=LOCAL_IP[15:8];
                if (i==16'd29) b=LOCAL_IP[7:0];

                if (i==16'd30) b=peer_ip[31:24];
                if (i==16'd31) b=peer_ip[23:16];
                if (i==16'd32) b=peer_ip[15:8];
                if (i==16'd33) b=peer_ip[7:0];

                // UDP header: src=LOCAL_PORT, dst=peer_port
                if (i==16'd34) b=LOCAL_PORT[15:8];
                if (i==16'd35) b=LOCAL_PORT[7:0];
                if (i==16'd36) b=peer_port[15:8];
                if (i==16'd37) b=peer_port[7:0];
                if (i==16'd38) b=udp_len_w[15:8];
                if (i==16'd39) b=udp_len_w[7:0];
                if (i==16'd40) b=8'h00;
                if (i==16'd41) b=8'h00;

                if (i >= 16'd42) begin
                    pi = i - 16'd42;
                    if (pi < MAX_PAY) b = mem[pi[8:0]];
                    else b = 8'h00;
                end
            end

            build_byte = b;
        end
    endfunction

endmodule
