module ipv4_tcp_rx #(
    parameter [47:0] LOCAL_MAC  = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP   = {8'd10,8'd10,8'd10,8'd100},
    parameter [15:0] LOCAL_PORT = 16'd5005
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        sof,
    input  wire        eof,
    input  wire        vld,
    input  wire [7:0]  byte_in,

    output reg         arp_req,
    output reg  [47:0] arp_src_mac,
    output reg  [31:0] arp_src_ip,

    output reg         udp_done,
    output reg  [47:0] udp_src_mac,
    output reg  [31:0] udp_src_ip,
    output reg  [15:0] udp_src_port,
    output reg  [15:0] udp_dst_port,

    output reg         pay_vld,
    output reg  [7:0]  pay_byte,
    output reg  [15:0] pay_idx,
    output reg         pay_last
);

    reg [15:0] idx;

    reg [47:0] dmac, smac;
    reg [15:0] etype;

    reg [15:0] arp_op;
    reg [47:0] arp_sha;
    reg [31:0] arp_spa;
    reg [31:0] arp_tpa;

    reg [7:0]  ip_vihl;
    reg [7:0]  ip_proto;
    reg [31:0] ip_src;
    reg [31:0] ip_dst;
    reg [15:0] l4_start;

    reg [15:0] udp_sport;
    reg [15:0] udp_dport;

    reg        udp_match;
    reg [15:0] payload_start;

    wire is_arp  = (etype == 16'h0806);
    wire is_ipv4 = (etype == 16'h0800);

    always @(posedge clk) begin
        if (rst) begin
            idx <= 0;
            dmac <= 0; smac <= 0; etype <= 0;
            arp_op <= 0; arp_sha <= 0; arp_spa <= 0; arp_tpa <= 0;
            ip_vihl <= 0; ip_proto <= 0; ip_src <= 0; ip_dst <= 0; l4_start <= 16'd34;
            udp_sport <= 0; udp_dport <= 0;
            udp_match <= 1'b0;
            payload_start <= 0;

            arp_req <= 1'b0;
            arp_src_mac <= 0;
            arp_src_ip <= 0;

            udp_done <= 1'b0;
            udp_src_mac <= 0; udp_src_ip <= 0;
            udp_src_port <= 0; udp_dst_port <= 0;

            pay_vld <= 1'b0; pay_byte <= 0; pay_idx <= 0; pay_last <= 1'b0;
        end else begin
            arp_req <= 1'b0;
            udp_done <= 1'b0;
            pay_vld <= 1'b0;
            pay_last <= 1'b0;

            if (sof) begin
                idx <= 0;
                dmac <= 0; smac <= 0; etype <= 0;
                arp_op <= 0; arp_sha <= 0; arp_spa <= 0; arp_tpa <= 0;
                ip_vihl <= 0; ip_proto <= 0; ip_src <= 0; ip_dst <= 0;
                l4_start <= 16'd34;
                udp_sport <= 0; udp_dport <= 0;
                udp_match <= 1'b0;
                payload_start <= 16'd0;
                pay_idx <= 16'd0;
            end

            if (vld) begin
                // Ethernet header
                if (idx < 16'd6)       dmac <= {dmac[39:0], byte_in};
                else if (idx < 16'd12) smac <= {smac[39:0], byte_in};
                else if (idx == 16'd12) etype[15:8] <= byte_in;
                else if (idx == 16'd13) etype[7:0]  <= byte_in;

                // ARP fields we need
                if (idx == 16'd20) arp_op[15:8] <= byte_in;
                if (idx == 16'd21) arp_op[7:0]  <= byte_in;
                if (idx >= 16'd22 && idx < 16'd28) arp_sha <= {arp_sha[39:0], byte_in};
                if (idx >= 16'd28 && idx < 16'd32) arp_spa <= {arp_spa[23:0], byte_in};
                if (idx >= 16'd38 && idx < 16'd42) arp_tpa <= {arp_tpa[23:0], byte_in};

                // IPv4 minimal
                if (idx == 16'd14) begin
                    ip_vihl <= byte_in;
                    // l4_start = 14 + IHL*4 (IHL in low nibble)
                    l4_start <= 16'd14 + {10'd0, byte_in[3:0], 2'b00};
                end
                if (idx == 16'd23) ip_proto <= byte_in;
                if (idx >= 16'd26 && idx < 16'd30) ip_src <= {ip_src[23:0], byte_in};
                if (idx >= 16'd30 && idx < 16'd34) ip_dst <= {ip_dst[23:0], byte_in};

                // UDP header
                if (idx == (l4_start + 16'd0)) udp_sport[15:8] <= byte_in;
                if (idx == (l4_start + 16'd1)) udp_sport[7:0]  <= byte_in;
                if (idx == (l4_start + 16'd2)) udp_dport[15:8] <= byte_in;
                if (idx == (l4_start + 16'd3)) begin
                    udp_dport[7:0] <= byte_in;

                    // Decide match only once full dst port is known
                    if (is_ipv4 && (ip_proto == 8'd17) && (ip_dst == LOCAL_IP) &&
                        ({udp_dport[15:8], byte_in} == LOCAL_PORT)) begin
                        udp_match <= 1'b1;
                        payload_start <= l4_start + 16'd8;
                    end
                end

                // Payload stream
                if (udp_match && idx >= payload_start) begin
                    pay_vld <= 1'b1;
                    pay_byte <= byte_in;
                    pay_idx <= (idx - payload_start);
                end

                idx <= idx + 16'd1;
            end

            if (eof) begin
                // ARP who-has for us
                if (is_arp && (arp_op == 16'h0001) && (arp_tpa == LOCAL_IP)) begin
                    arp_req <= 1'b1;
                    arp_src_mac <= arp_sha;
                    arp_src_ip  <= arp_spa;
                end

                if (udp_match) begin
                    udp_done <= 1'b1;
                    udp_src_mac <= smac;
                    udp_src_ip  <= ip_src;
                    udp_src_port <= udp_sport;
                    udp_dst_port <= udp_dport;
                    pay_last <= 1'b1;
                end

                udp_match <= 1'b0;
            end
        end
    end

endmodule
