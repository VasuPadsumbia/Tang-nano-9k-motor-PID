module arp_responder #(
    parameter [47:0] LOCAL_MAC = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP  = {8'd10,8'd10,8'd10,8'd100}
)(
    input  wire [47:0] src_mac,
    input  wire [31:0] src_ip,
    output wire [15:0] plen,
    input  wire [15:0] pidx,
    output reg  [7:0]  pbyte
);
    assign plen = 16'd42;

    // Ethernet + ARP reply
    always @(*) begin
        case (pidx)
            // dst mac = requester
            16'd0: pbyte = src_mac[47:40];
            16'd1: pbyte = src_mac[39:32];
            16'd2: pbyte = src_mac[31:24];
            16'd3: pbyte = src_mac[23:16];
            16'd4: pbyte = src_mac[15:8];
            16'd5: pbyte = src_mac[7:0];

            // src mac = us
            16'd6:  pbyte = LOCAL_MAC[47:40];
            16'd7:  pbyte = LOCAL_MAC[39:32];
            16'd8:  pbyte = LOCAL_MAC[31:24];
            16'd9:  pbyte = LOCAL_MAC[23:16];
            16'd10: pbyte = LOCAL_MAC[15:8];
            16'd11: pbyte = LOCAL_MAC[7:0];

            // ethertype ARP
            16'd12: pbyte = 8'h08;
            16'd13: pbyte = 8'h06;

            // ARP header
            16'd14: pbyte = 8'h00; // HTYPE Ethernet
            16'd15: pbyte = 8'h01;
            16'd16: pbyte = 8'h08; // PTYPE IPv4
            16'd17: pbyte = 8'h00;
            16'd18: pbyte = 8'h06; // HLEN
            16'd19: pbyte = 8'h04; // PLEN
            16'd20: pbyte = 8'h00; // OPER reply
            16'd21: pbyte = 8'h02;

            // SHA = us
            16'd22: pbyte = LOCAL_MAC[47:40];
            16'd23: pbyte = LOCAL_MAC[39:32];
            16'd24: pbyte = LOCAL_MAC[31:24];
            16'd25: pbyte = LOCAL_MAC[23:16];
            16'd26: pbyte = LOCAL_MAC[15:8];
            16'd27: pbyte = LOCAL_MAC[7:0];

            // SPA = us
            16'd28: pbyte = LOCAL_IP[31:24];
            16'd29: pbyte = LOCAL_IP[23:16];
            16'd30: pbyte = LOCAL_IP[15:8];
            16'd31: pbyte = LOCAL_IP[7:0];

            // THA = requester
            16'd32: pbyte = src_mac[47:40];
            16'd33: pbyte = src_mac[39:32];
            16'd34: pbyte = src_mac[31:24];
            16'd35: pbyte = src_mac[23:16];
            16'd36: pbyte = src_mac[15:8];
            16'd37: pbyte = src_mac[7:0];

            // TPA = requester IP
            16'd38: pbyte = src_ip[31:24];
            16'd39: pbyte = src_ip[23:16];
            16'd40: pbyte = src_ip[15:8];
            16'd41: pbyte = src_ip[7:0];

            default: pbyte = 8'h00;
        endcase
    end
endmodule
