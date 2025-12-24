module eth_rx_parse(
  input  wire       clk50,
  input  wire       rst_n,
  input  wire [7:0] b,
  input  wire       v,
  input  wire       frame_active,
  
  output reg        is_arp,
  output reg        is_ipv4,
  output reg        is_udp,

  output reg [47:0] src_mac,
  output reg [31:0] src_ip,
  output reg [31:0] dst_ip,

  output reg [15:0] udp_src_port,
  output reg [15:0] udp_dst_port,

  output reg [7:0]  udp_payload,
  output reg        udp_payload_valid,
  output reg        udp_payload_last,

  output reg        frame_done
);
  reg [15:0] idx;
  reg [15:0] eth_type;
  reg [7:0]  ip_proto;
  reg [15:0] ip_hdr_len_bytes;

  reg [15:0] udp_start;
  reg [15:0] udp_len;
  reg [15:0] udp_payload_start;
  reg [15:0] udp_payload_end;

  always @(posedge clk50) begin
    if (!rst_n) begin
      idx <= 0;
      frame_done <= 0;
      is_arp <= 0; is_ipv4 <= 0; is_udp <= 0;
      src_mac <= 0; src_ip <= 0; dst_ip <= 0;
      udp_src_port <= 0; udp_dst_port <= 0;
      udp_payload <= 0; udp_payload_valid <= 0; udp_payload_last <= 0;
      eth_type <= 0; ip_proto <= 0; ip_hdr_len_bytes <= 0;
      udp_start <= 0; udp_len <= 0; udp_payload_start <= 0; udp_payload_end <= 0;
    end else begin
      frame_done <= 0;
      udp_payload_valid <= 0;
      udp_payload_last  <= 0;

      if (!frame_active) begin
        if (idx != 0) frame_done <= 1'b1;
        idx <= 0;
        is_udp <= 1'b0;
      end else if (v) begin
        idx <= idx + 1;

        // Ethernet: dst[0..5], src[6..11], type[12..13]
        if (idx == 6)  src_mac[47:40] <= b;
        if (idx == 7)  src_mac[39:32] <= b;
        if (idx == 8)  src_mac[31:24] <= b;
        if (idx == 9)  src_mac[23:16] <= b;
        if (idx == 10) src_mac[15:8]  <= b;
        if (idx == 11) src_mac[7:0]   <= b;

        if (idx == 12) eth_type[15:8] <= b;
        if (idx == 13) begin
          eth_type[7:0] <= b;
          is_arp  <= ({eth_type[15:8], b} == 16'h0806);
          is_ipv4 <= ({eth_type[15:8], b} == 16'h0800);
        end

        // IPv4 header begins at idx 14
        if (idx == 14) ip_hdr_len_bytes <= {12'd0, b[3:0]} * 16'd4; // IHL*4
        if (idx == 23) ip_proto <= b;

        if (idx == 26) src_ip[31:24] <= b;
        if (idx == 27) src_ip[23:16] <= b;
        if (idx == 28) src_ip[15:8]  <= b;
        if (idx == 29) src_ip[7:0]   <= b;

        if (idx == 30) dst_ip[31:24] <= b;
        if (idx == 31) dst_ip[23:16] <= b;
        if (idx == 32) dst_ip[15:8]  <= b;
        if (idx == 33) dst_ip[7:0]   <= b;

        udp_start <= 16'd14 + ip_hdr_len_bytes;

        if (idx == udp_start + 0) udp_src_port[15:8] <= b;
        if (idx == udp_start + 1) udp_src_port[7:0]  <= b;
        if (idx == udp_start + 2) udp_dst_port[15:8] <= b;
        if (idx == udp_start + 3) udp_dst_port[7:0]  <= b;

        if (idx == udp_start + 4) udp_len[15:8] <= b;
        if (idx == udp_start + 5) begin
          udp_len[7:0] <= b;
          is_udp <= (is_ipv4 && (ip_proto == 8'h11));
          udp_payload_start <= udp_start + 16'd8;
          udp_payload_end   <= (udp_start + {udp_len[15:8], b}) - 16'd1;
        end

        if (is_udp && idx >= udp_payload_start && idx <= udp_payload_end) begin
          udp_payload <= b;
          udp_payload_valid <= 1'b1;
          udp_payload_last  <= (idx == udp_payload_end);
        end
      end
    end
  end
endmodule
