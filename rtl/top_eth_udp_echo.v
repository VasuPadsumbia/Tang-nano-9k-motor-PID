module top_eth_udp_echo#(
  parameter [47:0] FPGA_MAC = 48'h02_12_34_56_78_9A,
  parameter [31:0] FPGA_IP  = 32'hC0A80164,     // 192.168.1.100
  parameter [15:0] UDP_LISTEN_PORT = 16'd5005
)(
  input  wire       clk50,
  input  wire       rst_n,
  input  wire [1:0] rxd,
  input  wire       crs_dv,
  output wire [1:0] txd,
  output wire       tx_en,
  output reg        led_activity
);
  // RMII RX
  wire [7:0] rx_b;
  wire       rx_v;
  wire       frame_active;
  wire [31:0] dst_ip;

  rmii_rx RX(
  .clk50(clk50),
  .rst_n(rst_n),
  .rxd(rxd),
  .crs_dv(crs_dv),
  .rx_byte(rx_b),
  .rx_byte_valid(rx_v),
  .frame_active(frame_active)
);

  // Parse
  wire        is_arp, is_ipv4, is_udp;
  wire [47:0] src_mac;
  wire [31:0] src_ip;
  wire [15:0] udp_sp, udp_dp;
  wire [7:0]  udp_pl;
  wire        udp_pl_v, udp_pl_last;
  wire        frame_done;

  eth_rx_parse P(
    .clk50(clk50), .rst_n(rst_n),
    .b(rx_b), .v(rx_v), .frame_active(frame_active),
    .is_arp(is_arp), .is_ipv4(is_ipv4), .is_udp(is_udp),
    .src_mac(src_mac), .src_ip(src_ip), .dst_ip(dst_ip),
    .udp_src_port(udp_sp), .udp_dst_port(udp_dp),
    .udp_payload(udp_pl), .udp_payload_valid(udp_pl_v), .udp_payload_last(udp_pl_last),
    .frame_done(frame_done)
  );

  // Decide ARP + UDP echo
  wire send_arp_reply, send_udp_echo;
  wire [47:0] echo_dst_mac;
  wire [31:0] echo_dst_ip;
  wire [15:0] echo_dst_port, echo_src_port;
  wire [7:0]  echo_payload;
  wire        echo_payload_valid, echo_payload_last;

  arp_ipv4_udp A(
    .clk50(clk50), .rst_n(rst_n),
    .is_arp(is_arp),
    .rx_src_mac(src_mac), .rx_src_ip(src_ip), .dst_ip(dst_ip),
    .udp_src_port(udp_sp), .udp_dst_port(udp_dp),
    .udp_payload(udp_pl), .udp_payload_valid(udp_pl_v), .udp_payload_last(udp_pl_last),
    .frame_done(frame_done),
    .send_arp_reply(send_arp_reply),
    .send_udp_echo(send_udp_echo),
    .echo_dst_mac(echo_dst_mac), .echo_dst_ip(echo_dst_ip),
    .echo_dst_port(echo_dst_port), .echo_src_port(echo_src_port),
    .echo_payload(echo_payload), .echo_payload_valid(echo_payload_valid), .echo_payload_last(echo_payload_last)
  );

  // Periodic hello timer (1 Hz)
  reg [25:0] t;
  wire tick_1hz = (t == 26'd50_000_000-1); // clk50
  always @(posedge clk50) begin
    if (!rst_n) t <= 0;
    else t <= tick_1hz ? 0 : (t+1);
  end

  // TX builder
  wire tx_busy;
  wire        mac_start;
  wire [7:0]  mac_data;
  wire        mac_valid;
  wire        mac_last;

  udp_tx_build TXB(
    .clk50(clk50), .rst_n(rst_n),
    .send_hello(tick_1hz),
    .send_echo(send_udp_echo),
    .dst_mac(echo_dst_mac),
    .dst_ip(echo_dst_ip),
    .src_port(echo_src_port),
    .dst_port(echo_dst_port),
    .in_payload(echo_payload),
    .in_valid(echo_payload_valid),
    .in_last(echo_payload_last),
    .mac_start(mac_start),
    .mac_data(mac_data),
    .mac_valid(mac_valid),
    .mac_last(mac_last),
    .mac_busy(tx_busy)
  );

  // RMII TX
  rmii_tx TX(
    .clk50(clk50), .rst_n(rst_n),
    .start(mac_start),
    .data(mac_data),
    .data_valid(mac_valid),
    .last(mac_last),
    .ready(),
    .busy(tx_busy),
    .txd(txd),
    .tx_en(tx_en)
  );

  // LED activity pulse on RX/TX events
  reg [23:0] led_cnt;
  always @(posedge clk50) begin
    if (!rst_n) begin
      led_cnt <= 0; led_activity <= 0;
    end else begin
      if (rx_v || mac_start) led_cnt <= 24'd2_000_000; // ~40ms
      else if (led_cnt != 0) led_cnt <= led_cnt - 1;

      led_activity <= (led_cnt != 0);
    end
  end
endmodule
