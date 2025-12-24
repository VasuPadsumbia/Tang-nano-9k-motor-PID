`include "eth_defs.vh"

module arp_ipv4_udp (
  input  wire        clk50,
  input  wire        rst_n,

  // from parser
  input  wire        is_arp,
  input  wire        is_ipv4,
  input  wire        is_udp,

  input  wire [47:0] rx_src_mac,
  input  wire [31:0] rx_src_ip,

  input  wire [15:0] udp_src_port,
  input  wire [15:0] udp_dst_port,
  input  wire [7:0]  udp_payload,
  input  wire        udp_payload_valid,
  input  wire        udp_payload_last,

  input  wire        frame_done,

  // decisions to TX path
  output reg         send_arp_reply,
  output reg         send_udp_echo,

  output reg  [47:0] echo_dst_mac,
  output reg  [31:0] echo_dst_ip,
  output reg  [15:0] echo_dst_port,
  output reg  [15:0] echo_src_port,

  output reg  [7:0]  echo_payload,
  output reg         echo_payload_valid,
  output reg         echo_payload_last
);

  // Our configured IP/port from the header file
  localparam [31:0] MY_IP   = `FPGA_IP;
  localparam [15:0] MY_PORT = `UDP_LISTEN_PORT;

  always @(posedge clk50) begin
    if (!rst_n) begin
      send_arp_reply      <= 1'b0;
      send_udp_echo       <= 1'b0;

      echo_dst_mac        <= 48'd0;
      echo_dst_ip         <= 32'd0;
      echo_dst_port       <= 16'd0;
      echo_src_port       <= 16'd0;

      echo_payload        <= 8'd0;
      echo_payload_valid  <= 1'b0;
      echo_payload_last   <= 1'b0;
    end else begin
      // defaults (1-cycle pulses)
      send_arp_reply     <= 1'b0;
      send_udp_echo      <= 1'b0;
      echo_payload_valid <= 1'b0;
      echo_payload_last  <= 1'b0;

      // ----------------------------
      // ARP reply decision (minimal)
      // ----------------------------
      // NOTE: Your current top-level doesn't pass "dst_ip" from the parser into this module,
      // so we can't check "ARP target IP == MY_IP" here yet.
      // For bring-up: reply to any ARP frame that finishes.
      if (frame_done && is_arp) begin
        send_arp_reply <= 1'b1;
        echo_dst_mac   <= rx_src_mac;
        echo_dst_ip    <= rx_src_ip;
      end

      // ----------------------------
      // UDP echo decision (disabled for now)
      // ----------------------------
      // If you want to enable UDP echo later, uncomment this block and ensure your parser
      // provides correct ports/payload and your TX builder supports echo mode.
      /*
      if (frame_done && is_ipv4 && is_udp && (udp_dst_port == MY_PORT)) begin
        send_udp_echo <= 1'b1;
        echo_dst_mac  <= rx_src_mac;
        echo_dst_ip   <= rx_src_ip;
        echo_dst_port <= udp_src_port;
        echo_src_port <= MY_PORT;
      end

      // Stream payload through when echo is active (optional)
      if (is_ipv4 && is_udp && (udp_dst_port == MY_PORT)) begin
        echo_payload       <= udp_payload;
        echo_payload_valid <= udp_payload_valid;
        echo_payload_last  <= udp_payload_last;
      end
      */
    end
  end

endmodule