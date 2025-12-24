`include "rtl/eth_defs.vh"

module udp_tx_build(
  input  wire        clk50,
  input  wire        rst_n,

  // Requests
  input  wire        send_hello,
  input  wire        send_echo,

  // Echo destination and ports
  input  wire [47:0] dst_mac,
  input  wire [31:0] dst_ip,
  input  wire [15:0] src_port,
  input  wire [15:0] dst_port,

  // Echo payload stream
  input  wire [7:0]  in_payload,
  input  wire        in_valid,
  input  wire        in_last,

  // To RMII TX
  output reg         mac_start,
  output reg  [7:0]  mac_data,
  output reg         mac_valid,
  output reg         mac_last,
  input  wire        mac_busy
);

  // ------------------------------------------------------------
  // Echo payload buffer (up to 256 bytes)
  // ------------------------------------------------------------
  reg [7:0] echo_buf [0:255];
  reg [7:0] echo_len;
  reg [7:0] wr_ptr;

  // IMPORTANT: have_echo must be driven by ONE always block only
  reg have_echo;
  reg consume_echo;   // request to clear have_echo (from TX FSM)

  // Capture payload into buffer + manage have_echo ownership
  always @(posedge clk50) begin
    if (!rst_n) begin
      wr_ptr    <= 8'd0;
      echo_len  <= 8'd0;
      have_echo <= 1'b0;
    end else begin
      // TX FSM requests clearing after it finishes sending echo
      if (consume_echo) begin
        have_echo <= 1'b0;
      end

      if (in_valid) begin
        echo_buf[wr_ptr] <= in_payload;
        wr_ptr <= wr_ptr + 8'd1;

        if (in_last) begin
          echo_len  <= wr_ptr + 8'd1;
          wr_ptr    <= 8'd0;
          have_echo <= 1'b1; // ONLY this block sets it
        end
      end
    end
  end

  // ------------------------------------------------------------
  // Hello payload
  // ------------------------------------------------------------
  function [7:0] hello_byte(input [7:0] idx);
    begin
      case (idx)
        0: hello_byte="H";
        1: hello_byte="e";
        2: hello_byte="l";
        3: hello_byte="l";
        4: hello_byte="o";
        5: hello_byte=" ";
        6: hello_byte="F";
        7: hello_byte="P";
        8: hello_byte="G";
        9: hello_byte="A";
        10: hello_byte=8'h0A;
        default: hello_byte=8'h00;
      endcase
    end
  endfunction
  wire [7:0] hello_len = 8'd11;

  // ------------------------------------------------------------
  // Header generator (Ethernet + IPv4 + UDP) -> 42 bytes
  // NOTE: IP header checksum not computed here (0x0000)
  // ------------------------------------------------------------
  function [7:0] hdr_byte(
    input [15:0] idx,
    input [47:0] dmac,
    input [47:0] smac,
    input [31:0] sip,
    input [31:0] dip,
    input [15:0] sport,
    input [15:0] dport,
    input [15:0] ip_total_len,
    input [15:0] udp_total_len
  );
    begin
      // Ethernet
      if (idx < 6)       hdr_byte = dmac[47-8*idx -: 8];
      else if (idx < 12) hdr_byte = smac[47-8*(idx-6) -: 8];
      else if (idx==12)  hdr_byte = 8'h08;
      else if (idx==13)  hdr_byte = 8'h00;

      // IPv4
      else if (idx==14)  hdr_byte = 8'h45;
      else if (idx==15)  hdr_byte = 8'h00;
      else if (idx==16)  hdr_byte = ip_total_len[15:8];
      else if (idx==17)  hdr_byte = ip_total_len[7:0];
      else if (idx==18)  hdr_byte = 8'h00;
      else if (idx==19)  hdr_byte = 8'h01;
      else if (idx==20)  hdr_byte = 8'h00;
      else if (idx==21)  hdr_byte = 8'h00;
      else if (idx==22)  hdr_byte = 8'h40;
      else if (idx==23)  hdr_byte = 8'h11; // UDP
      else if (idx==24)  hdr_byte = 8'h00; // IP checksum TODO
      else if (idx==25)  hdr_byte = 8'h00;
      else if (idx==26)  hdr_byte = sip[31:24];
      else if (idx==27)  hdr_byte = sip[23:16];
      else if (idx==28)  hdr_byte = sip[15:8];
      else if (idx==29)  hdr_byte = sip[7:0];
      else if (idx==30)  hdr_byte = dip[31:24];
      else if (idx==31)  hdr_byte = dip[23:16];
      else if (idx==32)  hdr_byte = dip[15:8];
      else if (idx==33)  hdr_byte = dip[7:0];

      // UDP
      else if (idx==34)  hdr_byte = sport[15:8];
      else if (idx==35)  hdr_byte = sport[7:0];
      else if (idx==36)  hdr_byte = dport[15:8];
      else if (idx==37)  hdr_byte = dport[7:0];
      else if (idx==38)  hdr_byte = udp_total_len[15:8];
      else if (idx==39)  hdr_byte = udp_total_len[7:0];
      else if (idx==40)  hdr_byte = 8'h00; // UDP checksum=0 allowed in IPv4
      else if (idx==41)  hdr_byte = 8'h00;
      else hdr_byte = 8'h00;
    end
  endfunction

  wire [47:0] fpga_mac = `FPGA_MAC;
  wire [31:0] fpga_ip  = `FPGA_IP;

  // ------------------------------------------------------------
  // TX FSM
  // ------------------------------------------------------------
  localparam S_IDLE=0, S_HDR=1, S_PAY=2, S_DONE=3;
  reg [1:0] st;

  reg [15:0] i;
  reg [7:0]  pay_i;
  reg        do_hello, do_echo;

  reg [47:0] use_dmac;
  reg [31:0] use_dip;
  reg [15:0] use_sport, use_dport;

  reg [15:0] udp_total_len;
  reg [15:0] ip_total_len;

  always @(posedge clk50) begin
    if (!rst_n) begin
      st <= S_IDLE;
      mac_start <= 0; mac_valid <= 0; mac_last <= 0; mac_data <= 0;
      i <= 0; pay_i <= 0;
      do_hello <= 0; do_echo <= 0;
      use_dmac <= 0; use_dip <= 0; use_sport <= 0; use_dport <= 0;
      udp_total_len <= 0; ip_total_len <= 0;
      consume_echo <= 0;
    end else begin
      mac_start <= 0;
      mac_valid <= 0;
      mac_last  <= 0;
      consume_echo <= 0; // default

      case (st)
        S_IDLE: begin
          if (!mac_busy) begin
            if (send_hello) begin
              do_hello <= 1'b1;
              do_echo  <= 1'b0;

              use_dmac  <= `PC_MAC_DEFAULT;
              use_dip   <= `PC_IP_DEFAULT;
              use_sport <= `UDP_LISTEN_PORT;
              use_dport <= `UDP_LISTEN_PORT;

              udp_total_len <= 16'd8 + {8'd0, hello_len};                  // width-fix
              ip_total_len  <= 16'd20 + 16'd8 + {8'd0, hello_len};         // width-fix

              i <= 0; pay_i <= 0;
              mac_start <= 1'b1;
              st <= S_HDR;

            end else if (send_echo && have_echo) begin
              do_hello <= 1'b0;
              do_echo  <= 1'b1;

              use_dmac  <= dst_mac;
              use_dip   <= dst_ip;
              use_sport <= src_port;
              use_dport <= dst_port;

              udp_total_len <= 16'd8 + {8'd0, echo_len};                   // width-fix
              ip_total_len  <= 16'd20 + 16'd8 + {8'd0, echo_len};          // width-fix

              i <= 0; pay_i <= 0;
              mac_start <= 1'b1;
              st <= S_HDR;
            end
          end
        end

        S_HDR: begin
          mac_data  <= hdr_byte(i, use_dmac, fpga_mac, fpga_ip, use_dip,
                                use_sport, use_dport,
                                16'd14 + ip_total_len, udp_total_len);
          mac_valid <= 1'b1;

          if (i == 16'd41) begin
            st <= S_PAY;
            i <= 0;
          end else begin
            i <= i + 16'd1;
          end
        end

        S_PAY: begin
          mac_valid <= 1'b1;

          if (do_hello) begin
            mac_data <= hello_byte(pay_i);
            if (pay_i == hello_len-1) begin
              mac_last <= 1'b1;
              st <= S_DONE;
            end
            pay_i <= pay_i + 8'd1;

          end else if (do_echo) begin
            mac_data <= echo_buf[pay_i];
            if (pay_i == echo_len-1) begin
              mac_last <= 1'b1;
              st <= S_DONE;
            end
            pay_i <= pay_i + 8'd1;
          end
        end

        S_DONE: begin
          // IMPORTANT: TX FSM does NOT directly modify have_echo
          // It only requests the capture block to clear it.
          if (do_echo) begin
            consume_echo <= 1'b1;
          end
          st <= S_IDLE;
        end
      endcase
    end
  end

endmodule
