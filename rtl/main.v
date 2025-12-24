module Main(
  input  wire       CLK50,
  input  wire [1:0] RMII_RXD,
  input  wire       RMII_CRS_DV,
  output wire [1:0] RMII_TXD,
  output wire       RMII_TX_EN,
  input  wire       BTN1,
  input  wire       BTN2,
  output wire       LED1
);

  // power-on reset
  reg [15:0] rst_cnt = 16'd0;
  wire rst_n = &rst_cnt;
  always @(posedge CLK50) if (!rst_n) rst_cnt <= rst_cnt + 16'd1;

  // heartbeat
  reg [25:0] hb_cnt = 26'd0;
  reg hb_led = 1'b0;

  // buttons are usually active-low on boards (pressed = 0)
  wire b1 = ~BTN1;
  wire b2 = ~BTN2;
  wire [1:0] txd_tmp;
  wire       tx_en_tmp;
  always @(posedge CLK50) begin
    if (!rst_n) begin
      hb_cnt <= 0;
      hb_led <= 0;
    end else if (b2) begin
      hb_cnt <= 0;          // demo: reset heartbeat counter
      hb_led <= 0;
    end else begin
      if (hb_cnt == 26'd50_000_000-1) begin
        hb_cnt <= 0;
        hb_led <= ~hb_led;
      end else hb_cnt <= hb_cnt + 1;
    end
  end

  wire eth_led;
  top_eth_udp_echo ETH (
    .clk50(CLK50),
    .rst_n(rst_n),
    .rxd(RMII_RXD),
    .crs_dv(RMII_CRS_DV),
    .txd(txd_tmp),
    .tx_en(tx_en_tmp),
    .led_activity(eth_led)
  );
  // DEBUG: hold TX lines idle
  assign RMII_TXD   = 2'b00;
  assign RMII_TX_EN = 1'b0;
  // BTN1 switches between heartbeat-only and ethernet+heartbeat
  wire led_out = b1 ? (eth_led | hb_led) : hb_led;

  assign LED1 = ~led_out;  // active-low LED

endmodule
