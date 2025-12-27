module Main #(
    parameter [47:0] LOCAL_MAC = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP  = {8'd10,8'd10,8'd10,8'd100},
    parameter [15:0] LOCAL_PORT = 16'd5005
)(
    // Some projects keep this pin; you can ignore it
    input  wire        CLK,      // from CST pin 52 (optional)
    input  wire        CLK50,     // RMII REF_CLK 50MHz from LAN8720
    input  wire        BTN1,
    input  wire        BTN2,

    input  wire        RMII_CRS_DV,
    input  wire [1:0]  RMII_RXD,
    input  wire        RMII_RX_ER,   // optional, okay if tied low

    output wire        RMII_TX_EN,
    output wire [1:0]  RMII_TXD,

    output wire        RMII_MDC,
    input  wire        RMII_MDIO,

    output wire        LED1,
    output wire        LED2,
    output wire        LED3
);

    wire clk = CLK50;

    // BTN1 is pull-up in CST, so press -> 0
    // Reset asserted when pressed OR on power-up
    reg [19:0] por_cnt;
    reg        por_done;
    always @(posedge clk) begin
        if (!BTN1) begin
            por_cnt <= 20'd0;
            por_done <= 1'b0;
        end else begin
            if (!por_done) begin
                por_cnt <= por_cnt + 1'b1;
                if (por_cnt == 20'hFFFFF) por_done <= 1'b1;
            end
        end
    end
    wire rst = ~por_done;

    // -------- MDIO (unused for Step-1; keep PHY in strap/autoneg mode) ------
    assign RMII_MDC  = 1'b0;
    // assign RMII_MDIO = 1'bz;

    // ---------------- RMII RX -> byte stream ----------------
    wire        rx_sof, rx_eof, rx_vld;
    wire [7:0]  rx_byte;

    rmii_rx u_rmii_rx(
        .clk50(clk),
        .rst(rst),
        .crs_dv(RMII_CRS_DV),
        .rxd(RMII_RXD),
        .rx_er(RMII_RX_ER),
        .sof(rx_sof),
        .eof(rx_eof),
        .vld(rx_vld),
        .byte_out(rx_byte)
    );

    // ---------------- Parse ETH/ARP/IP/UDP ----------------
    wire        arp_req;
    wire [47:0] arp_src_mac;
    wire [31:0] arp_src_ip;

    wire        udp_done;
    wire [47:0] udp_src_mac;
    wire [31:0] udp_src_ip;
    wire [15:0] udp_src_port;
    wire [15:0] udp_dst_port;

    wire        pay_vld;
    wire [7:0]  pay_byte;
    wire [15:0] pay_idx;
    wire        pay_last;

    ipv4_tcp_rx #(
        .LOCAL_MAC(LOCAL_MAC),
        .LOCAL_IP(LOCAL_IP),
        .LOCAL_PORT(LOCAL_PORT)
    ) u_rxdec (
        .clk(clk),
        .rst(rst),
        .sof(rx_sof),
        .eof(rx_eof),
        .vld(rx_vld),
        .byte_in(rx_byte),

        .arp_req(arp_req),
        .arp_src_mac(arp_src_mac),
        .arp_src_ip(arp_src_ip),

        .udp_done(udp_done),
        .udp_src_mac(udp_src_mac),
        .udp_src_ip(udp_src_ip),
        .udp_src_port(udp_src_port),
        .udp_dst_port(udp_dst_port),

        .pay_vld(pay_vld),
        .pay_byte(pay_byte),
        .pay_idx(pay_idx),
        .pay_last(pay_last)
    );

    // --- inside Main ---

    wire        tx_req;
    wire [15:0] tx_len;
    wire        tx_busy;
    wire        tx_done;

    wire [15:0] tx_rd_idx;
    wire [7:0]  tx_rd_byte;

    tcp_echo_server #(
        .LOCAL_MAC(LOCAL_MAC),
        .LOCAL_IP(LOCAL_IP),
        .LOCAL_PORT(LOCAL_PORT)
    ) u_reply (
        .clk(clk),
        .rst(rst),

        .arp_req(arp_req),
        .arp_src_mac(arp_src_mac),
        .arp_src_ip(arp_src_ip),

        .udp_done(udp_done),
        .udp_src_mac(udp_src_mac),
        .udp_src_ip(udp_src_ip),
        .udp_src_port(udp_src_port),
        .udp_dst_port(udp_dst_port),

        .pay_vld(pay_vld),
        .pay_byte(pay_byte),
        .pay_idx(pay_idx),
        .pay_last(pay_last),

        .tx_req(tx_req),
        .tx_len(tx_len),
        .tx_busy(tx_busy),

        .tx_rd_idx(tx_rd_idx),
        .tx_rd_byte(tx_rd_byte),

        .tx_done(tx_done)
    );

    eth_tx_framer u_framer(
        .clk(clk),
        .rst(rst),
        .start(tx_req),
        .frame_len(tx_len),
        .rd_idx(tx_rd_idx),
        .rd_byte(tx_rd_byte),
        .busy(tx_busy),
        .done(tx_done),
        .rmii_tx_en(RMII_TX_EN),
        .rmii_txd(RMII_TXD)
    );

    // Optional LEDs (do not affect network)
    assign LED1 = ~rx_vld;      // activity
    assign LED2 = ~tx_req;      // request seen
    assign LED3 = ~RMII_TX_EN;  // actual TX

endmodule
