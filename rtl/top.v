// =============================================================================
// Module  : top  (Top-Level)
// Project : Tang Nano 9K – PID Motor Controller over Ethernet (LAN8720)
// File    : rtl/top.v
//
// Description
//   Connects all sub-modules:
//     sys/clk_div         → 1 kHz PID sample tick from 50 MHz REF_CLK
//     motor/encoder_reader → Quadrature position counter
//     motor/pid_controller → Fixed-point PID (Kp, Ki, Kd via UDP)
//     motor/motor_bridge   → Signed PID → PWM duty + direction
//     motor/pwm_gen        → PWM output pin
//     comm/rmii_rx         → RMII receive byte stream
//     comm/rmii_tx         → RMII transmit dibit stream
//     comm/eth_crc32       → CRC-32 (instantiated inside eth_tx_builder)
//     comm/eth_rx_parser   → ARP / IPv4-UDP frame parser
//     comm/arp_handler     → ARP reply builder
//     comm/udp_ctrl_rx     → Command decoder (setpoint, gains, enable)
//     comm/udp_telem_tx    → Periodic telemetry sender
//     comm/eth_tx_builder  → Eth+IP+UDP frame builder, arbiter, FCS
//
// Configurable Parameters (adjust to match your network setup):
//   LOCAL_MAC  – Unique locally-administered MAC for the FPGA
//   LOCAL_IP   – Static IPv4 address (PC must be on same /24 subnet)
//   LOCAL_PORT – UDP control/telemetry port (default 5005)
//
// Pin Assignments (see pinout.cst):
//   CLK50     – 50 MHz REF_CLK from LAN8720 INT/REFCLK (clock-capable pin 38)
//   BTN1      – Reset button (active LOW, pull-up in CST)
//   RMII_*    – LAN8720 RMII signals
//   ENC_A/B   – Quadrature encoder channels
//   PWM_OUT   – Motor PWM output → H-bridge EN
//   MOTOR_DIR – Motor direction → H-bridge IN1
//   LED1      – Heartbeat (toggles at 1 Hz when running)
//   LED2      – TX activity (flashes on Ethernet transmit)
//   LED3      – PID active indicator
// =============================================================================

`default_nettype none

module top #(
    parameter [47:0] LOCAL_MAC  = 48'h02_12_34_56_78_9A,
    parameter [31:0] LOCAL_IP   = {8'd10, 8'd10, 8'd10, 8'd100},
    parameter [15:0] LOCAL_PORT = 16'd5005
)(
    // System
    input  wire        CLK,         // 27 MHz onboard crystal (unused – keep for CST)
    input  wire        CLK50,       // 50 MHz from LAN8720 REF_CLK (clock-capable)
    input  wire        BTN1,        // Reset (active LOW, pull-up)

    // RMII – LAN8720
    input  wire        RMII_CRS_DV,
    input  wire [1:0]  RMII_RXD,
    input  wire        RMII_RX_ER,
    output wire        RMII_TX_EN,
    output wire [1:0]  RMII_TXD,
    output wire        RMII_MDC,
    input  wire        RMII_MDIO,

    // Motor control
    input  wire        ENC_A,       // Encoder channel A
    input  wire        ENC_B,       // Encoder channel B
    output wire        PWM_OUT,     // H-bridge EN/PWM
    output wire        MOTOR_DIR,   // H-bridge IN1 (IN2 = ~MOTOR_DIR externally)

    // LEDs (active LOW on Tang Nano 9K)
    output wire        LED1,
    output wire        LED2,
    output wire        LED3
);

    // =========================================================================
    // 1. Clock & Reset
    // =========================================================================
    wire clk = CLK50;    // Use the clean 50 MHz REF_CLK as system clock

    // Power-on reset counter – holds reset for ~21 ms after power-up or BTN1
    reg [19:0] por_cnt;
    reg        por_done;
    always @(posedge clk) begin
        if (!BTN1) begin
            por_cnt  <= 20'd0;
            por_done <= 1'b0;
        end else if (!por_done) begin
            por_cnt <= por_cnt + 1'b1;
            if (por_cnt == 20'hFFFFF) por_done <= 1'b1;
        end
    end
    wire rst = ~por_done;

    // MDIO: keep PHY in autoneg strap mode (MDC held low = no MDIO activity)
    assign RMII_MDC = 1'b0;

    // =========================================================================
    // 2. 1 kHz PID Sample Tick
    // =========================================================================
    wire sample_tick;

    clk_div #(
        .CLK_FREQ (50_000_000),
        .TICK_FREQ(1_000)
    ) u_clkdiv (
        .clk  (clk),
        .rst  (rst),
        .tick (sample_tick)
    );

    // =========================================================================
    // 3. Encoder Reader
    // =========================================================================
    wire [31:0] enc_position;
    wire        enc_dir;

    encoder_reader u_enc (
        .clk      (clk),
        .rst      (rst),
        .enc_a    (ENC_A),
        .enc_b    (ENC_B),
        .position (enc_position),
        .enc_dir  (enc_dir)
    );

    // Use the upper 16 bits of the 32-bit position as the PID feedback.
    // This effectively divides by 65536 – adjust to match your encoder resolution.
    wire signed [15:0] feedback = enc_position[31:16];

    // =========================================================================
    // 4. UDP Control Receiver – setpoint / gain registers
    // =========================================================================
    wire signed [15:0] setpoint;
    wire        [15:0] kp, ki, kd;
    wire               pid_enable;
    wire               pid_reset_pulse;

    // Payload wires from eth_rx_parser
    wire        pay_vld;
    wire [7:0]  pay_byte;
    wire [10:0] pay_idx;
    wire        pay_last;

    udp_ctrl_rx u_ctrl (
        .clk        (clk),
        .rst        (rst),
        .pay_vld    (pay_vld),
        .pay_byte   (pay_byte),
        .pay_idx    (pay_idx),
        .pay_last   (pay_last),
        .setpoint   (setpoint),
        .kp         (kp),
        .ki         (ki),
        .kd         (kd),
        .pid_enable (pid_enable),
        .pid_reset  (pid_reset_pulse)
    );

    // =========================================================================
    // 5. PID Controller
    // =========================================================================
    wire signed [15:0] pid_out;
    wire signed [15:0] error_out;
    wire signed [31:0] integral_dbg;

    pid_controller u_pid (
        .clk          (clk),
        .rst          (rst | pid_reset_pulse),
        .sample_tick  (sample_tick),
        .enable       (pid_enable),
        .kp           (kp),
        .ki           (ki),
        .kd           (kd),
        .setpoint     (setpoint),
        .feedback     (feedback),
        .pid_out      (pid_out),
        .error_out    (error_out),
        .integral_dbg (integral_dbg)
    );

    // =========================================================================
    // 6. Motor Bridge  →  PWM Generator
    // =========================================================================
    wire [15:0] pwm_duty;
    wire        motor_dir_int;
    wire        motor_en_int;

    motor_bridge u_bridge (
        .clk       (clk),
        .rst       (rst),
        .pid_out   (pid_out),
        .pwm_duty  (pwm_duty),
        .motor_dir (motor_dir_int),
        .motor_en  (motor_en_int)
    );

    wire pwm_out_int;
    pwm_gen u_pwm (
        .clk     (clk),
        .rst     (rst),
        .duty    (motor_en_int ? pwm_duty : 16'h0),
        .pwm_out (pwm_out_int)
    );

    assign PWM_OUT   = pwm_out_int;
    assign MOTOR_DIR = motor_dir_int;

    // =========================================================================
    // 7. RMII Receive Path
    // =========================================================================
    wire rx_sof, rx_eof, rx_vld;
    wire [7:0] rx_byte;

    rmii_rx u_rmii_rx (
        .clk50    (clk),
        .rst      (rst),
        .crs_dv   (RMII_CRS_DV),
        .rxd      (RMII_RXD),
        .rx_er    (RMII_RX_ER),
        .sof      (rx_sof),
        .eof      (rx_eof),
        .vld      (rx_vld),
        .byte_out (rx_byte)
    );

    // =========================================================================
    // 8. Ethernet RX Parser
    // =========================================================================
    wire        arp_req;
    wire [47:0] arp_src_mac;
    wire [31:0] arp_src_ip;

    eth_rx_parser #(
        .LOCAL_MAC  (LOCAL_MAC),
        .LOCAL_IP   (LOCAL_IP),
        .LOCAL_PORT (LOCAL_PORT)
    ) u_rx_parser (
        .clk              (clk),
        .rst              (rst),
        .rx_sof           (rx_sof),
        .rx_eof           (rx_eof),
        .rx_vld           (rx_vld),
        .rx_byte          (rx_byte),
        .arp_req          (arp_req),
        .arp_src_mac      (arp_src_mac),
        .arp_src_ip       (arp_src_ip),
        .udp_payload_vld  (pay_vld),
        .udp_payload_byte (pay_byte),
        .udp_payload_idx  (pay_idx),
        .udp_payload_last (pay_last)
    );

    // =========================================================================
    // 9. TX Arbiter & Frame Builder
    // =========================================================================

    // Learn destination endpoint from the first UDP command received
    reg [47:0] pc_mac;
    reg [31:0] pc_ip;
    reg [15:0] pc_port;
    always @(posedge clk) begin
        if (rst) begin
            pc_mac  <= 48'hFF_FF_FF_FF_FF_FF;
            pc_ip   <= {8'd10, 8'd10, 8'd10, 8'd10};
            pc_port <= 16'd5005;
        end else if (pay_vld && pay_idx == 0) begin
            // Capture source address from the ARP or first UDP packet
            // (In a full impl, capture from arp_src_mac/ip at arp_req time)
            pc_mac  <= arp_src_mac;
            pc_ip   <= arp_src_ip;
            pc_port <= LOCAL_PORT;
        end
    end

    // ARP handler buffer interface
    wire        arp_tx_req;
    wire [5:0]  arp_buf_idx;
    wire [7:0]  arp_buf_byte;
    wire [5:0]  arp_buf_len;
    wire        tx_busy_builder;

    arp_handler #(
        .LOCAL_MAC (LOCAL_MAC),
        .LOCAL_IP  (LOCAL_IP)
    ) u_arp (
        .clk         (clk),
        .rst         (rst),
        .arp_req     (arp_req),
        .arp_src_mac (arp_src_mac),
        .arp_src_ip  (arp_src_ip),
        .tx_req      (arp_tx_req),
        .tx_busy     (tx_busy_builder),
        .buf_idx     (arp_buf_idx),
        .buf_byte    (arp_buf_byte),
        .buf_len     (arp_buf_len),
        .buf_last    ()  // unused here; builder uses buf_len
    );

    // Telemetry TX buffer interface
    wire        telem_tx_req;
    wire [4:0]  telem_buf_idx;
    wire [7:0]  telem_buf_byte;
    wire [4:0]  telem_buf_len;

    udp_telem_tx #(
        .TELEM_INTERVAL (50)
    ) u_telem (
        .clk         (clk),
        .rst         (rst),
        .sample_tick (sample_tick),
        .setpoint    (setpoint),
        .feedback    (feedback),
        .pid_out     (pid_out),
        .error_in    (error_out),
        .kp          (kp),
        .ki          (ki),
        .kd          (kd),
        .pid_enable  (pid_enable),
        .enc_dir     (enc_dir),
        .dst_mac     (pc_mac),
        .dst_ip      (pc_ip),
        .dst_port    (pc_port),
        .tx_req      (telem_tx_req),
        .tx_busy     (tx_busy_builder),
        .buf_idx     (telem_buf_idx),
        .buf_byte    (telem_buf_byte),
        .buf_len     (telem_buf_len),
        .buf_last    ()
    );

    // RMII TX wires
    wire        rmii_start;
    wire [7:0]  rmii_byte;
    wire        rmii_req;
    wire        rmii_last;
    wire        rmii_tx_busy;

    eth_tx_builder #(
        .LOCAL_MAC  (LOCAL_MAC),
        .LOCAL_IP   (LOCAL_IP),
        .LOCAL_PORT (LOCAL_PORT)
    ) u_tx_builder (
        .clk         (clk),
        .rst         (rst),
        .arp_tx_req  (arp_tx_req),
        .udp_tx_req  (telem_tx_req),
        .tx_busy     (tx_busy_builder),
        .arp_buf_idx (arp_buf_idx),
        .arp_buf_byte(arp_buf_byte),
        .arp_buf_len (arp_buf_len),
        .pay_buf_idx (telem_buf_idx),
        .pay_buf_byte(telem_buf_byte),
        .pay_buf_len (telem_buf_len),
        .dst_mac     (pc_mac),
        .dst_ip      (pc_ip),
        .dst_port    (pc_port),
        .rmii_start  (rmii_start),
        .rmii_byte   (rmii_byte),
        .rmii_req    (rmii_req),
        .rmii_last   (rmii_last),
        .rmii_busy   (rmii_tx_busy)
    );

    // =========================================================================
    // 10. RMII Transmit Path
    // =========================================================================
    rmii_tx u_rmii_tx (
        .clk50     (clk),
        .rst       (rst),
        .start     (rmii_start),
        .tx_byte   (rmii_byte),
        .tx_req    (rmii_req),
        .tx_last   (rmii_last),
        .tx_busy   (rmii_tx_busy),
        .rmii_tx_en(RMII_TX_EN),
        .rmii_txd  (RMII_TXD)
    );

    // =========================================================================
    // 11. LED Indicators (active LOW on Tang Nano 9K)
    // =========================================================================
    // LED1 – Heartbeat at ~1 Hz
    reg [25:0] hb_cnt;
    always @(posedge clk) begin
        if (rst) hb_cnt <= 26'd0;
        else     hb_cnt <= hb_cnt + 1'b1;
    end
    assign LED1 = ~hb_cnt[25];              // toggles at ~0.75 Hz at 50 MHz

    assign LED2 = ~RMII_TX_EN;              // flashes on TX activity
    assign LED3 = ~pid_enable;              // ON when PID is enabled

endmodule

`default_nettype wire
