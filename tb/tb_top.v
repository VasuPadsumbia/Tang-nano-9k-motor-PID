// =============================================================================
// Testbench : tb_top  (Enhanced with Communication Tests)
// Project   : Tang Nano 9K – PID Motor Controller over Ethernet
// File      : tb/tb_top.v
//
// Purpose   : Full top-level testbench covering:
//   SYSTEM TESTS:
//     1. POR reset held while BTN1 low
//     2. POR reset releases after BTN1 high
//     3. LED1 heartbeat counter increments
//     4. PWM = 0 when PID disabled
//     5. Sample tick fires at ~1 kHz rate
//
//   COMMUNICATION TESTS (RMII frame injection):
//     6. ARP who-has → FPGA replies (RMII_TX_EN asserts)
//     7. UDP setpoint command → FPGA setpoint register updates
//     8. UDP Kp gain command → gain register updates
//     9. UDP enable command  → pid_enable asserts
//
//   All waveforms → data/sim_top.vcd
//   Telemetry log → data/sim_top_telem.csv
//
// Run (from project root):
//   iverilog -g2012 -o data/sim_top \
//     tb/tb_top.v rtl/top.v \
//     rtl/sys/clk_div.v \
//     rtl/motor/pid_controller.v rtl/motor/motor_bridge.v \
//     rtl/motor/pwm_gen.v rtl/motor/encoder_reader.v \
//     rtl/comm/rmii_rx.v rtl/comm/rmii_tx.v rtl/comm/eth_crc32.v \
//     rtl/comm/eth_rx_parser.v rtl/comm/eth_tx_builder.v \
//     rtl/comm/arp_handler.v rtl/comm/udp_ctrl_rx.v rtl/comm/udp_telem_tx.v
//   vvp data/sim_top
//   gtkwave data/sim_top.vcd
// =============================================================================

`timescale 1ns/1ps

module tb_top;

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg        CLK         = 0;
    reg        CLK50       = 0;
    reg        BTN1        = 0;     // active LOW reset
    reg        RMII_CRS_DV = 0;
    reg  [1:0] RMII_RXD    = 2'b00;
    reg        RMII_RX_ER  = 0;
    reg        ENC_A       = 0;
    reg        ENC_B       = 0;

    wire       RMII_TX_EN;
    wire [1:0] RMII_TXD;
    wire       RMII_MDC;
    wire       PWM_OUT;
    wire       MOTOR_DIR;
    wire       LED1, LED2, LED3;

    // =========================================================================
    // DUT
    // =========================================================================
    top #(
        .LOCAL_MAC  (48'h02_12_34_56_78_9A),
        .LOCAL_IP   ({8'd10, 8'd10, 8'd10, 8'd100}),
        .LOCAL_PORT (16'd5005)
    ) u_dut (
        .CLK         (CLK),
        .CLK50       (CLK50),
        .BTN1        (BTN1),
        .RMII_CRS_DV (RMII_CRS_DV),
        .RMII_RXD    (RMII_RXD),
        .RMII_RX_ER  (RMII_RX_ER),
        .RMII_TX_EN  (RMII_TX_EN),
        .RMII_TXD    (RMII_TXD),
        .RMII_MDC    (RMII_MDC),
        .RMII_MDIO   (1'b1),
        .ENC_A       (ENC_A),
        .ENC_B       (ENC_B),
        .PWM_OUT     (PWM_OUT),
        .MOTOR_DIR   (MOTOR_DIR),
        .LED1        (LED1),
        .LED2        (LED2),
        .LED3        (LED3)
    );

    // =========================================================================
    // Clocks  (50 MHz RMII REF_CLK, 27 MHz onboard)
    // =========================================================================
    always #10 CLK50 = ~CLK50;   // 50 MHz (20 ns period)
    always #18 CLK   = ~CLK;     // ~27 MHz

    // =========================================================================
    // Frame buffer for RMII injection
    // =========================================================================
    reg [7:0] frame_buf [0:127];
    integer   frame_len;

    // -------------------------------------------------------------------------
    // Task: emit one byte as 4 RMII dibits (LSB first)
    // -------------------------------------------------------------------------
    task rmii_send_byte;
        input [7:0] b;
        begin
            RMII_RXD = b[1:0]; @(posedge CLK50); #2;
            RMII_RXD = b[3:2]; @(posedge CLK50); #2;
            RMII_RXD = b[5:4]; @(posedge CLK50); #2;
            RMII_RXD = b[7:6]; @(posedge CLK50); #2;
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: transmit the bytes in frame_buf[0..frame_len-1] as an RMII frame
    //       Preamble: 28× 2'b01, SFD end: 2'b11, data bytes, EoF by CRS_DV=0
    //       FCS: not computed (parser does not verify FCS)
    // -------------------------------------------------------------------------
    task rmii_send_frame;
        integer i;
        begin
            // Preamble (28 dibits = 7 bytes × 4 dibits of 0x55)
            RMII_CRS_DV = 1'b1;
            for (i = 0; i < 28; i = i + 1) begin
                RMII_RXD = 2'b01;
                @(posedge CLK50); #2;
            end
            // SFD last 4 dibits: 0xD5 = 01,01,01,11  (LSB first)
            RMII_RXD = 2'b01; @(posedge CLK50); #2;
            RMII_RXD = 2'b01; @(posedge CLK50); #2;
            RMII_RXD = 2'b01; @(posedge CLK50); #2;
            RMII_RXD = 2'b11; @(posedge CLK50); #2;   // SFD end → parser enters DATA

            // Data bytes
            for (i = 0; i < frame_len; i = i + 1)
                rmii_send_byte(frame_buf[i]);

            // End of frame: deassert CRS_DV
            RMII_CRS_DV = 1'b0;
            RMII_RXD    = 2'b00;
            // Append dummy FCS (4 bytes of 0x00 – parser won't check)
            // Note: CRS_DV kept HIGH for FCS dummy bytes
            RMII_CRS_DV = 1'b1;
            rmii_send_byte(8'h00);
            rmii_send_byte(8'h00);
            rmii_send_byte(8'h00);
            rmii_send_byte(8'h00);
            RMII_CRS_DV = 1'b0;
            RMII_RXD    = 2'b00;
            repeat (8) @(posedge CLK50);   // inter-frame gap
        end
    endtask

    // -------------------------------------------------------------------------
    // Build ARP who-has frame in frame_buf
    //   Src MAC = AA:BB:CC:DD:EE:FF  (simulated PC)
    //   Src IP  = 10.10.10.10
    //   Target  = 10.10.10.100  (FPGA IP)
    // -------------------------------------------------------------------------
    task build_arp_request;
        begin
            // Dst MAC: broadcast
            frame_buf[0]  = 8'hFF; frame_buf[1]  = 8'hFF; frame_buf[2]  = 8'hFF;
            frame_buf[3]  = 8'hFF; frame_buf[4]  = 8'hFF; frame_buf[5]  = 8'hFF;
            // Src MAC: simulated PC
            frame_buf[6]  = 8'hAA; frame_buf[7]  = 8'hBB; frame_buf[8]  = 8'hCC;
            frame_buf[9]  = 8'hDD; frame_buf[10] = 8'hEE; frame_buf[11] = 8'hFF;
            // EtherType = ARP (0x0806)
            frame_buf[12] = 8'h08; frame_buf[13] = 8'h06;
            // HTYPE=1 (Ethernet), PTYPE=0x0800 (IPv4)
            frame_buf[14] = 8'h00; frame_buf[15] = 8'h01;
            frame_buf[16] = 8'h08; frame_buf[17] = 8'h00;
            // HLEN=6, PLEN=4
            frame_buf[18] = 8'h06; frame_buf[19] = 8'h04;
            // OPER = 0x0001 (request)
            frame_buf[20] = 8'h00; frame_buf[21] = 8'h01;
            // SHA = PC MAC
            frame_buf[22] = 8'hAA; frame_buf[23] = 8'hBB; frame_buf[24] = 8'hCC;
            frame_buf[25] = 8'hDD; frame_buf[26] = 8'hEE; frame_buf[27] = 8'hFF;
            // SPA = PC IP (10.10.10.10)
            frame_buf[28] = 8'h0A; frame_buf[29] = 8'h0A;
            frame_buf[30] = 8'h0A; frame_buf[31] = 8'h0A;
            // THA = zeros
            frame_buf[32] = 8'h00; frame_buf[33] = 8'h00; frame_buf[34] = 8'h00;
            frame_buf[35] = 8'h00; frame_buf[36] = 8'h00; frame_buf[37] = 8'h00;
            // TPA = FPGA IP (10.10.10.100 = 0x0A0A0A64)
            frame_buf[38] = 8'h0A; frame_buf[39] = 8'h0A;
            frame_buf[40] = 8'h0A; frame_buf[41] = 8'h64;
            frame_len = 42;
        end
    endtask

    // -------------------------------------------------------------------------
    // Build UDP command frame in frame_buf
    //   Cmd 0x01 (setpoint), value = 0x4000 (16384)
    // -------------------------------------------------------------------------
    task build_udp_cmd;
        input [7:0] cmd_id;
        input [15:0] cmd_val;
        // IP total length = 20 + 8 + 6 = 34 = 0x0022
        // UDP length      =      8 + 6 = 14 = 0x000E
        begin
            // Dst MAC: FPGA
            frame_buf[0]  = 8'h02; frame_buf[1]  = 8'h12; frame_buf[2]  = 8'h34;
            frame_buf[3]  = 8'h56; frame_buf[4]  = 8'h78; frame_buf[5]  = 8'h9A;
            // Src MAC: PC
            frame_buf[6]  = 8'hAA; frame_buf[7]  = 8'hBB; frame_buf[8]  = 8'hCC;
            frame_buf[9]  = 8'hDD; frame_buf[10] = 8'hEE; frame_buf[11] = 8'hFF;
            // EtherType = IPv4 (0x0800)
            frame_buf[12] = 8'h08; frame_buf[13] = 8'h00;
            // ----- IPv4 Header (20 bytes, offset 14) -----
            frame_buf[14] = 8'h45;  // Version=4, IHL=5
            frame_buf[15] = 8'h00;  // DSCP/ECN
            frame_buf[16] = 8'h00; frame_buf[17] = 8'h22;  // Total len = 34
            frame_buf[18] = 8'h00; frame_buf[19] = 8'h01;  // ID
            frame_buf[20] = 8'h40; frame_buf[21] = 8'h00;  // Flags: DF
            frame_buf[22] = 8'h40;  // TTL=64
            frame_buf[23] = 8'h11;  // Protocol: UDP
            frame_buf[24] = 8'h00; frame_buf[25] = 8'h00;  // Checksum (zero = skip)
            frame_buf[26] = 8'h0A; frame_buf[27] = 8'h0A;  // Src IP: 10.10.10.10
            frame_buf[28] = 8'h0A; frame_buf[29] = 8'h0A;
            frame_buf[30] = 8'h0A; frame_buf[31] = 8'h0A;  // Dst IP: 10.10.10.100
            frame_buf[32] = 8'h0A; frame_buf[33] = 8'h64;
            // ----- UDP Header (8 bytes, offset 34) -----
            frame_buf[34] = 8'h13; frame_buf[35] = 8'h8D;  // Src port 5005
            frame_buf[36] = 8'h13; frame_buf[37] = 8'h8D;  // Dst port 5005
            frame_buf[38] = 8'h00; frame_buf[39] = 8'h0E;  // UDP len = 14
            frame_buf[40] = 8'h00; frame_buf[41] = 8'h00;  // Checksum = 0
            // ----- Payload (6 bytes, offset 42) -----
            frame_buf[42] = cmd_id;
            frame_buf[43] = cmd_val[15:8];
            frame_buf[44] = cmd_val[7:0];
            frame_buf[45] = 8'h00;
            frame_buf[46] = 8'h00;
            frame_buf[47] = 8'h00;
            frame_len = 48;
        end
    endtask

    // =========================================================================
    // Telemetry CSV logger (runs every sample_tick)
    // =========================================================================
    integer csv_fd;
    integer tick_cnt = 0;

    always @(posedge CLK50) begin
        if (u_dut.sample_tick) begin
            tick_cnt = tick_cnt + 1;
            $fwrite(csv_fd, "%0d,%0d,%0d,%0d,%0d,0x%04X,0x%04X,0x%04X,%0b\n",
                tick_cnt,
                $signed(u_dut.setpoint),
                $signed(u_dut.feedback),
                $signed(u_dut.pid_out),
                $signed(u_dut.error_out),
                u_dut.kp, u_dut.ki, u_dut.kd,
                u_dut.pid_enable);
        end
    end

    // =========================================================================
    // Pass/fail tracking
    // =========================================================================
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task chk;
        input cond;
        input [1023:0] msg;
        begin
            if (cond) begin
                $display("  PASS  %0s", msg);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL  %0s", msg);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // Wait up to N cycles for a signal to become expected value
    task wait_for_signal;
        input      expected;
        input reg  signal;
        input integer timeout_cycles;
        input [255:0] label;
        integer i;
        reg found;
        begin
            found = 0;
            for (i = 0; i < timeout_cycles && !found; i = i + 1) begin
                @(posedge CLK50);
                if (signal === expected) found = 1;
            end
            chk(found, label);
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        $dumpfile("data/sim_top.vcd");
        $dumpvars(0, tb_top);

        csv_fd = $fopen("data/sim_top_telem.csv", "w");
        $fwrite(csv_fd, "tick,setpoint,feedback,pid_out,error,kp,ki,kd,enabled\n");

        $display("==========================================================");
        $display("  Top-Level Testbench  (System + Communication Tests)");
        $display("==========================================================");

        // =====================================================================
        // SECTION 1: SYSTEM TESTS
        // =====================================================================
        $display("\n──── SYSTEM TESTS ────────────────────────────────────────");

        // --- Test 1: Reset held -------------------------------------------
        $display("\n[T1] POR reset asserted (BTN1=0)");
        BTN1 = 0;
        repeat (20) @(posedge CLK50);
        chk(u_dut.rst === 1'b1, "rst=1 while BTN1 held low");

        // --- Test 2: Reset releases after POR counter --------------------
        $display("\n[T2] POR reset releases (waiting ~1.1M cycles)...");
        BTN1 = 1;
        repeat (1_100_000) @(posedge CLK50);
        chk(u_dut.rst === 1'b0, "rst=0 after POR counter completes");

        // --- Test 3: Heartbeat counter active ----------------------------
        $display("\n[T3] Heartbeat LED counter incrementing");
        chk(u_dut.hb_cnt > 0, "hb_cnt > 0 after reset release");

        // --- Test 4: PWM = 0 when PID disabled (default) ----------------
        $display("\n[T4] PWM duty = 0 while PID disabled");
        @(posedge CLK50); #1;
        chk(u_dut.pwm_duty === 16'h0, "pwm_duty=0 (PID not yet enabled)");
        chk(PWM_OUT === 1'b0,         "PWM_OUT=0  (motor not driven)");

        // --- Test 5: Sample tick rate ~1 kHz -----------------------------
        $display("\n[T5] Sample tick rate check (100ms window)");
        begin : tick_check
            integer start_tick;
            integer delta;
            start_tick = tick_cnt;
            repeat (5_000_000) @(posedge CLK50);  // 100 ms @ 50 MHz
            delta = tick_cnt - start_tick;
            chk((delta >= 95) && (delta <= 105),
                "tick delta in [95,105] over 100ms");
            $display("  Info: tick delta = %0d (expected ~100)", delta);
        end

        // =====================================================================
        // SECTION 2: COMMUNICATION TESTS
        // =====================================================================
        $display("\n──── COMMUNICATION TESTS ─────────────────────────────────");

        // --- Test 6: ARP who-has → FPGA TX_EN asserts --------------------
        $display("\n[T6] ARP request → expect RMII_TX_EN (ARP reply)");
        build_arp_request;
        rmii_send_frame;
        // Give ARP handler time to queue and TX builder to start
        // ARP reply should begin within a few hundred cycles
        begin : arp_check
            integer waited;
            reg     got_tx;
            got_tx = 0;
            for (waited = 0; waited < 10_000 && !got_tx; waited = waited + 1) begin
                @(posedge CLK50);
                if (RMII_TX_EN === 1'b1) got_tx = 1;
            end
            chk(got_tx, "RMII_TX_EN asserted within 10000 cycles after ARP RX");
            // Wait for TX to complete
            if (got_tx)
                repeat (2000) @(posedge CLK50);
        end

        // --- Test 7: UDP setpoint command → setpoint register updates ----
        $display("\n[T7] UDP CMD 0x01 (setpoint=16384 / 0x4000)");
        build_udp_cmd(8'h01, 16'h4000);
        rmii_send_frame;
        // Parser + ctrl_rx need ~100 cycles after EoF to latch
        repeat (500) @(posedge CLK50);
        chk(u_dut.setpoint === 16'sh4000,
            "setpoint register = 0x4000 after CMD_SETPOINT");
        $display("  Info: u_dut.setpoint = %0d (0x%04X)",
                 $signed(u_dut.setpoint), u_dut.setpoint);

        // --- Test 8: UDP Kp gain command → kp register updates -----------
        $display("\n[T8] UDP CMD 0x02 (Kp=0x0200 = 2.0)");
        build_udp_cmd(8'h02, 16'h0200);
        rmii_send_frame;
        repeat (500) @(posedge CLK50);
        chk(u_dut.kp === 16'h0200,
            "kp register = 0x0200 after CMD_KP");
        $display("  Info: u_dut.kp = 0x%04X", u_dut.kp);

        // --- Test 9: UDP enable command → pid_enable asserts -------------
        $display("\n[T9] UDP CMD 0x05 (enable PID)");
        build_udp_cmd(8'h05, 16'h0001);
        rmii_send_frame;
        repeat (500) @(posedge CLK50);
        chk(u_dut.pid_enable === 1'b1,
            "pid_enable=1 after CMD_ENABLE");
        chk(LED3 === 1'b0,              // LED3 active-LOW = ON when enabled
            "LED3=0 (active, PID enabled)");

        // --- Test 10:  UDP disable command -------------------------------
        $display("\n[T10] UDP CMD 0x05 (disable PID)");
        build_udp_cmd(8'h05, 16'h0000);
        rmii_send_frame;
        repeat (500) @(posedge CLK50);
        chk(u_dut.pid_enable === 1'b0, "pid_enable=0 after CMD_DISABLE");

        // --- Test 11: UDP reset command → integrator clears --------------
        $display("\n[T11] UDP CMD 0x06 (reset PID integrator)");
        // First enable and run a few ticks with non-zero setpoint
        build_udp_cmd(8'h05, 16'h0001);  // enable
        rmii_send_frame;
        repeat (200) @(posedge CLK50);
        repeat (50) @(posedge CLK50);    // let integrator accumulate
        // Now send reset
        build_udp_cmd(8'h06, 16'h0000);
        rmii_send_frame;
        repeat (500) @(posedge CLK50);
        chk(u_dut.u_pid.integrator === 32'sd0,
            "integrator=0 after CMD_RESET");

        // =====================================================================
        // SUMMARY
        // =====================================================================
        $fclose(csv_fd);
        $display("\n==========================================================");
        $display("  TOTAL  PASS: %0d  FAIL: %0d", pass_cnt, fail_cnt);
        $display("  VCD saved  : data/sim_top.vcd");
        $display("  CSV log    : data/sim_top_telem.csv");
        $display("==========================================================");
        if (fail_cnt == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  *** %0d TEST(S) FAILED – check signals in GTKWave ***",
                     fail_cnt);
        $finish;
    end

    // Timeout watchdog (300 ms sim time)
    initial begin
        #300_000_000_000;
        $display("TIMEOUT – simulation exceeded 300 ms");
        $fclose(csv_fd);
        $finish;
    end

endmodule
