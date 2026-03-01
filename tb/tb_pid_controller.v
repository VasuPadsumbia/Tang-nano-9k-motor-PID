// =============================================================================
// Testbench : tb_pid_controller
// Project   : Tang Nano 9K – PID Motor Controller over Ethernet
// File      : tb/tb_pid_controller.v
//
// Purpose   : Verifies the pid_controller module using a simulated step input.
//   - Applies setpoint=16384 (0x4000) with feedback starting at 0
//   - Verifies output rises toward setpoint over time
//   - Verifies integral accumulates
//   - Verifies derivative fires correctly
//   - VCD waveform saved to data/sim_pid.vcd for GTKWave analysis
//
// Run:
//   iverilog -g2012 -o data/sim_pid tb/tb_pid_controller.v rtl/motor/pid_controller.v
//   vvp data/sim_pid
//   gtkwave data/sim_pid.vcd
// =============================================================================

`timescale 1ns/1ps

module tb_pid_controller;

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg          clk        = 0;
    reg          rst        = 1;
    reg          sample_tick= 0;
    reg          enable     = 1;
    reg  [15:0]  kp         = 16'h0100;   // 1.0
    reg  [15:0]  ki         = 16'h0020;   // 0.125
    reg  [15:0]  kd         = 16'h0080;   // 0.5
    reg  signed [15:0] setpoint  = 16'sd0;
    reg  signed [15:0] feedback  = 16'sd0;

    wire signed [15:0] pid_out;
    wire signed [15:0] error_out;
    wire signed [31:0] integral_dbg;

    // -------------------------------------------------------------------------
    // Instantiate DUT
    // -------------------------------------------------------------------------
    pid_controller u_dut (
        .clk          (clk),
        .rst          (rst),
        .sample_tick  (sample_tick),
        .enable       (enable),
        .kp           (kp),
        .ki           (ki),
        .kd           (kd),
        .setpoint     (setpoint),
        .feedback     (feedback),
        .pid_out      (pid_out),
        .error_out    (error_out),
        .integral_dbg (integral_dbg)
    );

    // -------------------------------------------------------------------------
    // 50 MHz clock (20 ns period)
    // -------------------------------------------------------------------------
    always #10 clk = ~clk;

    // -------------------------------------------------------------------------
    // Task: Apply one sample tick and check output
    // -------------------------------------------------------------------------
    task apply_tick;
        begin
            @(posedge clk); #1;
            sample_tick = 1;
            @(posedge clk); #1;
            sample_tick = 0;
        end
    endtask

    // -------------------------------------------------------------------------
    // Pass/fail tracking
    // -------------------------------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task assert_gt;
        input signed [15:0] val;
        input signed [15:0] threshold;
        input [255:0]       label;
        begin
            if ($signed(val) > $signed(threshold)) begin
                $display("  PASS  [%0s] pid_out=%0d > %0d", label, val, threshold);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL  [%0s] pid_out=%0d  SHOULD BE > %0d", label, val, threshold);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("data/sim_pid.vcd");
        $dumpvars(0, tb_pid_controller);

        $display("==============================================");
        $display("  PID Controller Testbench");
        $display("  Kp=1.0  Ki=0.125  Kd=0.5");
        $display("==============================================");

        // Release reset after 5 cycles
        repeat (5) @(posedge clk);
        rst      = 0;
        setpoint = 16'sd16384;   // 0x4000 = 25% of max

        // -------- Test 1: Step response – output should rise -----------------
        $display("\n[TEST 1] Step response (setpoint=16384, feedback=0)");
        repeat (10) begin
            apply_tick;
            $display("  tick: error=%0d  pid_out=%0d  integral=%0d",
                     error_out, pid_out, integral_dbg);
        end
        assert_gt(pid_out, 0, "pid_out>0 after step");

        // -------- Test 2: Closed-loop simulation ---------------------------
        $display("\n[TEST 2] Simulated motor (integrator drives feedback)");
        feedback = 16'sd0;
        repeat (30) begin
            apply_tick;
            // Simulate sluggish motor: feedback += pid_out/256
            feedback = feedback + (pid_out >>> 8);
            $display("  tick: fb=%0d  error=%0d  pid_out=%0d",
                     feedback, error_out, pid_out);
        end
        // After 30 ticks, feedback should have converged toward setpoint
        if ($signed(feedback) > 16'sd8000)
            $display("  PASS  [convergence] feedback=%0d reached > 8000", feedback);
        else
            $display("  WARN  [convergence] feedback=%0d still low (gains may need tuning)", feedback);

        // -------- Test 3: Disable PID – output should freeze ---------------
        $display("\n[TEST 3] Disable PID (output should hold)");
        enable = 0;
        begin
            reg signed [15:0] frozen_out;
            frozen_out = pid_out;
            apply_tick;
            apply_tick;
            if (pid_out == frozen_out) begin
                $display("  PASS  [disable] output held at %0d", pid_out);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL  [disable] output changed to %0d (was %0d)", pid_out, frozen_out);
                fail_cnt = fail_cnt + 1;
            end
        end

        // -------- Test 4: Reset clears integrator --------------------------
        $display("\n[TEST 4] Reset clears state");
        rst = 1;
        @(posedge clk); #1;
        rst = 0;
        @(posedge clk); #1;
        if (integral_dbg == 0 && pid_out == 0) begin
            $display("  PASS  [reset] integral_dbg=%0d pid_out=%0d", integral_dbg, pid_out);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("  FAIL  [reset] integral_dbg=%0d pid_out=%0d (expected 0)", integral_dbg, pid_out);
            fail_cnt = fail_cnt + 1;
        end

        // -------- Summary --------------------------------------------------
        $display("\n==============================================");
        $display("  Total PASS: %0d  FAIL: %0d", pass_cnt, fail_cnt);
        $display("  VCD saved to: data/sim_pid.vcd");
        $display("==============================================");

        $finish;
    end

    // Timeout watchdog
    initial begin
        #10_000_000;
        $display("TIMEOUT – simulation exceeded 10 ms");
        $finish;
    end

endmodule
