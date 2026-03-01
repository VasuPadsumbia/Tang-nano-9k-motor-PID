// =============================================================================
// Testbench : tb_pwm_gen
// Project   : Tang Nano 9K – PID Motor Controller over Ethernet
// File      : tb/tb_pwm_gen.v
//
// Purpose   : Verifies PWM generator duty cycle at several test values.
//   Counts HIGH and LOW periods over exactly one 16-bit counter cycle (65536
//   clock edges) and checks they match the expected duty cycle.
//
//   VCD waveform saved to data/sim_pwm.vcd
//
// Run:
//   iverilog -g2012 -o data/sim_pwm tb/tb_pwm_gen.v rtl/motor/pwm_gen.v
//   vvp data/sim_pwm
//   gtkwave data/sim_pwm.vcd
// =============================================================================

`timescale 1ns/1ps

module tb_pwm_gen;

    reg        clk  = 0;
    reg        rst  = 1;
    reg [15:0] duty = 16'h0;
    wire       pwm_out;

    pwm_gen u_dut (
        .clk     (clk),
        .rst     (rst),
        .duty    (duty),
        .pwm_out (pwm_out)
    );

    // 50 MHz clock
    always #10 clk = ~clk;

    // -------------------------------------------------------------------------
    // Count high cycles over one full 16-bit counter period (65536 clocks)
    // -------------------------------------------------------------------------
    integer high_cnt;
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check_duty;
        input [15:0] set_duty;
        input [15:0] tolerance;
        integer i;
        begin
            duty     = set_duty;
            high_cnt = 0;

            // Wait for counter to roll-over (sync start)
            @(posedge clk); #1;

            // Count 65536 consecutive cycles
            for (i = 0; i < 65536; i = i + 1) begin
                @(posedge clk);
                if (pwm_out) high_cnt = high_cnt + 1;
            end

            if ((set_duty == 0 && high_cnt == 0) ||
                (set_duty != 0 &&
                 high_cnt >= set_duty - tolerance &&
                 high_cnt <= set_duty + tolerance)) begin
                $display("  PASS  duty=0x%04X  measured_high=%0d  expected=%0d",
                         set_duty, high_cnt, set_duty);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL  duty=0x%04X  measured_high=%0d  expected=%0d  (±%0d)",
                         set_duty, high_cnt, set_duty, tolerance);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("data/sim_pwm.vcd");
        $dumpvars(0, tb_pwm_gen);

        $display("==============================================");
        $display("  PWM Generator Testbench");
        $display("  Tolerance: ±2 counts out of 65536");
        $display("==============================================");

        // Release reset
        repeat (4) @(posedge clk);
        rst = 0;
        repeat (4) @(posedge clk);

        // Test cases
        check_duty(16'h0000,  2);   //   0%
        check_duty(16'h4000,  2);   //  25%
        check_duty(16'h8000,  2);   //  50%
        check_duty(16'hC000,  2);   //  75%
        check_duty(16'hFFFF,  2);   // ~100%

        $display("\n==============================================");
        $display("  Total PASS: %0d  FAIL: %0d", pass_cnt, fail_cnt);
        $display("  VCD saved to: data/sim_pwm.vcd");
        $display("==============================================");
        $finish;
    end

    initial begin
        #500_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
