// Test Bench for PID Motor Controller System
// Tests PID control, auto-tuning, and motor response

`timescale 1ns / 1ps

module main_tb;

    reg CLK = 0;
    wire TX;
    reg RX = 1;
    wire LED1, LED2;
    wire PWM_OUT, MOTOR_DIR;
    
    // Instantiate DUT
    Main dut (
        .CLK(CLK),
        .RX(RX),
        .TX(TX),
        .LED1(LED1),
        .LED2(LED2),
        .PWM_OUT(PWM_OUT),
        .MOTOR_DIR(MOTOR_DIR)
    );
    
    // Generate 12MHz clock
    always #42 CLK = ~CLK;
    
    // Motor model for simulation
    reg [15:0] motor_feedback = 16'h0000;
    reg [15:0] simulated_position = 16'h0000;
    real motor_velocity = 0;
    real motor_acceleration;
    
    initial begin
        $dumpvars(0, main_tb);
        $display("=== PID Motor Controller Test ===");
        $display("Testing: PID control, Auto-tuning, UART interface");
        
        // Initial setup
        repeat(100) @(posedge CLK);
        $display("\n--- Initial State ---");
        $display("LED1: %b, LED2: %b", LED1, LED2);
        $display("Motor PWM: %b, Direction: %b", PWM_OUT, MOTOR_DIR);
        
        // Test 1: Step response with default gains
        $display("\n--- Test 1: Step Response (Default Gains) ---");
        $display("Time\tError\tPID_Out\tFeedback");
        
        // Simulate motor reaching setpoint
        repeat(100_000) @(posedge CLK) begin
            // Simple motor model: first-order system
            motor_acceleration = (PWM_OUT ? 0.001 : -0.0005);
            motor_velocity = motor_velocity + motor_acceleration;
            
            // Limit velocity
            if (motor_velocity > 0.05) motor_velocity = 0.05;
            if (motor_velocity < -0.05) motor_velocity = -0.05;
            
            // Update position
            simulated_position <= simulated_position + $rtoi(motor_velocity * 1000);
            motor_feedback <= simulated_position >>> 10;
            
            // Print periodically
            if ($time % 100_000 == 0) begin
                $display("%0d\t%h\t%h\t%h", $time/1000, 
                         dut.error, dut.pid_output, motor_feedback);
            end
        end
        
        // Test 2: Auto-tuning
        $display("\n--- Test 2: Auto-Tuning Sequence ---");
        $display("Starting auto-tune...");
        send_command(8'h54);  // Send 'T' for auto-tune
        
        repeat(5_000_000) @(posedge CLK) begin
            if (dut.tuning_done) begin
                $display("Tuning complete!");
                $display("Tuned Gains: Kp=%h, Ki=%h, Kd=%h",
                         dut.kp_tune, dut.ki_tune, dut.kd_tune);
                break;
            end
        end
        
        // Test 3: Step response with tuned gains
        $display("\n--- Test 3: Step Response (Tuned Gains) ---");
        $display("Applying step input...");
        motor_feedback <= 0;
        simulated_position <= 0;
        motor_velocity = 0;
        
        repeat(200_000) @(posedge CLK) begin
            // Motor model update
            motor_acceleration = (PWM_OUT ? 0.002 : -0.0008);
            motor_velocity = motor_velocity + motor_acceleration;
            if (motor_velocity > 0.08) motor_velocity = 0.08;
            if (motor_velocity < -0.08) motor_velocity = -0.08;
            
            simulated_position <= simulated_position + $rtoi(motor_velocity * 1000);
            motor_feedback <= simulated_position >>> 10;
            
            if ($time % 100_000 == 0) begin
                $display("Time: %0d ns, Error: %h, PWM: %b, Pos: %h",
                         $time, dut.error, PWM_OUT, motor_feedback);
            end
        end
        
        // Test 4: UART parameter update
        $display("\n--- Test 4: UART Parameter Update ---");
        $display("Sending new Kp value via UART...");
        send_hex_command(8'h4B, 16'h0200);  // Set Kp to 2.0
        
        repeat(100_000) @(posedge CLK);
        
        // Final status
        $display("\n--- Final Status ---");
        $display("LED1: %b (tuning complete indicator)", LED1);
        $display("LED2: %b (heartbeat)", LED2);
        $display("Motor Direction: %b", MOTOR_DIR);
        $display("Current Kp: %h", dut.kp_reg);
        $display("Current Ki: %h", dut.ki_reg);
        $display("Current Kd: %h", dut.kd_reg);
        
        $display("\n=== Test Completed ===");
        $finish;
    end
    
    // Helper task to send command via UART
    task send_command(input [7:0] cmd);
        integer i;
        begin
            // Start bit
            RX = 0;
            repeat(16) @(posedge CLK);
            
            // Data bits
            for (i = 0; i < 8; i = i + 1) begin
                RX = cmd[i];
                repeat(16) @(posedge CLK);
            end
            
            // Stop bit
            RX = 1;
            repeat(16) @(posedge CLK);
        end
    endtask
    
    // Helper task to send hex command
    task send_hex_command(input [7:0] cmd, input [15:0] value);
        begin
            send_command(cmd);
            send_hex_byte(value[15:8]);
            send_hex_byte(value[7:0]);
            send_command(8'h0D);  // Enter key
        end
    endtask
    
    // Helper task to send hex byte
    task send_hex_byte(input [7:0] hex_val);
        integer i;
        reg [3:0] nibble;
        begin
            for (i = 1; i >= 0; i = i - 1) begin
                nibble = (i == 1) ? hex_val[7:4] : hex_val[3:0];
                if (nibble < 10)
                    send_command(nibble + 8'h30);  // '0'-'9'
                else
                    send_command(nibble + 8'h37);  // 'A'-'F'
            end
        end
    endtask

endmodule
