/* // Top-Level Motor PID Controller System
// Integrates all modules: PID, Auto-tuner, UART, Motor Control

module Main (
    input CLK,      // 12MHz clock
    input RX,       // UART RX
    output TX,      // UART TX
    output LED1,    // Status LED (tuning progress)
    output LED2,    // Activity LED (heartbeat)
    output LED3,    // Simulation output (PWM in sim mode)
    
    // Motor outputs (optional - can be used with external driver)
    output PWM_OUT,
    output MOTOR_DIR
);

    // Internal signals
    wire reset = 0;  // Active low reset (no external reset button)
    
    // PID signals
    wire [15:0] kp, ki, kd;
    wire [15:0] setpoint, feedback;
    wire [15:0] error;
    wire [15:0] pid_output;
    wire [31:0] integral_sum;
    wire [15:0] prev_error;
    
    // Auto-tuner signals
    wire [15:0] kp_tune, ki_tune, kd_tune;
    wire tuning_done;
    wire [7:0] tuning_progress;
    wire start_autotune;
    
    // UART signals
    wire [15:0] kp_rx, ki_rx, kd_rx, setpoint_rx, feedback_rx;
    wire load_params, uart_autotune;
    wire [7:0] rx_data;
    wire rx_valid;
    wire mode_valid;
    wire [7:0] mode_char;
    wire tx_start, tx_ready;
    wire [7:0] tx_char;
    
    // Motor signals
    wire [9:0] pwm_duty;
    wire [15:0] motor_speed;
    
    // Status signals
    reg [31:0] cycle_counter;
    reg [31:0] control_timer;
    reg pid_enable;
    
    // Register file for PID parameters
    reg [15:0] kp_reg = 16'h0100;    // Default Kp = 1.0 (scaled by 256)
    reg [15:0] ki_reg = 16'h0020;    // Default Ki = 0.125 (scaled by 256)
    reg [15:0] kd_reg = 16'h0080;    // Default Kd = 0.5 (scaled by 256)
    reg [15:0] setpoint_reg = 16'h0000;
    reg [15:0] feedback_reg = 16'h0000;
    
    // LED control for status indication
    reg led1_state, led2_state, led3_state;
    
    // Hardware vs Simulation mode
    reg sim_mode = 0;  // 0=hardware, 1=simulation
    reg mode_selected = 0;
    reg startup_done = 0;
    
    // Serial output/debug telemetry
    reg [31:0] debug_timer;
    reg [15:0] debug_counter;
    
    // Simulation motor state
    reg [15:0] sim_velocity = 0;
    reg [15:0] sim_position = 0;
    reg [15:0] motor_feedback;
    wire [15:0] sim_pid_output;
    
    // Instantiate modules
    pid_controller pid_inst (
        .clk(CLK),
        .reset(reset),
        .kp(kp),
        .ki(ki),
        .kd(kd),
        .error_in(error),
        .enable(pid_enable),
        .pid_output(pid_output),
        .integral_sum(integral_sum),
        .prev_error(prev_error)
    );
    
    auto_tuner tuner_inst (
        .clk(CLK),
        .reset(reset),
        .error(error),
        .start_tune(start_autotune),
        .kp_tune(kp_tune),
        .ki_tune(ki_tune),
        .kd_tune(kd_tune),
        .tuning_done(tuning_done),
        .tuning_progress(tuning_progress)
    );
    
    uart_interface uart_inst (
        .clk(CLK),
        .reset(reset),
        .rx(RX),
        .tx(TX),
        .tx_start(tx_start),
        .tx_char(tx_char),
        .tx_ready(tx_ready),
        .kp_rx(kp_rx),
        .ki_rx(ki_rx),
        .kd_rx(kd_rx),
        .setpoint_rx(setpoint_rx),
        .feedback_rx(feedback_rx),
        .load_params(load_params),
        .start_autotune(start_autotune),
        .pid_output_dbg(pid_output),
        .integral_dbg(integral_sum),
        .error_dbg(error),
        .tuning_progress_dbg(tuning_progress),
        .tuning_done_dbg(tuning_done),
        .uart_busy(),
        .rx_data(rx_data),
        .rx_valid(rx_valid),
        .mode_valid(mode_valid),
        .mode_char(mode_char)
    );
    
    motor_control motor_inst (
        .clk(CLK),
        .reset(reset),
        .pid_output(sim_mode ? sim_pid_output : pid_output),
        .pwm_out(PWM_OUT),
        .motor_direction(MOTOR_DIR),
        .pwm_duty(pwm_duty),
        .motor_speed(motor_speed)
    );
    
    // Debug output module - sends telemetry over UART
    debug_output debug_inst (
        .clk(CLK),
        .reset(reset),
        .tx_start(tx_start),
        .tx_char(tx_char),
        .tx_ready(tx_ready),
        .pid_output(pid_output),
        .error(error),
        .setpoint(setpoint_reg),
        .feedback(motor_feedback),
        .kp(kp),
        .ki(ki),
        .kd(kd),
        .tuning_progress(tuning_progress),
        .tuning_done(tuning_done),
        .sim_mode(sim_mode),
        .sim_velocity(sim_velocity),
        .sim_position(sim_position),
        .startup_done(startup_done),
        .send_telemetry(1'b0)
    );
    
    // In simulation mode, use pid_output directly for LED3 visualization
    assign sim_pid_output = pid_output;
    
    // Calculate error signal
    assign error = setpoint - (sim_mode ? sim_position : feedback);
    
    // In simulation mode, use simulated feedback
    always @(*) begin
        if (sim_mode)
            motor_feedback = sim_position;
        else
            motor_feedback = feedback_reg;
    end
    
    // Use registered or tuned parameters
    assign kp = (tuning_done) ? kp_tune : kp_reg;
    assign ki = (tuning_done) ? ki_tune : ki_reg;
    assign kd = (tuning_done) ? kd_tune : kd_reg;
    assign setpoint = setpoint_reg;
    assign feedback = motor_feedback;
    
    // Main control loop
    always @(posedge CLK) begin
        // Startup sequence - wait for user mode selection
        if (!startup_done) begin
            // LED2 pulses to indicate waiting for user input
            led2_state <= cycle_counter[20];  // Fast pulse while waiting
            
            // Check for user mode selection via UART (mode_valid asserted on single-char+Enter)
            if (mode_valid) begin
                if (mode_char == 8'h48) begin  // 'H' for Hardware mode
                    sim_mode <= 0;
                    mode_selected <= 1;
                    startup_done <= 1;
                end else if (mode_char == 8'h53) begin  // 'S' for Simulation mode
                    sim_mode <= 1;
                    mode_selected <= 1;
                    startup_done <= 1;
                end
            end
        end
        
        cycle_counter <= cycle_counter + 1;
        
        // PID update rate: 1kHz (every 12,000 cycles at 12MHz)
        // ONLY update after startup_done to prevent early computation
        if (startup_done) begin
            if (control_timer == 12_000) begin
                control_timer <= 0;
                pid_enable <= 1;
            end else begin
                control_timer <= control_timer + 1;
                pid_enable <= 0;
            end
        end else begin
            pid_enable <= 0;  // Keep disabled during startup
        end
        
        // Simulation motor model (first-order system with damping)
        if (sim_mode && startup_done) begin
            // Simple first-order motor model: acceleration proportional to pid_output
            // Velocity = Velocity + (PID_OUTPUT >> 10) / 2^7 (acts as acceleration)
            if (pid_output[15] == 0) begin  // Positive direction
                sim_velocity <= sim_velocity + {{7'b0000000}, pid_output[15:7]};
            end else begin  // Negative direction
                sim_velocity <= sim_velocity - {{7'b0000000}, (~pid_output[15:7]) + 9'b1};
            end
            
            // Damping: reduce velocity over time (friction simulation)
            if (sim_velocity[15] == 0) begin
                sim_velocity <= (sim_velocity > 256) ? (sim_velocity - 256) : 0;
            end else begin
                sim_velocity <= (sim_velocity < -256) ? (sim_velocity + 256) : 0;
            end
            
            // Integrate velocity to get position
            sim_position <= sim_position + (sim_velocity >>> 8);
            
            // Drive LED3 with PWM duty cycle (visual feedback)
            led3_state <= (cycle_counter[9:0] < pwm_duty[9:0]) ? 1 : 0;
        end else begin
            // Hardware mode - drive actual motor
            led3_state <= 0;
        end
        
        // Update parameters from UART
        if (load_params) begin
            kp_reg <= kp_rx;
            ki_reg <= ki_rx;
            kd_reg <= kd_rx;
        end
        
        // Update setpoint and feedback from UART
        if (rx_valid) begin
            if (rx_data == 8'h53 && !startup_done) begin  // 'S' for Simulation mode (startup only)
                // Handled in startup sequence
            end else if (rx_data == 8'h48 && !startup_done) begin  // 'H' for Hardware mode (startup only)
                // Handled in startup sequence
            end
        end
        
        // LED status indicators
        if (!startup_done) begin
            led2_state <= cycle_counter[20];  // Fast blink while waiting for mode selection
            led1_state <= 0;  // Keep LED1 off during startup
            led3_state <= 0;  // Keep LED3 off during startup
        end else begin
            led2_state <= cycle_counter[21];  // Slow blink - heartbeat after startup
            
            // LED1: Tuning status (only after startup)
            if (tuning_done) begin
                led1_state <= 1;  // LED1 on during tuning complete
            end else if (start_autotune) begin
                led1_state <= cycle_counter[20];  // Fast blink during tuning
            end else begin
                led1_state <= 0;
            end
        end
        
        // Debug output: send telemetry every 120,000 cycles (~10Hz at 12MHz)
        debug_timer <= debug_timer + 1;
        if (debug_timer == 32'd120_000) begin
            debug_timer <= 0;
            debug_counter <= debug_counter + 1;
        end
    end
    
    // Assign LEDs and Motor outputs
    assign LED1 = led1_state;
    assign LED2 = led2_state;
    assign LED3 = led3_state;
    // Motor outputs already routed from motor_inst instantiation

endmodule
 */