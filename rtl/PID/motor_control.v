// Motor Control Module - PWM output with direction control

module motor_control #(
    parameter integer PWM_WIDTH = 10  // 10-bit PWM (0-1023)
) (
    input clk,
    input reset,
    
    // Control signals from PID
    input signed [15:0] pid_output,
    
    // Motor outputs
    output pwm_out,
    output motor_direction,  // 1 = forward, 0 = reverse
    
    // Debug outputs
    output [PWM_WIDTH-1:0] pwm_duty,
    output [15:0] motor_speed
);

    reg [PWM_WIDTH-1:0] pwm_counter;
    reg [PWM_WIDTH-1:0] duty_cycle;
    reg direction;
    
    always @(posedge clk) begin
        if (reset) begin
            pwm_counter <= 0;
            duty_cycle <= 0;
            direction <= 0;
        end else begin
            // PWM counter
            if (pwm_counter == {PWM_WIDTH{1'b1}}) begin
                pwm_counter <= 0;
            end else begin
                pwm_counter <= pwm_counter + 1;
            end
            
            // Extract magnitude and direction from PID output
            if (pid_output[15]) begin
                // Negative - reverse direction
                direction <= 0;
                begin : scale_neg
                    reg [15:0] scaled_val;
                    scaled_val = (~pid_output + 1) >>> 6;
                    if (scaled_val > 16'h03FF) begin
                        duty_cycle <= {PWM_WIDTH{1'b1}};
                    end else begin
                        duty_cycle <= scaled_val[PWM_WIDTH-1:0];
                    end
                end
            end else begin
                // Positive - forward direction
                direction <= 1;
                begin : scale_pos
                    reg [15:0] scaled_val;
                    scaled_val = pid_output >>> 6;
                    if (scaled_val > 16'h03FF) begin
                        duty_cycle <= {PWM_WIDTH{1'b1}};
                    end else begin
                        duty_cycle <= scaled_val[PWM_WIDTH-1:0];
                    end
                end
            end
        end
    end
    
    // PWM output generation
    assign pwm_out = (pwm_counter < duty_cycle) ? 1 : 0;
    assign motor_direction = direction;
    assign pwm_duty = duty_cycle;
    assign motor_speed = {direction, pid_output[14:0]};

endmodule
