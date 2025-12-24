// PID Controller Module - Core PID calculation
// Modular design with fixed-point arithmetic

module pid_controller #(
    parameter integer KP_WIDTH = 16,    // Proportional gain width
    parameter integer KI_WIDTH = 16,    // Integral gain width
    parameter integer KD_WIDTH = 16,    // Derivative gain width
    parameter integer ERROR_WIDTH = 16, // Error signal width
    parameter integer OUTPUT_WIDTH = 16 // Output width
) (
    input clk,
    input reset,
    
    // PID Coefficients (fixed-point, scaled by 256)
    input [KP_WIDTH-1:0] kp,
    input [KI_WIDTH-1:0] ki,
    input [KD_WIDTH-1:0] kd,
    
    // Input signals
    input [ERROR_WIDTH-1:0] error_in,  // Setpoint - Feedback
    input enable,
    
    // Output
    output reg [OUTPUT_WIDTH-1:0] pid_output,
    output reg [31:0] integral_sum,    // Debug: integral accumulator
    output reg [ERROR_WIDTH-1:0] prev_error
);

    reg [31:0] p_term;
    reg [31:0] i_term;
    reg [31:0] d_term;
    reg [31:0] raw_output;
    
    always @(posedge clk) begin
        if (reset) begin
            integral_sum <= 0;
            prev_error <= 0;
            pid_output <= 0;
            p_term <= 0;
            i_term <= 0;
            d_term <= 0;
        end else if (enable) begin
            // Proportional term: Kp * error
            p_term <= (kp * error_in) >>> 8;  // Fixed-point division by 256
            
            // Integral term: Ki * sum(error)
            integral_sum <= integral_sum + (ki * error_in);
            i_term <= integral_sum >>> 8;
            
            // Derivative term: Kd * (error - prev_error)
            d_term <= (kd * {{ERROR_WIDTH{error_in[ERROR_WIDTH-1] ^ prev_error[ERROR_WIDTH-1]}}, (error_in - prev_error)}) >>> 8;
            
            // Total output
            raw_output <= p_term + i_term + d_term;
            
            // Saturate output to OUTPUT_WIDTH
            if (raw_output[31]) begin  // Negative
                pid_output <= (raw_output[OUTPUT_WIDTH-1:0] < ~(1 << (OUTPUT_WIDTH-1))) ?
                              ~(1 << (OUTPUT_WIDTH-1)) : raw_output[OUTPUT_WIDTH-1:0];
            end else begin  // Positive
                pid_output <= (raw_output[OUTPUT_WIDTH-1:0] > ((1 << (OUTPUT_WIDTH-1)) - 1)) ?
                              ((1 << (OUTPUT_WIDTH-1)) - 1) : raw_output[OUTPUT_WIDTH-1:0];
            end
            
            // Store previous error for derivative
            prev_error <= error_in;
        end
    end

endmodule
