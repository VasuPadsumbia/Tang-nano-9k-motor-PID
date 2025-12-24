// Debug Output Module - Sends telemetry over UART
// Formats and transmits PID controller state information

module debug_output (
    input clk,
    input reset,
    
    // UART TX interface
    output reg tx_start,
    output reg [7:0] tx_char,
    input tx_ready,
    
    // System state
    input [15:0] pid_output,
    input [15:0] error,
    input [15:0] setpoint,
    input [15:0] feedback,
    input [15:0] kp, ki, kd,
    input [7:0] tuning_progress,
    input tuning_done,
    input sim_mode,
    input [15:0] sim_velocity,
    input [15:0] sim_position,
    
    // Control
    input startup_done,
    input send_telemetry
);

    // State machine for sending debug messages
    localparam STATE_IDLE = 0;
    localparam STATE_SEND_BANNER = 1;
    localparam STATE_SEND_DATA = 2;
    localparam STATE_WAIT_STARTUP = 3;
    
    reg [7:0] state = STATE_WAIT_STARTUP;
    reg [7:0] message_index = 0;
    reg [31:0] send_counter = 0;
    reg banner_sent = 0;
    
    // Message buffer - up to 128 characters for banner or telemetry
    reg [7:0] msg_buffer[127:0];
    reg [7:0] msg_length = 0;
    
    // Helper function to convert 4-bit hex to ASCII
    function [7:0] hex_to_ascii;
        input [3:0] hex_val;
        reg [7:0] hex_extended;
        begin
            hex_extended = {4'h0, hex_val};
            if (hex_extended < 8'd10)
                hex_to_ascii = 8'h30 + hex_extended;  // '0'-'9'
            else
                hex_to_ascii = 8'h41 + (hex_extended - 8'd10);  // 'A'-'F'
        end
    endfunction
    
    // Helper to format 16-bit value as 4 hex digits + space
    task format_hex_word;
        input [15:0] value;
        input [6:0] buffer_idx;
        begin
            msg_buffer[buffer_idx] = hex_to_ascii(value[15:12]);
            msg_buffer[buffer_idx + 1] = hex_to_ascii(value[11:8]);
            msg_buffer[buffer_idx + 2] = hex_to_ascii(value[7:4]);
            msg_buffer[buffer_idx + 3] = hex_to_ascii(value[3:0]);
            msg_buffer[buffer_idx + 4] = 8'h20;  // Space
        end
    endtask
    
    always @(posedge clk) begin
        if (reset) begin
            state <= STATE_WAIT_STARTUP;
            message_index <= 0;
            send_counter <= 0;
            banner_sent <= 0;
            tx_start <= 0;
        end else begin
            // Trigger telemetry output periodically
            send_counter <= send_counter + 1;
            
            case (state)
                STATE_WAIT_STARTUP: begin
                    tx_start <= 0;
                    
                    // Wait for startup to complete, then send banner
                    if (startup_done && !banner_sent) begin
                        banner_sent <= 1;
                        state <= STATE_SEND_BANNER;
                        message_index <= 0;
                        
                        // Format startup banner
                        msg_buffer[0] = 8'h0D;  // \r
                        msg_buffer[1] = 8'h0A;  // \n
                        msg_buffer[2] = 8'h2A;  // '*'
                        msg_buffer[3] = 8'h2A;  // '*'
                        msg_buffer[4] = 8'h2A;  // '*'
                        msg_buffer[5] = 8'h20;  // Space
                        msg_buffer[6] = 8'h50;  // 'P'
                        msg_buffer[7] = 8'h49;  // 'I'
                        msg_buffer[8] = 8'h44;  // 'D'
                        msg_buffer[9] = 8'h20;  // Space
                        msg_buffer[10] = 8'h4D;  // 'M'
                        msg_buffer[11] = 8'h6F;  // 'o'
                        msg_buffer[12] = 8'h74;  // 't'
                        msg_buffer[13] = 8'h6F;  // 'o'
                        msg_buffer[14] = 8'h72;  // 'r'
                        msg_buffer[15] = 8'h20;  // Space
                        msg_buffer[16] = 8'h43;  // 'C'
                        msg_buffer[17] = 8'h6F;  // 'o'
                        msg_buffer[18] = 8'h6E;  // 'n'
                        msg_buffer[19] = 8'h74;  // 't'
                        msg_buffer[20] = 8'h72;  // 'r'
                        msg_buffer[21] = 8'h6F;  // 'o'
                        msg_buffer[22] = 8'h6C;  // 'l'
                        msg_buffer[23] = 8'h6C;  // 'l'
                        msg_buffer[24] = 8'h65;  // 'e'
                        msg_buffer[25] = 8'h72;  // 'r'
                        msg_buffer[26] = 8'h20;  // Space
                        msg_buffer[27] = 8'h2A;  // '*'
                        msg_buffer[28] = 8'h2A;  // '*'
                        msg_buffer[29] = 8'h2A;  // '*'
                        msg_buffer[30] = 8'h0D;  // \r
                        msg_buffer[31] = 8'h0A;  // \n
                        msg_buffer[32] = 8'h4D;  // 'M'
                        msg_buffer[33] = 8'h6F;  // 'o'
                        msg_buffer[34] = 8'h64;  // 'd'
                        msg_buffer[35] = 8'h65;  // 'e'
                        msg_buffer[36] = 8'h3A;  // ':'
                        msg_buffer[37] = 8'h20;  // Space
                        
                        if (sim_mode) begin
                            msg_buffer[38] = 8'h53;  // 'S'
                            msg_buffer[39] = 8'h49;  // 'I'
                            msg_buffer[40] = 8'h4D;  // 'M'
                            msg_buffer[41] = 8'h55;  // 'U'
                            msg_buffer[42] = 8'h4C;  // 'L'
                            msg_buffer[43] = 8'h41;  // 'A'
                            msg_buffer[44] = 8'h54;  // 'T'
                            msg_buffer[45] = 8'h49;  // 'I'
                            msg_buffer[46] = 8'h4F;  // 'O'
                            msg_buffer[47] = 8'h4E;  // 'N'
                            msg_length = 48;
                        end else begin
                            msg_buffer[38] = 8'h48;  // 'H'
                            msg_buffer[39] = 8'h41;  // 'A'
                            msg_buffer[40] = 8'h52;  // 'R'
                            msg_buffer[41] = 8'h44;  // 'D'
                            msg_buffer[42] = 8'h57;  // 'W'
                            msg_buffer[43] = 8'h41;  // 'A'
                            msg_buffer[44] = 8'h52;  // 'R'
                            msg_buffer[45] = 8'h45;  // 'E'
                            msg_length = 46;
                        end
                        msg_buffer[msg_length[6:0]] = 8'h0D;  // \r
                        msg_buffer[msg_length[6:0] + 1] = 8'h0A;  // \n
                        msg_length = msg_length + 2;
                    end else if (startup_done && banner_sent) begin
                        state <= STATE_IDLE;
                    end
                end
                
                STATE_SEND_BANNER: begin
                    if (tx_ready && message_index < msg_length) begin
                        tx_char <= msg_buffer[message_index[6:0]];
                        tx_start <= 1;
                        message_index <= message_index + 1;
                    end else if (message_index >= msg_length) begin
                        tx_start <= 0;
                        state <= STATE_IDLE;
                    end
                end
                
                STATE_IDLE: begin
                    tx_start <= 0;
                    
                    // Send telemetry every ~120K cycles (~10Hz)
                    if (send_counter >= 32'd120_000 || send_telemetry) begin
                        send_counter <= 0;
                        state <= STATE_SEND_DATA;
                        message_index <= 0;
                        
                        // Format message: "SET:xxxx FB:xxxx PID:xxxx ERR:xxxx KP:xxxx KI:xxxx KD:xxxx T:xxx\r\n"
                        // [0-3]: SET value
                        format_hex_word(setpoint, 0);
                        // [5-8]: FB value
                        format_hex_word(feedback, 5);
                        // [10-13]: PID output
                        format_hex_word(pid_output, 10);
                        // [15-18]: Error
                        format_hex_word(error, 15);
                        // [20-23]: Kp
                        format_hex_word(kp, 20);
                        // [25-28]: Ki
                        format_hex_word(ki, 25);
                        // [30-33]: Kd
                        format_hex_word(kd, 30);
                        
                        // Tuning status at [35]: T:xxx
                        msg_buffer[35] = 8'h54;  // 'T'
                        msg_buffer[36] = 8'h3A;  // ':'
                        msg_buffer[37] = hex_to_ascii(tuning_progress[7:4]);
                        msg_buffer[38] = hex_to_ascii(tuning_progress[3:0]);
                        msg_buffer[39] = 8'h20;  // Space
                        
                        // Mode indicator
                        if (sim_mode) begin
                            msg_buffer[40] = 8'h53;  // 'S' for Simulation
                        end else begin
                            msg_buffer[40] = 8'h48;  // 'H' for Hardware
                        end
                        msg_buffer[41] = 8'h0D;  // \r
                        msg_buffer[42] = 8'h0A;  // \n
                        msg_length = 43;
                    end
                end
                
                STATE_SEND_DATA: begin
                    if (tx_ready && message_index < msg_length) begin
                        tx_char <= msg_buffer[message_index[6:0]];
                        tx_start <= 1;
                        message_index <= message_index + 1;
                    end else if (message_index >= msg_length) begin
                        tx_start <= 0;
                        state <= STATE_IDLE;
                    end
                end
                
                default: begin
                    tx_start <= 0;
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
