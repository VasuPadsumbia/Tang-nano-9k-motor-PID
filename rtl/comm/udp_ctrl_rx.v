// =============================================================================
// Module  : udp_ctrl_rx
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/udp_ctrl_rx.v
//
// Purpose : Decodes 6-byte UDP command datagrams from the PC dashboard and
//           updates PID control parameters or setpoint registers.
//
//   Command packet format (6 bytes, network byte order):
//     Byte 0   : Command ID
//     Bytes 1-2: 16-bit value (MSB first)
//     Bytes 3-5: Reserved / ignore
//
//   Command IDs:
//     0x01 – Set setpoint  (signed 16-bit)
//     0x02 – Set Kp gain   (unsigned 16-bit, 8.8 fixed-point)
//     0x03 – Set Ki gain
//     0x04 – Set Kd gain
//     0x05 – Enable PID    (value[0]: 1=enable, 0=disable)
//     0x06 – Reset PID     (any value clears integrator & error)
//
// Interface
//   clk/rst      – Clock and reset
//   pay_vld      – Payload byte valid (from eth_rx_parser)
//   pay_byte     – Payload byte
//   pay_idx      – Byte index within payload (0-based)
//   pay_last     – Last payload byte flag
//   setpoint     – Decoded setpoint (signed 16-bit)
//   kp/ki/kd     – Decoded PID gains (unsigned 16-bit, 8.8 format)
//   pid_enable   – PID enable flag
//   pid_reset    – 1-cycle pulse: reset PID state
// =============================================================================

`default_nettype none

module udp_ctrl_rx (
    input  wire        clk,
    input  wire        rst,

    // Payload from eth_rx_parser
    input  wire        pay_vld,
    input  wire [7:0]  pay_byte,
    input  wire [10:0] pay_idx,
    input  wire        pay_last,

    // PID control outputs (registered, hold until updated)
    output reg  signed [15:0] setpoint,
    output reg         [15:0] kp,
    output reg         [15:0] ki,
    output reg         [15:0] kd,
    output reg                 pid_enable,
    output reg                 pid_reset
);

    // Command byte register
    reg [7:0] cmd;
    reg [7:0] val_hi;    // MSB of 16-bit value

    // Default gain values (8.8 fixed-point)
    localparam [15:0] KP_DEFAULT = 16'h0100;  // 1.0
    localparam [15:0] KI_DEFAULT = 16'h0020;  // 0.125
    localparam [15:0] KD_DEFAULT = 16'h0080;  // 0.5

    always @(posedge clk) begin
        pid_reset <= 1'b0;   // default de-assert

        if (rst) begin
            setpoint   <= 16'sd0;
            kp         <= KP_DEFAULT;
            ki         <= KI_DEFAULT;
            kd         <= KD_DEFAULT;
            pid_enable <= 1'b0;
            cmd        <= 8'h00;
            val_hi     <= 8'h00;
        end else if (pay_vld) begin
            case (pay_idx)
                11'd0: cmd     <= pay_byte;
                11'd1: val_hi  <= pay_byte;
                11'd2: begin
                    // Latch full 16-bit value and execute command
                    case (cmd)
                        8'h01: setpoint   <= {val_hi, pay_byte};
                        8'h02: kp         <= {val_hi, pay_byte};
                        8'h03: ki         <= {val_hi, pay_byte};
                        8'h04: kd         <= {val_hi, pay_byte};
                        8'h05: pid_enable <= pay_byte[0];
                        8'h06: pid_reset  <= 1'b1;
                        default: ;
                    endcase
                end
                default: ;  // bytes 3-5 reserved, ignore
            endcase
        end
    end

endmodule

`default_nettype wire
