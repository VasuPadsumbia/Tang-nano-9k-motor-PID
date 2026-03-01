// =============================================================================
// Module  : udp_telem_tx
// Project : Tang Nano 9K – PID Motor Controller over Ethernet
// File    : rtl/udp_telem_tx.v
//
// Purpose : Periodically transmits a 20-byte UDP telemetry datagram to the PC
//           dashboard.  The interval is configurable via TELEM_INTERVAL
//           (in sample ticks, default 50 → 50 ms at 1 kHz sample rate).
//
//   Telemetry packet format (20 bytes, all big-endian):
//     Bytes 0-1  : setpoint  (signed 16-bit)
//     Bytes 2-3  : feedback  (signed 16-bit)
//     Bytes 4-5  : pid_out   (signed 16-bit)
//     Bytes 6-7  : error     (signed 16-bit)
//     Bytes 8-9  : kp        (unsigned 16-bit)
//     Bytes 10-11: ki
//     Bytes 12-13: kd
//     Byte 14    : status flags { 6'b0, enc_dir, pid_enable }
//     Bytes 15-19: reserved (0x00)
//
// Interface
//   clk/rst        – Clock and reset
//   sample_tick    – 1-cycle pulse at PID sample rate (1 kHz)
//   setpoint ..    – PID state signals
//   tx_req         – Pulse: request telemetry TX
//   tx_busy        – HIGH when TX pipeline busy
//   buf_idx        – Frame builder drives this to read frame bytes
//   buf_byte       – Muxed frame byte output
//   buf_len        – Total payload byte count (20)
//   buf_last       – HIGH on last byte
//   dst_mac        – Destination MAC (PC's MAC, learned from last UDP command)
//   dst_ip         – Destination IP
//   dst_port       – Destination UDP port
// =============================================================================

`default_nettype none

module udp_telem_tx #(
    parameter integer TELEM_INTERVAL = 50   // Sample ticks between sends (50ms)
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        sample_tick,

    // PID state snapshot
    input  wire signed [15:0] setpoint,
    input  wire signed [15:0] feedback,
    input  wire signed [15:0] pid_out,
    input  wire signed [15:0] error_in,
    input  wire        [15:0] kp,
    input  wire        [15:0] ki,
    input  wire        [15:0] kd,
    input  wire               pid_enable,
    input  wire               enc_dir,

    // Destination (PC), updated from first UDP command received
    input  wire [47:0] dst_mac,
    input  wire [31:0] dst_ip,
    input  wire [15:0] dst_port,

    // TX request interface
    output reg         tx_req,
    input  wire        tx_busy,

    // Buffer interface (driven by eth_tx_builder)
    input  wire [4:0]  buf_idx,
    output reg  [7:0]  buf_byte,
    output wire [4:0]  buf_len,
    output wire        buf_last
);

    localparam PAYLOAD_LEN = 5'd20;
    assign buf_len  = PAYLOAD_LEN;
    assign buf_last = (buf_idx == PAYLOAD_LEN - 1);

    // -------------------------------------------------------------------------
    // Interval timer
    // -------------------------------------------------------------------------
    reg [$clog2(TELEM_INTERVAL+1)-1:0] interval_cnt;

    // Snapshot registers (latched at send-time to avoid mid-packet tearing)
    reg signed [15:0] snap_setpoint;
    reg signed [15:0] snap_feedback;
    reg signed [15:0] snap_pid_out;
    reg signed [15:0] snap_error;
    reg        [15:0] snap_kp, snap_ki, snap_kd;
    reg        [7:0]  snap_status;

    always @(posedge clk) begin
        tx_req <= 1'b0;
        if (rst) begin
            interval_cnt <= 0;
        end else if (sample_tick) begin
            if (interval_cnt == TELEM_INTERVAL - 1) begin
                interval_cnt <= 0;
                if (!tx_busy) begin
                    // Latch snapshot
                    snap_setpoint <= setpoint;
                    snap_feedback <= feedback;
                    snap_pid_out  <= pid_out;
                    snap_error    <= error_in;
                    snap_kp       <= kp;
                    snap_ki       <= ki;
                    snap_kd       <= kd;
                    snap_status   <= {6'b0, enc_dir, pid_enable};
                    tx_req        <= 1'b1;
                end
            end else begin
                interval_cnt <= interval_cnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Combinatorial payload byte mux
    // -------------------------------------------------------------------------
    always @(*) begin
        case (buf_idx)
            5'd0:  buf_byte = snap_setpoint[15:8];
            5'd1:  buf_byte = snap_setpoint[7:0];
            5'd2:  buf_byte = snap_feedback[15:8];
            5'd3:  buf_byte = snap_feedback[7:0];
            5'd4:  buf_byte = snap_pid_out[15:8];
            5'd5:  buf_byte = snap_pid_out[7:0];
            5'd6:  buf_byte = snap_error[15:8];
            5'd7:  buf_byte = snap_error[7:0];
            5'd8:  buf_byte = snap_kp[15:8];
            5'd9:  buf_byte = snap_kp[7:0];
            5'd10: buf_byte = snap_ki[15:8];
            5'd11: buf_byte = snap_ki[7:0];
            5'd12: buf_byte = snap_kd[15:8];
            5'd13: buf_byte = snap_kd[7:0];
            5'd14: buf_byte = snap_status;
            default: buf_byte = 8'h00;  // reserved padding
        endcase
    end

endmodule

`default_nettype wire
