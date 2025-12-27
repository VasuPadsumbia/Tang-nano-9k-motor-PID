module eth_tx_framer(
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    input  wire [15:0] frame_len,      // bytes in Ethernet frame (destmac..payload)
    output reg  [15:0] rd_idx,          // which byte we are requesting
    input  wire [7:0]  rd_byte,         // byte at rd_idx

    output reg         busy,
    output reg         done,

    output reg         rmii_tx_en,
    output reg  [1:0]  rmii_txd
);

    localparam S_IDLE  = 3'd0;
    localparam S_PREAM = 3'd1;
    localparam S_DATA  = 3'd2;
    localparam S_FCS   = 3'd3;
    localparam S_IPG   = 3'd4;

    reg [2:0] st;
    reg [15:0] cnt;
    reg [2:0] bitp;
    reg [7:0] cur;

    // CRC32 over DATA bytes only
    reg        crc_init, crc_en;
    wire [31:0] crc_out;
    eth_fcs32 u_crc(
        .clk(clk),
        .rst(rst),
        .init(crc_init),
        .en(crc_en),
        .data(cur),
        .crc(crc_out)
    );

    reg [1:0] fcs_idx;
    reg [31:0] fcs_latched;

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            rmii_tx_en <= 1'b0;
            rmii_txd <= 2'b00;

            cnt <= 16'd0;
            bitp <= 3'd0;
            cur <= 8'd0;
            rd_idx <= 16'd0;

            crc_init <= 1'b0;
            crc_en <= 1'b0;
            fcs_idx <= 2'd0;
            fcs_latched <= 32'd0;
        end else begin
            done <= 1'b0;
            crc_init <= 1'b0;
            crc_en <= 1'b0;

            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    rmii_tx_en <= 1'b0;
                    rmii_txd <= 2'b00;
                    cnt <= 16'd0;
                    bitp <= 3'd0;
                    rd_idx <= 16'd0;
                    fcs_idx <= 2'd0;

                    if (start) begin
                        st <= S_PREAM;
                        busy <= 1'b1;
                        rmii_tx_en <= 1'b1;
                    end
                end

                S_PREAM: begin
                    rmii_tx_en <= 1'b1;
                    if (cnt < 16'd7) cur <= 8'h55;
                    else cur <= 8'hD5;

                    rmii_txd <= cur[ (bitp*2) +: 2 ];

                    if (bitp == 3'd3) begin
                        bitp <= 3'd0;
                        cnt <= cnt + 16'd1;
                        if (cnt == 16'd7) begin
                            st <= S_DATA;
                            cnt <= 16'd0;
                            rd_idx <= 16'd0;
                            crc_init <= 1'b1;
                        end
                    end else bitp <= bitp + 3'd1;
                end

                S_DATA: begin
                    rmii_tx_en <= 1'b1;

                    // Load new byte only at start of its 2-bit chunk sequence
                    if (bitp == 3'd0) begin
                        cur <= rd_byte;   // pull byte from client
                        crc_en <= 1'b1;
                    end

                    rmii_txd <= cur[ (bitp*2) +: 2 ];

                    if (bitp == 3'd3) begin
                        bitp <= 3'd0;
                        cnt <= cnt + 16'd1;
                        rd_idx <= rd_idx + 16'd1;

                        if (cnt == frame_len - 16'd1) begin
                            fcs_latched <= ~crc_out;
                            st <= S_FCS;
                            fcs_idx <= 2'd0;
                        end
                    end else bitp <= bitp + 3'd1;
                end

                S_FCS: begin
                    rmii_tx_en <= 1'b1;
                    case (fcs_idx)
                        2'd0: cur <= fcs_latched[7:0];
                        2'd1: cur <= fcs_latched[15:8];
                        2'd2: cur <= fcs_latched[23:16];
                        2'd3: cur <= fcs_latched[31:24];
                    endcase

                    rmii_txd <= cur[ (bitp*2) +: 2 ];

                    if (bitp == 3'd3) begin
                        bitp <= 3'd0;
                        if (fcs_idx == 2'd3) begin
                            st <= S_IPG;
                            cnt <= 16'd0;
                            rmii_tx_en <= 1'b0;
                        end else fcs_idx <= fcs_idx + 2'd1;
                    end else bitp <= bitp + 3'd1;
                end

                S_IPG: begin
                    rmii_tx_en <= 1'b0;
                    rmii_txd <= 2'b00;
                    if (cnt == 16'd200) begin
                        st <= S_IDLE;
                        done <= 1'b1;
                    end else cnt <= cnt + 16'd1;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
