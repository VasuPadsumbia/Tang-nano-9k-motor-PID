module rmii_rx(
  input  wire       clk50,
  input  wire       rst_n,
  input  wire [1:0] rxd,
  input  wire       crs_dv,

  output reg  [7:0] rx_byte,
  output reg        rx_byte_valid,
  output reg        frame_active
);

  reg [1:0] sym_cnt;
  reg [7:0] sh;
  reg       locked;      // locked to byte boundary after preamble
  reg [2:0] pre_cnt;     // count 0x55 bytes

  // Assemble bytes LSB-first (RMII sends 2-bit symbols, LSB-first)
  wire [7:0] assembled = { rxd, sh[7:2] };

  always @(posedge clk50) begin
    if (!rst_n) begin
      sym_cnt       <= 2'd0;
      sh            <= 8'd0;
      rx_byte       <= 8'd0;
      rx_byte_valid <= 1'b0;
      frame_active  <= 1'b0;
      locked        <= 1'b0;
      pre_cnt       <= 3'd0;
    end else begin
      rx_byte_valid <= 1'b0;

      if (!crs_dv) begin
        // end of frame / idle
        sym_cnt      <= 2'd0;
        frame_active <= 1'b0;
        locked       <= 1'b0;
        pre_cnt      <= 3'd0;
      end else begin
        frame_active <= 1'b1;

        // shift in symbols
        sh <= { rxd, sh[7:2] };

        if (sym_cnt == 2'd3) begin
          sym_cnt <= 2'd0;

          // We have a complete byte candidate
          if (!locked) begin
            // Hunt for 7x 0x55 then 0xD5
            if (assembled == 8'h55) begin
              if (pre_cnt != 3'd7) pre_cnt <= pre_cnt + 3'd1;
            end else if (assembled == 8'hD5 && pre_cnt >= 3'd5) begin
              // Seen enough preamble then SFD: lock
              locked   <= 1'b1;
              pre_cnt  <= 3'd0;
              // Do not output SFD byte to upper layer
            end else begin
              pre_cnt <= 3'd0;
            end
          end else begin
            // Locked: output bytes to parser
            rx_byte       <= assembled;
            rx_byte_valid <= 1'b1;
          end

        end else begin
          sym_cnt <= sym_cnt + 2'd1;
        end
      end
    end
  end
endmodule
