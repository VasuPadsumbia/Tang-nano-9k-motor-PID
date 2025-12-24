module rmii_tx(
  input  wire       clk50,
  input  wire       rst_n,

  input  wire       start,
  input  wire [7:0] data,
  input  wire       data_valid,
  input  wire       last,
  output reg        ready,
  output reg        busy,

  output reg  [1:0] txd,
  output reg        tx_en
);

  localparam ST_IDLE     = 3'd0;
  localparam ST_PREAMBLE = 3'd1;
  localparam ST_SFD      = 3'd2;
  localparam ST_DATA     = 3'd3;
  localparam ST_IFG      = 3'd4;

  reg [2:0] st;

  reg [2:0] pre_bytes;      // 0..6  (7 bytes of 0x55)
  reg [1:0] sym_cnt;        // 0..3  (4 symbols per byte)

  reg [7:0] cur_byte;
  reg       have_byte;

  // drive 2-bit symbols at 50MHz; 4 cycles per byte
  function [1:0] sym;
    input [7:0] b;
    input [1:0] i;
    begin
      case (i)
        2'd0: sym = b[1:0];
        2'd1: sym = b[3:2];
        2'd2: sym = b[5:4];
        default: sym = b[7:6];
      endcase
    end
  endfunction

  always @(posedge clk50) begin
    if (!rst_n) begin
      st        <= ST_IDLE;
      tx_en     <= 1'b0;
      txd       <= 2'b00;
      busy      <= 1'b0;
      ready     <= 1'b1;
      pre_bytes <= 3'd0;
      sym_cnt   <= 2'd0;
      cur_byte  <= 8'd0;
      have_byte <= 1'b0;
    end else begin
      // Defaults each cycle (prevents "sticky" outputs)
      tx_en <= 1'b0;
      txd   <= 2'b00;
      busy  <= (st != ST_IDLE);
      ready <= (st == ST_IDLE);

      case (st)
        ST_IDLE: begin
          pre_bytes <= 3'd0;
          sym_cnt   <= 2'd0;
          have_byte <= 1'b0;
          if (start) begin
            st <= ST_PREAMBLE;
          end
        end

        ST_PREAMBLE: begin
          tx_en <= 1'b1;
          txd   <= sym(8'h55, sym_cnt);

          if (sym_cnt == 2'd3) begin
            sym_cnt <= 2'd0;
            if (pre_bytes == 3'd6) begin
              st <= ST_SFD;
            end else begin
              pre_bytes <= pre_bytes + 3'd1;
            end
          end else begin
            sym_cnt <= sym_cnt + 2'd1;
          end
        end

        ST_SFD: begin
          tx_en <= 1'b1;
          txd   <= sym(8'hD5, sym_cnt);

          if (sym_cnt == 2'd3) begin
            sym_cnt   <= 2'd0;
            have_byte <= 1'b0;
            st        <= ST_DATA;
          end else begin
            sym_cnt <= sym_cnt + 2'd1;
          end
        end

        ST_DATA: begin
          tx_en <= 1'b1;

          // Load a new byte only at byte boundary
          if (!have_byte) begin
            if (data_valid) begin
              cur_byte  <= data;
              have_byte <= 1'b1;
              sym_cnt   <= 2'd0;
            end else begin
              // no data yet -> keep txd at 00 but keep tx_en asserted in this state
              txd <= 2'b00;
            end
          end else begin
            txd <= sym(cur_byte, sym_cnt);

            if (sym_cnt == 2'd3) begin
              sym_cnt   <= 2'd0;
              have_byte <= 1'b0;
              if (last) st <= ST_IFG;
            end else begin
              sym_cnt <= sym_cnt + 2'd1;
            end
          end
        end

        ST_IFG: begin
          // minimal IFG (you can extend with a counter later)
          tx_en <= 1'b0;
          txd   <= 2'b00;
          st    <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end
endmodule
