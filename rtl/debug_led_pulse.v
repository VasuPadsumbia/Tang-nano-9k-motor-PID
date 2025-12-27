module debug_led_pulse #(parameter integer HOLD_CYCLES = 2_500_000)(
    input  wire clk,
    input  wire rst,
    input  wire trig,
    output reg  led
);
    reg [$clog2(HOLD_CYCLES+1)-1:0] cnt;

    always @(posedge clk) begin
        if (rst) begin
            led <= 1'b0;
            cnt <= 'd0;
        end else begin
            if (trig) begin
                led <= 1'b1;
                cnt <= HOLD_CYCLES[$clog2(HOLD_CYCLES+1)-1:0];
            end else if (cnt != 0) begin
                cnt <= cnt - 1'b1;
                if (cnt == 1) led <= 1'b0;
            end
        end
    end
endmodule
