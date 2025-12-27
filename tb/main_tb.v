module main_tb;
    reg CLK50=0, CLK=0, BTN1=1, BTN2=1;
    reg RMII_CRS_DV=0;
    reg [1:0] RMII_RXD=0;
    reg RMII_RX_ER=0;
    wire RMII_TX_EN;
    wire [1:0] RMII_TXD;
    wire RMII_MDC;
    wire RMII_MDIO;
    wire LED1,LED2,LED3;

    Main dut(
        .CLK(CLK),
        .CLK50(CLK50),
        .BTN1(BTN1),
        .BTN2(BTN2),
        .RMII_CRS_DV(RMII_CRS_DV),
        .RMII_RXD(RMII_RXD),
        .RMII_RX_ER(RMII_RX_ER),
        .RMII_TX_EN(RMII_TX_EN),
        .RMII_TXD(RMII_TXD),
        .RMII_MDC(RMII_MDC),
        .RMII_MDIO(RMII_MDIO),
        .LED1(LED1), .LED2(LED2), .LED3(LED3)
    );

    always #10 CLK50 = ~CLK50; // 50MHz -> not accurate in sim, ok

    initial begin
        // Just run to check it compiles
        #1000;
        $finish;
    end
endmodule
