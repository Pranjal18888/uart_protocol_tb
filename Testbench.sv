//=====================================================================
// TOP - clock generation, DUT + interface instantiation, test invocation
// This is the simulation entry point (top module).
//=====================================================================
`include "tb_classes.sv"   // brings in txn/generator/driver/monitor/scoreboard/env/test

module testbench;

    // Free-running system clock: 100MHz (10ns period, toggling every 5ns)
    logic clk = 0;
    always #5 clk = ~clk;

    // Interface instance - single handle carrying all DUT<->TB signals
    uart_if intf (clk);

    // DUT instantiation - loopback: tx_serial -> rx_serial
    uart_dut #(
        .CLK_FREQ  (100_000_000),
        .BAUD_RATE (1_000_000)     // fast baud so sim finishes quickly
    ) dut (
        .clk       (clk),
        .rst_n     (intf.rst_n),
        .tx_start  (intf.tx_start),
        .tx_data   (intf.tx_data),
        .tx_busy   (intf.tx_busy),
        .tx_serial (intf.tx_serial),
        .rx_serial (intf.rx_serial),
        .rx_data   (intf.rx_data),
        .rx_valid  (intf.rx_valid)
    );

    // Loopback connection: TX output directly feeds RX input.
    // This lets us verify TX+RX together without a second physical device.
    assign intf.rx_serial = intf.tx_serial;

    uart_test test;

    // Build and run the test - this single call kicks off the entire
    // generator -> driver -> DUT -> monitor -> scoreboard pipeline
    initial begin
        test = new(intf, 10); // run 10 random transactions
        test.run();
    end

    // Dump waveforms so they can be viewed in EPWave on EDA Playground
    initial begin
        $dumpfile("uart_dump.vcd");
        $dumpvars(0, testbench);
    end

endmodule
