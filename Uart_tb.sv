//=====================================================================
// UART Transaction (Sequence Item)
// This is the basic "packet" of data that flows between generator ->
// driver -> DUT -> monitor -> scoreboard.
//=====================================================================
class uart_txn;
    rand bit [7:0] data;   // the byte to be sent/received - randomized by generator

    // Deep copy helper - needed because class handles are references;
    // without copy(), driver/monitor/scoreboard could all end up
    // pointing at the SAME object and overwrite each other's data.
    function uart_txn copy();
        copy      = new();
        copy.data = this.data;
        return copy;
    endfunction

    // Debug print helper - tag tells you which component printed it
    // (GEN/DRV/MON) so you can trace a byte through the whole pipeline.
    function void display(string tag = "");
        $display("[%0t] %s TXN : data = 0x%0h", $time, tag, data);
    endfunction
endclass


//=====================================================================
// Generator - creates random transactions and pushes to driver mailbox
// This is the "stimulus" component: it has no knowledge of the DUT,
// it just produces random test data.
//=====================================================================
class uart_generator;
    mailbox #(uart_txn) gen2drv_mbx;  // channel to send txns to the driver
    int num_txns;                     // how many random bytes to generate
    event  done;                      // fired when all txns have been generated

    function new(mailbox #(uart_txn) gen2drv_mbx, int num_txns = 10);
        this.gen2drv_mbx = gen2drv_mbx;
        this.num_txns    = num_txns;
    endfunction

    task run();
        uart_txn txn;
        for (int i = 0; i < num_txns; i++) begin
            txn = new();
            if (!txn.randomize())          // randomize the byte
                $fatal("Randomization failed");
            txn.display("GEN");
            gen2drv_mbx.put(txn);          // hand off to driver (blocks if mailbox full)
        end
        -> done;   // signal environment that generation is complete
    endtask
endclass


//=====================================================================
// Driver - drives transactions onto DUT TX interface
// Converts an abstract uart_txn into actual pin wiggles on the DUT.
//=====================================================================
class uart_driver;
    virtual uart_if vif;               // handle to the physical interface signals
    mailbox #(uart_txn) gen2drv_mbx;   // channel to receive txns from generator

    function new(virtual uart_if vif, mailbox #(uart_txn) gen2drv_mbx);
        this.vif          = vif;
        this.gen2drv_mbx  = gen2drv_mbx;
    endfunction

    // Apply and release reset before the test starts driving anything
    task reset();
        vif.rst_n    <= 0;
        vif.tx_start <= 0;
        vif.tx_data  <= 0;
        repeat (5) @(posedge vif.clk);   // hold reset for a few clocks
        vif.rst_n <= 1;
        @(posedge vif.clk);
        $display("[%0t] DRV : Reset complete", $time);
    endtask

    // Main driving loop: pop a txn, load it into DUT, pulse tx_start,
    // then wait for the DUT to finish shifting it out before taking the next one
    task run();
        uart_txn txn;
        forever begin
            gen2drv_mbx.get(txn);         // blocks until generator produces a txn
            @(posedge vif.clk);
            vif.tx_data  <= txn.data;     // present data on the bus
            vif.tx_start <= 1;            // pulse start for exactly 1 cycle
            @(posedge vif.clk);
            vif.tx_start <= 0;
            wait (vif.tx_busy == 0);      // wait until byte fully shifted out
            txn.display("DRV");
        end
    endtask
endclass


//=====================================================================
// Monitor - watches DUT RX output, packages into txn, sends to scoreboard
// Passive component: only observes the bus, never drives it.
//=====================================================================
class uart_monitor;
    virtual uart_if vif;
    mailbox #(uart_txn) mon2scb_mbx;   // channel to send observed txns to scoreboard

    function new(virtual uart_if vif, mailbox #(uart_txn) mon2scb_mbx);
        this.vif          = vif;
        this.mon2scb_mbx  = mon2scb_mbx;
    endfunction

    // Every clock edge, check if the receiver has produced a valid byte;
    // if so, capture it and forward to the scoreboard for checking
    task run();
        uart_txn txn;
        forever begin
            @(posedge vif.clk);
            if (vif.rx_valid) begin
                txn      = new();
                txn.data = vif.rx_data;
                txn.display("MON");
                mon2scb_mbx.put(txn);
            end
        end
    endtask
endclass


//=====================================================================
// Scoreboard - compares expected (sent) vs actual (received) data
// This is where PASS/FAIL verdicts are decided.
//=====================================================================
class uart_scoreboard;
    mailbox #(uart_txn) exp_mbx;      // expected txns (from driver, i.e. what we sent)
    mailbox #(uart_txn) mon2scb_mbx;  // actual txns (from monitor, i.e. what we received)

    int pass_cnt = 0;
    int fail_cnt = 0;

    function new(mailbox #(uart_txn) exp_mbx, mailbox #(uart_txn) mon2scb_mbx);
        this.exp_mbx      = exp_mbx;
        this.mon2scb_mbx  = mon2scb_mbx;
    endfunction

    // For every txn sent, wait for the matching received txn and compare.
    // Since UART is a simple in-order pipe, FIFO ordering guarantees
    // exp_mbx and mon2scb_mbx entries line up 1-to-1.
    task run();
        uart_txn exp_txn, act_txn;
        forever begin
            exp_mbx.get(exp_txn);
            mon2scb_mbx.get(act_txn);
            if (exp_txn.data == act_txn.data) begin
                pass_cnt++;
                $display("[%0t] SCB : PASS - expected=0x%0h actual=0x%0h",
                          $time, exp_txn.data, act_txn.data);
            end else begin
                fail_cnt++;
                $display("[%0t] SCB : FAIL - expected=0x%0h actual=0x%0h",
                          $time, exp_txn.data, act_txn.data);
            end
        end
    endtask

    // Final summary printed at the end of simulation
    function void report();
        $display("=====================================================");
        $display(" SCOREBOARD REPORT : PASS = %0d  FAIL = %0d", pass_cnt, fail_cnt);
        $display("=====================================================");
    endfunction
endclass


//=====================================================================
// Environment - instantiates & connects all TB components
// This is the "glue" layer: creates mailboxes, wires generator ->
// driver -> monitor -> scoreboard together, and runs them concurrently.
//=====================================================================
class uart_env;
    virtual uart_if vif;

    mailbox #(uart_txn) gen2drv_mbx;   // generator  -> driver
    mailbox #(uart_txn) mon2scb_mbx;   // monitor    -> scoreboard (actual)
    mailbox #(uart_txn) exp_mbx;       // driver     -> scoreboard (expected)

    uart_generator  gen;
    uart_driver     drv;
    uart_monitor    mon;
    uart_scoreboard scb;

    function new(virtual uart_if vif, int num_txns = 10);
        this.vif = vif;

        gen2drv_mbx = new();
        mon2scb_mbx = new();
        exp_mbx     = new();

        gen = new(gen2drv_mbx, num_txns);
        drv = new(vif, gen2drv_mbx, exp_mbx);
        mon = new(vif, mon2scb_mbx);
        scb = new(exp_mbx, mon2scb_mbx);
    endfunction

    task run();
        drv.reset();
        // Run all components concurrently. join_any exits this fork block
        // as soon as ANY one of them finishes (used here just to keep the
        // structure simple); the real end-of-test wait is below.
        fork
            gen.run();
            drv.run();
            mon.run();
            scb.run();
        join_any
        // wait for generator to finish + some drain time for last byte
        wait (gen.done.triggered);
        repeat (2000) @(posedge vif.clk); // drain time for last byte(s)
        scb.report();
        $finish;
    endtask
endclass


//=====================================================================
// Test - top-level test that builds env and runs it
// Keeping "test" separate from "env" lets you later create multiple
// test classes (different num_txns, different scenarios) that all
// reuse the same environment.
//=====================================================================
class uart_test;
    uart_env env;

    function new(virtual uart_if vif, int num_txns = 10);
        env = new(vif, num_txns);
    endfunction

    task run();
        env.run();
    endtask
endclass
