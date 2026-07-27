//=====================================================================
// UART Interface
// Bundles all signals between testbench <-> DUT so we only need to
// pass ONE handle (vif) around to driver/monitor instead of many wires.
//=====================================================================
interface uart_if (input logic clk);
    logic       rst_n;       // active-low reset, driven by driver

    // ---- TX side signals (testbench drives these into DUT) ----
    logic       tx_start;    // pulse HIGH for 1 clk to kick off a new byte transmit
    logic [7:0] tx_data;     // byte to be transmitted (must be stable when tx_start=1)
    logic       tx_busy;     // DUT drives HIGH while a byte is being shifted out
    logic       tx_serial;   // actual UART TX line (start bit + 8 data bits + stop bit)

    // ---- RX side signals (DUT drives these out, testbench observes) ----
    logic       rx_serial;   // UART RX line, driven from tx_serial via loopback in TOP
    logic [7:0] rx_data;     // byte decoded by the receiver, valid when rx_valid=1
    logic       rx_valid;    // 1-cycle pulse indicating rx_data is valid/ready

    // loopback: DUT tx_serial feeds DUT rx_serial (done in TOP)
endinterface

//=====================================================================
// UART DUT - Simple 8N1 UART (Transmitter + Receiver)
// 8N1 = 8 data bits, No parity, 1 stop bit. LSB shifted out first.
//=====================================================================
module uart_dut #(
    parameter CLK_FREQ  = 50_000_000,   // system clock frequency in Hz
    parameter BAUD_RATE = 115200        // desired UART baud rate
) (
    input  logic       clk,
    input  logic       rst_n,

    // TX side
    input  logic       tx_start,
    input  logic [7:0] tx_data,
    output logic       tx_busy,
    output logic       tx_serial,

    // RX side
    input  logic       rx_serial,
    output logic [7:0] rx_data,
    output logic       rx_valid
);

    // Number of system clock cycles per UART bit period.
    // e.g. 100MHz / 1Mbaud = 100 clk cycles per bit.
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;

    //-----------------------------------------------------------------
    // TRANSMITTER
    // FSM: IDLE -> START (drive start bit) -> DATA (shift 8 bits, LSB
    //      first) -> STOP (drive stop bit) -> back to IDLE
    //-----------------------------------------------------------------
    typedef enum logic [1:0] {TX_IDLE, TX_START, TX_DATA, TX_STOP} tx_state_t;
    tx_state_t tx_state;

    integer tx_baud_cnt;   // counts clk cycles within the current bit period
    integer tx_bit_cnt;    // counts which data bit (0-7) is currently being sent
    logic [7:0] tx_shift;  // shift register holding the byte being transmitted

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            tx_serial   <= 1'b1;   // UART line idles HIGH
            tx_busy     <= 1'b0;
            tx_baud_cnt <= 0;
            tx_bit_cnt  <= 0;
        end else begin
            case (tx_state)
                // Wait here until testbench/driver asserts tx_start
                TX_IDLE: begin
                    tx_serial <= 1'b1;   // line idle = HIGH
                    tx_busy   <= 1'b0;
                    if (tx_start) begin
                        tx_shift    <= tx_data;  // latch byte to send
                        tx_busy     <= 1'b1;     // tell outside world we're busy
                        tx_state    <= TX_START;
                        tx_baud_cnt <= 0;
                    end
                end
                // Drive the START bit (always 0) for one full bit period
                TX_START: begin
                    tx_serial <= 1'b0; // start bit
                    if (tx_baud_cnt == BAUD_DIV-1) begin
                        tx_baud_cnt <= 0;
                        tx_bit_cnt  <= 0;
                        tx_state    <= TX_DATA;
                    end else tx_baud_cnt <= tx_baud_cnt + 1;
                end
                // Shift out 8 data bits, LSB first, one bit per BAUD_DIV cycles
                TX_DATA: begin
                    tx_serial <= tx_shift[0]; // drive current LSB onto the line
                    if (tx_baud_cnt == BAUD_DIV-1) begin
                        tx_baud_cnt <= 0;
                        tx_shift    <= tx_shift >> 1; // move to next bit
                        if (tx_bit_cnt == 7) tx_state <= TX_STOP; // all 8 bits sent
                        else tx_bit_cnt <= tx_bit_cnt + 1;
                    end else tx_baud_cnt <= tx_baud_cnt + 1;
                end
                // Drive the STOP bit (always 1) for one full bit period
                TX_STOP: begin
                    tx_serial <= 1'b1; // stop bit
                    if (tx_baud_cnt == BAUD_DIV-1) begin
                        tx_baud_cnt <= 0;
                        tx_busy     <= 1'b0;   // byte fully sent, ready for next
                        tx_state    <= TX_IDLE;
                    end else tx_baud_cnt <= tx_baud_cnt + 1;
                end
            endcase
        end
    end

    //-----------------------------------------------------------------
    // RECEIVER
    // FSM: IDLE (detect start bit) -> START (wait half a bit period to
    //      land in the middle of the bit for reliable sampling) ->
    //      DATA (sample 8 bits, LSB first) -> STOP -> back to IDLE
    //-----------------------------------------------------------------
    typedef enum logic [1:0] {RX_IDLE, RX_START, RX_DATA, RX_STOP} rx_state_t;
    rx_state_t rx_state;

    integer rx_baud_cnt;   // counts clk cycles within the current bit period
    integer rx_bit_cnt;    // counts which data bit (0-7) is currently being sampled
    logic [7:0] rx_shift;  // shift register that accumulates the received byte

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state    <= RX_IDLE;
            rx_valid    <= 1'b0;
            rx_baud_cnt <= 0;
            rx_bit_cnt  <= 0;
        end else begin
            rx_valid <= 1'b0; // default: de-assert every cycle, pulse only in RX_STOP
            case (rx_state)
                // Line idles HIGH; a HIGH->LOW transition marks a start bit
                RX_IDLE: begin
                    if (!rx_serial) begin // detect start bit
                        rx_state    <= RX_START;
                        rx_baud_cnt <= 0;
                    end
                end
                // Wait until the middle of the start bit before sampling data,
                // this avoids sampling too close to a noisy edge
                RX_START: begin
                    if (rx_baud_cnt == BAUD_DIV/2) begin // sample mid start-bit
                        rx_baud_cnt <= 0;
                        rx_bit_cnt  <= 0;
                        rx_state    <= RX_DATA;
                    end else rx_baud_cnt <= rx_baud_cnt + 1;
                end
                // Sample rx_serial once per bit period, shifting into rx_shift.
                // {rx_serial, rx_shift[7:1]} shifts new bit in at MSB position
                // so that after 8 shifts the byte ends up in the correct (LSB-first) order.
                RX_DATA: begin
                    if (rx_baud_cnt == BAUD_DIV-1) begin
                        rx_baud_cnt <= 0;
                        rx_shift    <= {rx_serial, rx_shift[7:1]};
                        if (rx_bit_cnt == 7) rx_state <= RX_STOP; // all 8 bits sampled
                        else rx_bit_cnt <= rx_bit_cnt + 1;
                    end else rx_baud_cnt <= rx_baud_cnt + 1;
                end
                // Wait out the stop bit period, then publish the received byte
                RX_STOP: begin
                    if (rx_baud_cnt == BAUD_DIV-1) begin
                        rx_data     <= rx_shift;
                        rx_valid    <= 1'b1;   // 1-cycle "data ready" pulse
                        rx_state    <= RX_IDLE;
                        rx_baud_cnt <= 0;
                    end else rx_baud_cnt <= rx_baud_cnt + 1;
                end
            endcase
        end
    end

endmodule
