// Top-level wrapper for real synthesis/implementation numbers on the
// Digilent Nexys A7-100T (Xilinx Artix-7, xc7a100tcsg324-1).
//
// This is NOT a functional demo (SW/BTNC are not a real market data feed) --
// it exists purely to give Vivado real I/O pins with IOSTANDARDs and a real
// 100MHz clock constraint, so synthesis + implementation produce accurate
// utilization and timing (Fmax) numbers for market_feed_top instead of
// simulation-only estimates.
//
//   SW[7:0]   -> byte_in[7:0]   (manually set a byte's value with the switches)
//   BTNC      -> byte_valid     (press to "send" the current switch byte)
//   CPU_RESETN-> rst            (active-low button -> active-high internal reset)
//   LED[7:0]  -> loopback of SW, so you can see what you're about to send
//   LED[13:8] -> last_slot[5:0]
//   LED[14]   -> op_done
//   LED[15]   -> op_success

module nexys_a7_top (
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,
    input  wire [7:0]  SW,
    input  wire        BTNC,
    output wire [15:0] LED
);

    wire rst = ~CPU_RESETN;

    wire       op_done;
    wire       op_success;
    wire [5:0] last_slot;

    market_feed_top dut (
        .clk        (CLK100MHZ),
        .rst        (rst),
        .byte_in    (SW),
        .byte_valid (BTNC),
        .op_done    (op_done),
        .op_success (op_success),
        .last_slot  (last_slot)
    );

    assign LED[7:0]   = SW;
    assign LED[13:8]  = last_slot;
    assign LED[14]    = op_done;
    assign LED[15]    = op_success;

endmodule
