`timescale 1ns / 1ps

// Setup check ("is my design gradable?") testbench for phase-4's hart.v.
//
// This is NOT a correctness test. It answers exactly one question: will a
// submission's hart INSTANTIATE, CLOCK, RESET, RETIRE and HALT under the
// same harness shape the autograder uses? Whether it computes the right
// answers is a separate question, answered by the two self-checks that ship
// alongside this one (hart_trace_tb.v and hart_cycle_trace_tb.v).
//
// Why this file exists at all: the graded path (source/rtl/hart_tb.v ->
// source/grade.py) decides pass/fail by PARSING PRINTED TEXT with a regex,
// and grade.py's parse_trace() silently DROPS any line that doesn't match.
// hart_trace_tb.v, by contrast, judges inside Verilog by comparing signals,
// so it never parses text at all. That means leftover $display debug output
// in a student's own RTL is completely invisible to hart_trace_tb.v but
// silently deletes retired instructions from the graded trace -- a correct
// processor scoring zero, with a diagnostic pointing at a CPU bug that
// doesn't exist. This testbench is the missing check for that class of
// problem. See ../../../documentation/DEVELOPER_GUIDE.md.
//
// ---------------------------------------------------------------------
// IMPORTANT, if you are editing source/rtl/hart_tb.v:
//
// Everything from the hart #(...) dut (...) instantiation through the reset
// sequence below is a VERBATIM COPY of source/rtl/hart_tb.v (its lines
// 121-196): same .RESET_ADDR(32'h00400000) parameter, same 29 ports in the
// same order, same word-addressed 4096-entry imem/dmem with the same [13:2]
// index slice, same combinational reads, same byte-lane masked synchronous
// write, same "program.mem" $readmemh load, same four-negedge reset. That
// verbatim-ness IS THE ENTIRE POINT of this file: it is what makes "it ran
// here" actually mean "it will instantiate in the grader." If hart_tb.v's
// instantiation, memory geometry or reset sequence ever changes, THIS FILE
// MUST BE RE-SYNCED or it starts quietly lying to students.
// ---------------------------------------------------------------------
//
// What is deliberately NOT copied from hart_tb.v, because this file ships to
// students and that one never does: the graded trace grammar, its
// opcode-classification logic, its MAX_CYCLES = 40000, and its
// "Total instructions retired"/"CPI:" trailer.
//
// Output contract: EVERY line this testbench prints begins with "SMOKE:".
// scripts/check_design.sh treats any other line on stdout as output coming
// from the submission's own RTL. Each line is emitted by a single $display
// (never a sequence of $write fragments like hart_tb.v uses), so a stray
// student $display lands on its own line, and a stray student $write shows
// up glued to the FRONT of one of ours -- both detectable, neither maskable.
module hart_smoke_tb ();

    localparam IMEM_WORDS = 4096;
    localparam DMEM_WORDS = 4096;
    localparam IMEM_ABITS = 12;  // $clog2(IMEM_WORDS)
    localparam DMEM_ABITS = 12;  // $clog2(DMEM_WORDS)
    // Generous for the ~10-instruction smoke program even on a deeply
    // stalling pipeline. Not hart_tb.v's 40000: a design that needs
    // thousands of cycles for ten instructions has a problem worth
    // reporting quickly rather than waiting out.
    localparam MAX_CYCLES = 500;

    reg i_clk;
    reg i_rst;

    wire [31:0] o_imem_raddr;
    reg  [31:0] i_imem_rdata;

    wire [31:0] o_dmem_addr;
    wire        o_dmem_ren;
    wire        o_dmem_wen;
    wire [31:0] o_dmem_wdata;
    wire [ 3:0] o_dmem_mask;
    reg  [31:0] i_dmem_rdata;

    wire        o_retire_valid;
    wire [31:0] o_retire_inst;
    wire        o_retire_trap;
    wire        o_retire_halt;
    wire [ 4:0] o_retire_rs1_raddr;
    wire [31:0] o_retire_rs1_rdata;
    wire [ 4:0] o_retire_rs2_raddr;
    wire [31:0] o_retire_rs2_rdata;
    wire [ 4:0] o_retire_rd_waddr;
    wire [31:0] o_retire_rd_wdata;
    wire [31:0] o_retire_pc;
    wire [31:0] o_retire_next_pc;

    wire [31:0] o_retire_dmem_addr;
    wire        o_retire_dmem_ren;
    wire        o_retire_dmem_wen;
    wire [ 3:0] o_retire_dmem_mask;
    wire [31:0] o_retire_dmem_wdata;
    wire [31:0] o_retire_dmem_rdata;

    // ---- VERBATIM from source/rtl/hart_tb.v -- see header comment. ----
    hart #(.RESET_ADDR(32'h00400000)) dut (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .o_imem_raddr (o_imem_raddr),
        .i_imem_rdata (i_imem_rdata),
        .o_dmem_addr  (o_dmem_addr),
        .o_dmem_ren   (o_dmem_ren),
        .o_dmem_wen   (o_dmem_wen),
        .o_dmem_wdata (o_dmem_wdata),
        .o_dmem_mask  (o_dmem_mask),
        .i_dmem_rdata (i_dmem_rdata),
        .o_retire_valid     (o_retire_valid),
        .o_retire_inst      (o_retire_inst),
        .o_retire_trap      (o_retire_trap),
        .o_retire_halt      (o_retire_halt),
        .o_retire_rs1_raddr (o_retire_rs1_raddr),
        .o_retire_rs1_rdata (o_retire_rs1_rdata),
        .o_retire_rs2_raddr (o_retire_rs2_raddr),
        .o_retire_rs2_rdata (o_retire_rs2_rdata),
        .o_retire_rd_waddr  (o_retire_rd_waddr),
        .o_retire_rd_wdata  (o_retire_rd_wdata),
        .o_retire_pc        (o_retire_pc),
        .o_retire_next_pc   (o_retire_next_pc),
        .o_retire_dmem_addr (o_retire_dmem_addr),
        .o_retire_dmem_ren  (o_retire_dmem_ren),
        .o_retire_dmem_wen  (o_retire_dmem_wen),
        .o_retire_dmem_mask (o_retire_dmem_mask),
        .o_retire_dmem_wdata(o_retire_dmem_wdata),
        .o_retire_dmem_rdata(o_retire_dmem_rdata)
    );

    reg [31:0] imem [0:IMEM_WORDS-1];
    always @(*) i_imem_rdata = imem[o_imem_raddr[IMEM_ABITS+1:2]];

    reg [31:0] dmem [0:DMEM_WORDS-1];
    always @(*) i_dmem_rdata = dmem[o_dmem_addr[DMEM_ABITS+1:2]];

    always @(posedge i_clk) begin
        if (o_dmem_wen) begin
            if (o_dmem_mask[0]) dmem[o_dmem_addr[DMEM_ABITS+1:2]][ 7: 0] <= o_dmem_wdata[ 7: 0];
            if (o_dmem_mask[1]) dmem[o_dmem_addr[DMEM_ABITS+1:2]][15: 8] <= o_dmem_wdata[15: 8];
            if (o_dmem_mask[2]) dmem[o_dmem_addr[DMEM_ABITS+1:2]][23:16] <= o_dmem_wdata[23:16];
            if (o_dmem_mask[3]) dmem[o_dmem_addr[DMEM_ABITS+1:2]][31:24] <= o_dmem_wdata[31:24];
        end
    end
    // ---- end verbatim block (the reset sequence below is verbatim too). ----

    integer i;
    integer cycles, run;
    integer num_instructions;
    integer num_loads, num_stores, num_writes;
    reg     halted;

    initial begin
        i_clk = 1;
        i_rst = 0;
        for (i = 0; i < IMEM_WORDS; i = i + 1) imem[i] = 32'h0;
        for (i = 0; i < DMEM_WORDS; i = i + 1) dmem[i] = 32'h0;

        $display("SMOKE: loading program.mem");
        $readmemh("program.mem", imem);

        $display("SMOKE: resetting hart");
        @(negedge i_clk); i_rst = 1;
        @(negedge i_clk);
        @(negedge i_clk);
        @(negedge i_clk); i_rst = 0;

        $display("SMOKE: running");
        cycles = 0;
        run = 1;
        halted = 0;
        num_instructions = 0;
        num_loads = 0;
        num_stores = 0;
        num_writes = 0;

        while (run) begin
            @(posedge i_clk);
            cycles = cycles + 1;

            if (o_retire_valid) begin
                num_instructions = num_instructions + 1;
                if (o_retire_rd_waddr != 5'd0) num_writes = num_writes + 1;
                if (o_retire_dmem_ren)         num_loads  = num_loads  + 1;
                if (o_retire_dmem_wen)         num_stores = num_stores + 1;

                // One atomic $display -- deliberately not hart_tb.v's
                // sequence of $write fragments (see header comment).
                $display("SMOKE:   retired #%0d  pc=%08h  inst=%08h",
                         num_instructions, o_retire_pc, o_retire_inst);

                if (o_retire_halt) begin
                    halted = 1;
                    run = 0;
                end
            end

            if (cycles > MAX_CYCLES) begin
                $display("SMOKE: gave up after %0d cycles without halting", MAX_CYCLES);
                run = 0;
            end
        end

        $display("SMOKE: cycles=%0d retired=%0d writes=%0d loads=%0d stores=%0d",
                 cycles, num_instructions, num_writes, num_loads, num_stores);
        if (halted)
            $display("SMOKE: HALTED");
        else
            $display("SMOKE: DID NOT HALT");
        $display("SMOKE: END");
        $finish;
    end

    // Redundant backstop, mirroring hart_tb.v's own watchdog thread: if the
    // loop above can never make progress, this still ends the simulation
    // rather than hanging the student's terminal.
    integer watchdog;
    initial begin
        for (watchdog = 0; watchdog < MAX_CYCLES + 100; watchdog = watchdog + 1) begin
            @(posedge i_clk);
        end
        $display("SMOKE: DID NOT HALT");
        $display("SMOKE: END");
        $finish;
    end

    always
        #5 i_clk = ~i_clk;
endmodule
