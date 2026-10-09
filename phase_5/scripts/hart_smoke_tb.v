`timescale 1ns / 1ps

// Setup smoke test for phase-5's hart.v, run by scripts/check_design.sh.
//
// It answers one question: does the design instantiate under the same
// harness as the grading testbench (same parameters, same line-wide memory
// ports, same memory model, same reset), retire instructions and halt? It
// runs a short program and compares nothing: no memory image, no pass/fail.
//
//   iverilog -g2005 -s hart_smoke_tb -o sim hart_smoke_tb.v <your .v files>
//   vvp -n sim +program=smoke_program.hex
//
// The plusarg defaults to the path above, relative to the directory vvp runs
// in (scripts/).
//
// Output contract: every line this testbench prints starts with "SMOKE:", so
// any other line on stdout came from the design under test.
module hart_smoke_tb #(
    // Cache configuration under test (default: the course base config).
    parameter EVICT_POLICY = 0,    // 0 = LRU, 1 = PLRU
    parameter NUM_WAYS     = 128,  // fully associative at 16 KB / 128 B
    parameter CACHE_SIZE   = 16,   // KB
    parameter BLOCK_SIZE   = 128,  // bytes per line
    parameter PREFETCH_EN  = 0,
    // Main memory: a line read requested in cycle N is answered in cycle
    // N + MEM_DELAY, so a cache miss stalls the pipeline MEM_DELAY cycles.
    // Must be at least 2 (the read pipeline below has MEM_DELAY-1 stages).
    parameter MEM_DELAY    = 4
) ();

    localparam IMEM_WORDS = 1 << 15;  // same memory as the grading testbench
    localparam DMEM_WORDS = 16384;
    localparam IMEM_ABITS = 15;  // $clog2(IMEM_WORDS)
    localparam DMEM_ABITS = 14;  // $clog2(DMEM_WORDS)
    // Only stops a design that never halts. The reference needs well under
    // this on the smoke program.
    localparam MAX_CYCLES = 2000;

    localparam BLOCK_WIDTH    = BLOCK_SIZE * 8;
    localparam WORDS_PER_LINE = BLOCK_SIZE / 4;

    reg i_clk;
    reg i_rst;

    wire                   o_imem_rd;
    wire [31:0]            o_imem_rd_addr;
    reg                    i_imem_valid;
    reg  [BLOCK_WIDTH-1:0] i_imem_rd_data;

    wire                   o_dmem_rd;
    wire [31:0]            o_dmem_rd_addr;
    reg                    i_dmem_valid;
    reg  [BLOCK_WIDTH-1:0] i_dmem_rd_data;
    wire                   o_dmem_wr;
    wire [31:0]            o_dmem_wr_addr;
    wire [BLOCK_WIDTH-1:0] o_dmem_wr_data;
    wire [BLOCK_SIZE-1:0]  o_dmem_wr_be;

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

    // The retiring instruction's own memory-op fields, carried through
    // hart.v's pipeline registers and reported on the cycle that
    // instruction reaches writeback.
    wire [31:0] o_retire_dmem_addr;
    wire        o_retire_dmem_ren;
    wire        o_retire_dmem_wen;
    wire [ 3:0] o_retire_dmem_mask;
    wire [31:0] o_retire_dmem_wdata;
    wire [31:0] o_retire_dmem_rdata;

    hart #(
        .RESET_ADDR  (32'h00400000),
        .EVICT_POLICY(EVICT_POLICY),
        .NUM_WAYS    (NUM_WAYS),
        .CACHE_SIZE  (CACHE_SIZE),
        .BLOCK_SIZE  (BLOCK_SIZE),
        .PREFETCH_EN (PREFETCH_EN)
    ) dut (
        .i_clk          (i_clk),
        .i_rst          (i_rst),
        .o_imem_rd      (o_imem_rd),
        .o_imem_rd_addr (o_imem_rd_addr),
        .i_imem_valid   (i_imem_valid),
        .i_imem_rd_data (i_imem_rd_data),
        .o_dmem_rd      (o_dmem_rd),
        .o_dmem_rd_addr (o_dmem_rd_addr),
        .i_dmem_valid   (i_dmem_valid),
        .i_dmem_rd_data (i_dmem_rd_data),
        .o_dmem_wr      (o_dmem_wr),
        .o_dmem_wr_addr (o_dmem_wr_addr),
        .o_dmem_wr_data (o_dmem_wr_data),
        .o_dmem_wr_be   (o_dmem_wr_be),
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

    // ---- Instruction memory: word array, read-only block port with a
    // MEM_DELAY-cycle pipelined read. ----
    reg [31:0] imem [0:IMEM_WORDS-1];
    reg        imem_rd_valid [0:MEM_DELAY-2];
    reg [31:0] imem_rd_addr  [0:MEM_DELAY-2];
    integer ib, ik;
    always @(posedge i_clk) begin
        if (i_rst) begin
            i_imem_valid <= 1'b0;
            for (ik = 0; ik < MEM_DELAY-1; ik = ik + 1) imem_rd_valid[ik] <= 1'b0;
        end else begin
            i_imem_valid <= imem_rd_valid[MEM_DELAY-2];
            if (imem_rd_valid[MEM_DELAY-2])
                for (ib = 0; ib < WORDS_PER_LINE; ib = ib + 1)
                    i_imem_rd_data[ib*32 +: 32] <= imem[imem_rd_addr[MEM_DELAY-2][IMEM_ABITS+1:2] + ib];
            for (ik = MEM_DELAY-2; ik > 0; ik = ik - 1) begin
                imem_rd_valid[ik] <= imem_rd_valid[ik-1];
                imem_rd_addr[ik]  <= imem_rd_addr[ik-1];
            end
            imem_rd_valid[0] <= o_imem_rd;
            imem_rd_addr[0]  <= o_imem_rd_addr;
        end
    end

    // ---- Data memory: word array, block read port with the same delay,
    // byte-enabled block write applied before the read of the same edge. ----
    reg [31:0] dmem [0:DMEM_WORDS-1];
    reg        dmem_rd_valid [0:MEM_DELAY-2];
    reg [31:0] dmem_rd_addr  [0:MEM_DELAY-2];
    integer db, dk;
    always @(posedge i_clk) begin
        if (i_rst) begin
            i_dmem_valid <= 1'b0;
            for (dk = 0; dk < MEM_DELAY-1; dk = dk + 1) dmem_rd_valid[dk] <= 1'b0;
        end else begin
            if (o_dmem_wr)
                for (db = 0; db < BLOCK_SIZE; db = db + 1)
                    if (o_dmem_wr_be[db])
                        dmem[o_dmem_wr_addr[DMEM_ABITS+1:2] + (db >> 2)][(db & 3)*8 +: 8] = o_dmem_wr_data[db*8 +: 8];
            i_dmem_valid <= dmem_rd_valid[MEM_DELAY-2];
            if (dmem_rd_valid[MEM_DELAY-2])
                for (db = 0; db < WORDS_PER_LINE; db = db + 1)
                    i_dmem_rd_data[db*32 +: 32] <= dmem[dmem_rd_addr[MEM_DELAY-2][DMEM_ABITS+1:2] + db];
            for (dk = MEM_DELAY-2; dk > 0; dk = dk - 1) begin
                dmem_rd_valid[dk] <= dmem_rd_valid[dk-1];
                dmem_rd_addr[dk]  <= dmem_rd_addr[dk-1];
            end
            dmem_rd_valid[0] <= o_dmem_rd;
            dmem_rd_addr[0]  <= o_dmem_rd_addr;
        end
    end

    reg [8*512-1:0] program_path;

    integer i, fd;
    integer cycles, run, halted, retired;
    initial begin
        i_clk = 1;
        i_rst = 0;
        for (i = 0; i < IMEM_WORDS; i = i + 1) imem[i] = 32'h0;
        for (i = 0; i < DMEM_WORDS; i = i + 1) dmem[i] = 32'h0;

        $display("SMOKE: CONFIG: EVICT_POLICY=%0d NUM_WAYS=%0d CACHE_SIZE=%0d BLOCK_SIZE=%0d PREFETCH_EN=%0d MEM_DELAY=%0d",
                 EVICT_POLICY, NUM_WAYS, CACHE_SIZE, BLOCK_SIZE, PREFETCH_EN, MEM_DELAY);
        if (MEM_DELAY < 2) begin
            $display("SMOKE: ERROR: MEM_DELAY must be at least 2.");
            $finish;
        end

        if (!$value$plusargs("program=%s", program_path))
            program_path = "smoke_program.hex";
        // $readmemh only warns on a missing file, which would look like a
        // broken hart, so check the file first.
        fd = $fopen(program_path, "r");
        if (fd == 0) begin
            $display("SMOKE: ERROR: cannot open program file %0s (pass +program=<path>)", program_path);
            $finish;
        end
        $fclose(fd);
        $readmemh(program_path, imem);

        // Reset the dut.
        @(negedge i_clk); i_rst = 1;
        @(negedge i_clk);
        @(negedge i_clk);
        @(negedge i_clk); i_rst = 0;

        cycles = 0;
        run = 1;
        halted = 0;
        retired = 0;
        while (run) begin
            @(posedge i_clk);
            cycles = cycles + 1;
            if (o_retire_valid) begin
                retired = retired + 1;
                // One line per retirement, so check_design.sh can still
                // count them if the run is cut short before the summary.
                $display("SMOKE:   retired #%0d  pc=%08h  inst=%08h",
                         retired, o_retire_pc, o_retire_inst);
                if (o_retire_halt) begin
                    halted = 1;
                    run = 0;
                end
            end
            if (cycles >= MAX_CYCLES)
                run = 0;
        end

        $display("SMOKE: cycles=%0d retired=%0d halted=%0d", cycles, retired, halted);
        if (!halted)
            $display("SMOKE: DID NOT HALT");
        $display("SMOKE: END");
        $finish;
    end

    always
        #5 i_clk = ~i_clk;
endmodule
