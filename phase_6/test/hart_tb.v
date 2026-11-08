`timescale 1ns / 1ps
`default_nettype none

module hart_tb;

    // Graded cache configuration according to Phase 6 PDF Section 6.1:
    // PLRU (EVICT_POLICY=1), 4 ways, 2 KB, 16-byte blocks, prefetch on (PREFETCH_EN=1), 4-cycle memory
    parameter RESET_ADDR   = 32'h00400000;
    parameter EVICT_POLICY = 1;
    parameter NUM_WAYS     = 4;
    parameter CACHE_SIZE   = 2;
    parameter BLOCK_SIZE   = 16;
    parameter PREFETCH_EN  = 1;

    reg clk;
    reg rst;

    wire                    imem_rd;
    wire [31:0]             imem_rd_addr;
    reg                     imem_valid;
    reg  [BLOCK_SIZE*8-1:0] imem_rd_data;

    wire                    dmem_rd;
    wire [31:0]             dmem_rd_addr;
    reg                     dmem_valid;
    reg  [BLOCK_SIZE*8-1:0] dmem_rd_data;
    wire                    dmem_wr;
    wire [31:0]             dmem_wr_addr;
    wire [BLOCK_SIZE*8-1:0] dmem_wr_data;
    wire [BLOCK_SIZE-1:0]   dmem_wr_be;

    wire        retire_valid;
    wire [31:0] retire_inst;
    wire        retire_trap;
    wire        retire_halt;
    wire [ 4:0] retire_rs1_raddr;
    wire [ 4:0] retire_rs2_raddr;
    wire [31:0] retire_rs1_rdata;
    wire [31:0] retire_rs2_rdata;
    wire [ 4:0] retire_rd_waddr;
    wire [31:0] retire_rd_wdata;
    wire [31:0] retire_dmem_addr;
    wire [ 3:0] retire_dmem_mask;
    wire        retire_dmem_ren;
    wire        retire_dmem_wen;
    wire [31:0] retire_dmem_rdata;
    wire [31:0] retire_dmem_wdata;
    wire [31:0] retire_pc;
    wire [31:0] retire_next_pc;

    // Phase 6 Retire Ports for Floating-Point
    wire [ 4:0] retire_frs1_raddr;
    wire [31:0] retire_frs1_rdata;
    wire [ 4:0] retire_frs2_raddr;
    wire [31:0] retire_frs2_rdata;
    wire [ 4:0] retire_frs3_raddr;
    wire [31:0] retire_frs3_rdata;
    wire        retire_frd_wen;
    wire [ 4:0] retire_frd_waddr;
    wire [31:0] retire_frd_wdata;
    wire [ 4:0] retire_fflags;

    hart #(
        .RESET_ADDR   (RESET_ADDR),
        .FWD_EN       (1),
        .BYPASS_EN    (1),
        .EVICT_POLICY (EVICT_POLICY),
        .NUM_WAYS     (NUM_WAYS),
        .CACHE_SIZE   (CACHE_SIZE),
        .BLOCK_SIZE   (BLOCK_SIZE),
        .PREFETCH_EN  (PREFETCH_EN)
    ) dut (
        .i_clk               (clk),
        .i_rst               (rst),
        .o_imem_rd           (imem_rd),
        .o_imem_rd_addr      (imem_rd_addr),
        .i_imem_valid        (imem_valid),
        .i_imem_rd_data      (imem_rd_data),
        .o_dmem_rd           (dmem_rd),
        .o_dmem_rd_addr      (dmem_rd_addr),
        .i_dmem_valid        (dmem_valid),
        .i_dmem_rd_data      (dmem_rd_data),
        .o_dmem_wr           (dmem_wr),
        .o_dmem_wr_addr      (dmem_wr_addr),
        .o_dmem_wr_data      (dmem_wr_data),
        .o_dmem_wr_be        (dmem_wr_be),
        .o_retire_valid      (retire_valid),
        .o_retire_inst       (retire_inst),
        .o_retire_trap       (retire_trap),
        .o_retire_halt       (retire_halt),
        .o_retire_rs1_raddr  (retire_rs1_raddr),
        .o_retire_rs2_raddr  (retire_rs2_raddr),
        .o_retire_rs1_rdata  (retire_rs1_rdata),
        .o_retire_rs2_rdata  (retire_rs2_rdata),
        .o_retire_rd_waddr   (retire_rd_waddr),
        .o_retire_rd_wdata   (retire_rd_wdata),
        .o_retire_dmem_addr  (retire_dmem_addr),
        .o_retire_dmem_mask  (retire_dmem_mask),
        .o_retire_dmem_ren   (retire_dmem_ren),
        .o_retire_dmem_wen   (retire_dmem_wen),
        .o_retire_dmem_rdata (retire_dmem_rdata),
        .o_retire_dmem_wdata (retire_dmem_wdata),
        .o_retire_pc         (retire_pc),
        .o_retire_next_pc    (retire_next_pc),
        .o_retire_frs1_raddr (retire_frs1_raddr),
        .o_retire_frs1_rdata (retire_frs1_rdata),
        .o_retire_frs2_raddr (retire_frs2_raddr),
        .o_retire_frs2_rdata (retire_frs2_rdata),
        .o_retire_frs3_raddr (retire_frs3_raddr),
        .o_retire_frs3_rdata (retire_frs3_rdata),
        .o_retire_frd_wen    (retire_frd_wen),
        .o_retire_frd_waddr  (retire_frd_waddr),
        .o_retire_frd_wdata  (retire_frd_wdata),
        .o_retire_fflags     (retire_fflags)
    );

    // Memory Models:
    // IMEM: Base 0x00400000, 256 KB (up to 65536 words)
    // DMEM: Base 0x10010000, 128 KB (32768 words)
    reg [7:0] imem_bytes [0:262143];
    reg [7:0] dmem_bytes [0:131071];

    // Buffers for loading traces
    reg [31:0] raw_imem_words [0:65535];
    reg [31:0] expected_dmem  [0:32767];

    // Pipelined memory delay queues (4 cycles)
    localparam LATENCY = 4;

    reg [31:0] imem_q_addr [0:15];
    integer    imem_q_timer[0:15];
    integer    imem_q_head, imem_q_tail, imem_q_count;

    reg [31:0] dmem_q_addr [0:15];
    integer    dmem_q_timer[0:15];
    integer    dmem_q_head, dmem_q_tail, dmem_q_count;

    always #5 clk = ~clk;

    integer i, b, retired_count, cycle_count;
    integer mismatch_count;
    reg [31:0] prog_word;
    integer idx, didx;
    reg [31:0] imem_offset, dmem_offset;
    reg [31:0] actual_word, exp_word;

    // Helper task to report which section a failing dump address belongs to
    task report_section_error(input [31:0] addr, input [31:0] exp_val, input [31:0] got_val);
        reg [191:0] sec_name;
        begin
            if (addr >= 32'h10020000 && addr <= 32'h100217FF)
                sec_name = "muldiv (RV32M)";
            else if (addr >= 32'h10021800 && addr <= 32'h10022FFF)
                sec_name = "fp_move_compare (Moves, Sign Injection, Min/Max, Compares, Class)";
            else if (addr >= 32'h10023000 && addr <= 32'h100247FF)
                sec_name = "fp_convert (Conversions)";
            else if (addr >= 32'h10024800 && addr <= 32'h10025FFF)
                sec_name = "fp_add_mul (fadd, fsub, fmul)";
            else if (addr >= 32'h10026000 && addr <= 32'h100277FF)
                sec_name = "fp_fused (fmadd, fmsub, fnmsub, fnmadd)";
            else if (addr >= 32'h10027800 && addr <= 32'h10028FFF)
                sec_name = "fp_div_sqrt (fdiv, fsqrt and stalls)";
            else if (addr >= 32'h10029000 && addr <= 32'h1002A7FF)
                sec_name = "fp_csr (fflags, frm, fcsr, dynamic rm)";
            else if (addr >= 32'h1002A800 && addr <= 32'h1002BFFF)
                sec_name = "mixed (All IMF operations)";
            else if (addr >= 32'h1002C000 && addr <= 32'h1002D7FF)
                sec_name = "cache_stall (Multi-cycle next to cache misses)";
            else
                sec_name = "program_data (0x10010000 - 0x10017FFF)";

            $display("  [MISMATCH] Addr: 0x%08h | Exp: 0x%08h | Got: 0x%08h | Section: %0s",
                     addr, exp_val, got_val, sec_name);
        end
    endtask

    initial begin
        clk = 0;
        rst = 1;
        imem_valid = 0;
        dmem_valid = 0;
        imem_rd_data = 0;
        dmem_rd_data = 0;
        imem_q_head = 0; imem_q_tail = 0; imem_q_count = 0;
        dmem_q_head = 0; dmem_q_tail = 0; dmem_q_count = 0;
        retired_count = 0;
        cycle_count = 0;
        mismatch_count = 0;

        for (i = 0; i < 262144; i = i + 1) imem_bytes[i] = 8'h00;
        for (i = 0; i < 131072; i = i + 1) dmem_bytes[i] = 8'h00;

        $display("==================================================");
        $display("   LOADING TRACE FILES FOR PHASE 6 REGRESSION    ");
        $display("==================================================");

        $readmemh("traces/random_program.hex", raw_imem_words);
        for (i = 0; i < 36088; i = i + 1) begin
            prog_word = raw_imem_words[i];
            imem_bytes[i*4 + 0] = prog_word[ 7: 0];
            imem_bytes[i*4 + 1] = prog_word[15: 8];
            imem_bytes[i*4 + 2] = prog_word[23:16];
            imem_bytes[i*4 + 3] = prog_word[31:24];
        end
        $display("Loaded %0d words into IMEM (Base: 0x00400000).", 36088);

        $readmemh("traces/random_dmem.hex", expected_dmem);
        $display("Loaded %0d words of expected final DMEM (Base: 0x10010000).", 32768);

        repeat (10) @(posedge clk);
        @(negedge clk);
        rst = 0;
        $display("Processor reset released. Starting execution...\n");
    end

    // Instruction Memory Controller (4-cycle latency)
    always @(posedge clk) begin
        if (rst) begin
            imem_valid   <= 0;
            imem_rd_data <= 0;
            imem_q_head  <= 0;
            imem_q_tail  <= 0;
            imem_q_count <= 0;
        end else begin
            imem_valid <= 0;

            if (imem_rd) begin
                imem_q_addr[imem_q_tail]  <= imem_rd_addr;
                imem_q_timer[imem_q_tail] <= LATENCY;
                imem_q_tail  <= (imem_q_tail + 1) % 16;
                imem_q_count <= imem_q_count + 1;
            end

            for (i = 0; i < 16; i = i + 1) begin
                if (i < imem_q_count) begin
                    idx = (imem_q_head + i) % 16;
                    if (imem_q_timer[idx] > 0)
                        imem_q_timer[idx] <= imem_q_timer[idx] - 1;
                end
            end

            if (imem_q_count > 0 && imem_q_timer[imem_q_head] <= 1) begin
                imem_offset = imem_q_addr[imem_q_head] - 32'h00400000;
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    imem_rd_data[b*8 +: 8] <= imem_bytes[imem_offset + b];
                end
                imem_valid   <= 1;
                imem_q_head  <= (imem_q_head + 1) % 16;
                imem_q_count <= imem_q_count - 1;
            end
        end
    end

    // Data Memory Controller (4-cycle latency, supports writes)
    always @(posedge clk) begin
        if (rst) begin
            dmem_valid   <= 0;
            dmem_rd_data <= 0;
            dmem_q_head  <= 0;
            dmem_q_tail  <= 0;
            dmem_q_count <= 0;
        end else begin
            dmem_valid <= 0;

            if (dmem_wr) begin
                dmem_offset = dmem_wr_addr - 32'h10010000;
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    if (dmem_wr_be[b]) begin
                        dmem_bytes[dmem_offset + b] = dmem_wr_data[b*8 +: 8];
                    end
                end
            end

            if (dmem_rd) begin
                dmem_q_addr[dmem_q_tail]  <= dmem_rd_addr;
                dmem_q_timer[dmem_q_tail] <= LATENCY;
                dmem_q_tail  <= (dmem_q_tail + 1) % 16;
                dmem_q_count <= dmem_q_count + 1;
            end

            for (i = 0; i < 16; i = i + 1) begin
                if (i < dmem_q_count) begin
                    didx = (dmem_q_head + i) % 16;
                    if (dmem_q_timer[didx] > 0)
                        dmem_q_timer[didx] <= dmem_q_timer[didx] - 1;
                end
            end

            if (dmem_q_count > 0 && dmem_q_timer[dmem_q_head] <= 1) begin
                dmem_offset = dmem_q_addr[dmem_q_head] - 32'h10010000;
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    dmem_rd_data[b*8 +: 8] <= dmem_bytes[dmem_offset + b];
                end
                dmem_valid   <= 1;
                dmem_q_head  <= (dmem_q_head + 1) % 16;
                dmem_q_count <= dmem_q_count - 1;
            end
        end
    end

    // Instruction Retirement & Trace Monitor
    always @(posedge clk) begin
        if (!rst) begin
            cycle_count <= cycle_count + 1;

            if (retire_valid) begin
                retired_count <= retired_count + 1;

                if (retired_count % 5000 == 0 && retired_count > 0) begin
                    $display("Cycles: %0d | Retired: %0d instructions | Current PC: 0x%08h",
                             cycle_count, retired_count, retire_pc);
                end

                if (retire_halt) begin
                    $display("\n[HALT] EBREAK retired at cycle %0d! Total instructions retired: %0d",
                             cycle_count, retired_count);

                    // Wait 50 cycles for any last write-backs or flushes to finish
                    repeat (50) @(posedge clk);

                    $display("==================================================");
                    $display("        FINAL DATA MEMORY VERIFICATION            ");
                    $display("==================================================");

                    for (i = 0; i < 32768; i = i + 1) begin
                        actual_word = {
                            dmem_bytes[i*4 + 3],
                            dmem_bytes[i*4 + 2],
                            dmem_bytes[i*4 + 1],
                            dmem_bytes[i*4 + 0]
                        };
                        exp_word = expected_dmem[i];

                        if (actual_word !== exp_word) begin
                            mismatch_count = mismatch_count + 1;
                            if (mismatch_count <= 25) begin
                                report_section_error(32'h10010000 + i*4, exp_word, actual_word);
                            end
                        end
                    end

                    if (mismatch_count == 0) begin
                        $display("\n>>> SUCCESS: ALL 32,768 WORDS IN DATA MEMORY MATCH PERFECTLY! <<<");
                        $display("Phase 6 regression test passed with 0 errors.");
                    end else begin
                        $display("\n>>> FAILURE: %0d / 32,768 WORDS MISMATCHED! <<<", mismatch_count);
                        $display("Inspect the failing addresses above to pinpoint the failing module/section.");
                    end

                    $finish;
                end
            end

            // Timeout watchdog
            if (cycle_count > 5000000) begin
                $display("\n[ERROR] Simulation timeout reached (5,000,000 cycles). Execution aborted.");
                $finish;
            end
        end
    end

endmodule
`default_nettype wire
