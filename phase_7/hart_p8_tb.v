`timescale 1ns / 1ps
`default_nettype none

// Phase 8 Multi-Workload Benchmark & Retire-Trace Dump Testbench
// Runs 7 diverse workloads (loops/branches, calls/returns, speculative load-branch,
// cache-miss+forwarding, div/mul, fpu/fdiv/fcsr, and mispredict+trap recovery),
// checks expected architectural results, and dumps retire traces with +dump_file=<path>
// so baseline Phase 7 and optimized Phase 8 can be compared instruction-for-instruction.
module hart_p8_tb;

    parameter RESET_ADDR     = 32'h00400000;
    parameter EVICT_POLICY   = 1;
    parameter NUM_WAYS       = 4;
    parameter CACHE_SIZE     = 2;
    parameter BLOCK_SIZE     = 16;
    parameter PREFETCH_EN    = 1;
    parameter WRITE_ALLOCATE = 0;

    reg clk, rst;

    wire [31:0] imem_raddr;
    wire [31:0] imem_rdata;

    wire                    mem_rd;
    wire [31:0]             mem_rd_addr;
    reg                     mem_valid;
    reg  [BLOCK_SIZE*8-1:0] mem_rd_data;
    wire                    mem_wr;
    wire [31:0]             mem_wr_addr;
    wire [BLOCK_SIZE*8-1:0] mem_wr_data;
    wire [BLOCK_SIZE-1:0]   mem_wr_be;

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

    wire        complete_valid;
    wire [ 2:0] complete_tag;
    wire [ 4:0] complete_rd;
    wire [31:0] complete_value;
    wire [31:0] complete_pc;

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
        .RESET_ADDR     (RESET_ADDR),
        .BYPASS_EN      (1),
        .EVICT_POLICY   (EVICT_POLICY),
        .NUM_WAYS       (NUM_WAYS),
        .CACHE_SIZE     (CACHE_SIZE),
        .BLOCK_SIZE     (BLOCK_SIZE),
        .PREFETCH_EN    (PREFETCH_EN),
        .WRITE_ALLOCATE (WRITE_ALLOCATE)
    ) dut (
        .i_clk               (clk),
        .i_rst               (rst),
        .o_imem_raddr        (imem_raddr),
        .i_imem_rdata        (imem_rdata),
        .i_mem_ready         (1'b1),
        .i_mem_valid         (mem_valid),
        .i_mem_rd_data       (mem_rd_data),
        .o_mem_rd            (mem_rd),
        .o_mem_wr            (mem_wr),
        .o_mem_rd_addr       (mem_rd_addr),
        .o_mem_wr_addr       (mem_wr_addr),
        .o_mem_wr_data       (mem_wr_data),
        .o_mem_wr_be         (mem_wr_be),
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
        .o_complete_valid    (complete_valid),
        .o_complete_tag      (complete_tag),
        .o_complete_rd       (complete_rd),
        .o_complete_value    (complete_value),
        .o_complete_pc       (complete_pc),
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

    localparam IMEM_WORDS = 1024;
    reg [31:0] imem [0:IMEM_WORDS-1];
    wire [31:0] imem_idx = (imem_raddr - RESET_ADDR) >> 2;
    assign imem_rdata = (imem_raddr >= RESET_ADDR && imem_idx < IMEM_WORDS) ?
                        imem[imem_idx] : 32'h00000013;

    localparam DMEM_BYTES = 65536;
    reg [7:0] dmem_bytes [0:DMEM_BYTES-1];
    integer   mem_latency;

    reg [31:0] dmem_q_addr  [0:15];
    integer    dmem_q_timer [0:15];
    integer    dmem_q_head, dmem_q_tail, dmem_q_count;
    integer    b, mi, didx;
    reg [31:0] dmem_offset;

    always @(posedge clk) begin
        if (rst) begin
            mem_valid    <= 1'b0;
            mem_rd_data  <= {(BLOCK_SIZE*8){1'b0}};
            dmem_q_head  <= 0;
            dmem_q_tail  <= 0;
            dmem_q_count <= 0;
        end else begin
            mem_valid <= 1'b0;

            if (mem_wr) begin
                dmem_offset = mem_wr_addr - 32'h10010000;
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    if (mem_wr_be[b] && (dmem_offset + b < DMEM_BYTES))
                        dmem_bytes[dmem_offset + b] = mem_wr_data[b*8 +: 8];
                end
            end

            if (mem_rd) begin
                dmem_q_addr[dmem_q_tail]  <= mem_rd_addr;
                dmem_q_timer[dmem_q_tail] <= mem_latency;
                dmem_q_tail  <= (dmem_q_tail + 1) % 16;
            end

            for (mi = 0; mi < 16; mi = mi + 1) begin
                if (mi < dmem_q_count) begin
                    didx = (dmem_q_head + mi) % 16;
                    if (dmem_q_timer[didx] > 0)
                        dmem_q_timer[didx] <= dmem_q_timer[didx] - 1;
                end
            end

            if (dmem_q_count > 0 && dmem_q_timer[dmem_q_head] <= 1) begin
                dmem_offset = dmem_q_addr[dmem_q_head] - 32'h10010000;
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    mem_rd_data[b*8 +: 8] <= (dmem_offset + b < DMEM_BYTES) ?
                                             dmem_bytes[dmem_offset + b] : 8'h00;
                end
                mem_valid    <= 1'b1;
                dmem_q_head  <= (dmem_q_head + 1) % 16;
            end
            dmem_q_count <= dmem_q_count + (mem_rd ? 1 : 0) - ((dmem_q_count > 0 && dmem_q_timer[dmem_q_head] <= 1) ? 1 : 0);
        end
    end

    always #5 clk = ~clk;

    integer trace_fd;
    reg [2047:0] dump_path;
    integer passed, failed, cycle, retire_count;
    reg [31:0] last_rd_val [0:31];
    reg [31:0] last_frd_val [0:31];

    task clear_memories;
        integer k;
        begin
            for (k = 0; k < IMEM_WORDS; k = k + 1) imem[k] = 32'h00000013;
            for (k = 0; k < DMEM_BYTES; k = k + 1) dmem_bytes[k] = 8'h00;
            for (k = 0; k < 32; k = k + 1) begin
                last_rd_val[k]  = 32'd0;
                last_frd_val[k] = 32'd0;
            end
        end
    endtask

    task run_bench(input [255:0] name, input integer max_cycles);
        integer cyc;
        reg done;
        begin
            retire_count = 0;
            cycle        = 0;
            done         = 1'b0;

            @(negedge clk);
            rst = 1'b1;
            repeat (3) @(posedge clk);
            @(negedge clk);
            rst = 1'b0;

            for (cyc = 0; cyc < max_cycles && !done; cyc = cyc + 1) begin
                @(negedge clk);
                cycle = cycle + 1;
                if (retire_valid) begin
                    if (retire_rd_waddr != 5'd0)
                        last_rd_val[retire_rd_waddr] = retire_rd_wdata;
                    if (retire_frd_wen)
                        last_frd_val[retire_frd_waddr] = retire_frd_wdata;
                    if (trace_fd != 0) begin
                        $fwrite(trace_fd, "%0s %0d %08h %08h %b %b %02d %08h %02d %08h %02d %08h %b %02d %08h %02h %b %b %08h %01h %08h %08h\n",
                                name, retire_count, retire_pc, retire_inst, retire_trap, retire_halt,
                                retire_rs1_raddr, retire_rs1_rdata, retire_rs2_raddr, retire_rs2_rdata,
                                retire_rd_waddr, retire_rd_wdata,
                                retire_frd_wen, retire_frd_waddr, retire_frd_wdata, retire_fflags,
                                retire_dmem_ren, retire_dmem_wen, retire_dmem_addr, retire_dmem_mask,
                                retire_dmem_wdata, retire_next_pc);
                    end
                    retire_count = retire_count + 1;
                    if (retire_halt)
                        done = 1'b1;
                end
            end

            if (!done) begin
                $display("[FAIL] %0s timed out after %0d cycles!", name, max_cycles);
                failed = failed + 1;
            end else begin
                $display("[BENCH] %-22s | retired=%5d | cycles=%6d | CPI=%0.4f | IPC=%0.4f",
                         name, retire_count, cycle, (1.0 * cycle) / retire_count, (1.0 * retire_count) / cycle);
            end
        end
    endtask

    task check_eq32(input [511:0] msg, input [31:0] got, input [31:0] exp);
        begin
            if (got === exp) begin
                passed = passed + 1;
            end else begin
                $display("[FAIL] %0s: got 0x%08h (%0d), exp 0x%08h (%0d)", msg, got, got, exp, exp);
                failed = failed + 1;
            end
        end
    endtask

    integer idx;

    initial begin
        clk         = 1'b0;
        rst         = 1'b1;
        mem_latency = 4;
        passed      = 0;
        failed      = 0;
        trace_fd    = 0;

        if ($value$plusargs("dump_file=%s", dump_path)) begin
            trace_fd = $fopen(dump_path, "w");
        end

        // =====================================================================
        // B1: Tight Loop + Correlated Conditional Branches (60 iterations)
        // Sum even numbers vs odd numbers: sum 1..60 = 1830, evens = 930
        // =====================================================================
        clear_memories();
        // 0x00: addi x1, x0, 0      (sum_all = 0)
        // 0x04: addi x2, x0, 1      (i = 1)
        // 0x08: addi x3, x0, 61     (limit = 61)
        // 0x0c: addi x4, x0, 0      (sum_even = 0)
        // loop (0x10):
        // 0x10: add  x1, x1, x2     (sum_all += i)
        // 0x14: andi x5, x2, 1      (i & 1)
        // 0x18: bne  x5, x0, +8     (if odd -> 0x20)
        // 0x1c: add  x4, x4, x2     (sum_even += i)
        // 0x20: addi x2, x2, 1      (i++)
        // 0x24: blt  x2, x3, -20    (-> 0x10)
        // 0x28: ebreak
        imem[ 0] = 32'h00000093;
        imem[ 1] = 32'h00100113;
        imem[ 2] = 32'h03d00193;
        imem[ 3] = 32'h00000213;
        imem[ 4] = 32'h002080b3;
        imem[ 5] = 32'h00117293;
        imem[ 6] = 32'h00029463;
        imem[ 7] = 32'h00220233;
        imem[ 8] = 32'h00110113;
        imem[ 9] = 32'hfe3146e3;
        imem[10] = 32'h00100073;
        run_bench("B1_loop_branches", 5000);
        check_eq32("B1 sum_all", last_rd_val[1], 32'd1830);
        check_eq32("B1 sum_even", last_rd_val[4], 32'd930);

        // =====================================================================
        // B2: Function Call & Return Heavy (JAL + JALR via RAS)
        // Calls leaf func 40 times: sum(i*3 + 1) for i=1..40 -> 2500
        // =====================================================================
        clear_memories();
        // 0x00: addi x10, x0, 0     (acc = 0)
        // 0x04: addi x11, x0, 1     (i = 1)
        // 0x08: addi x12, x0, 41    (limit = 41)
        // loop (0x0c):
        // 0x0c: jal  x1, +16        (call helper at 0x1c)
        // 0x10: add  x10, x10, x13  (acc += ret)
        // 0x14: addi x11, x11, 1    (i++)
        // 0x18: blt  x11, x12, -12  (-> 0x0c)
        // 0x1c: jal  x0, +16        (-> 0x2c ebreak)
        // Wait: put ebreak at 0x1c and helper at 0x20!
        // Let's re-encode cleanly:
        // 0x0c: jal  x1, +20        (-> 0x20)
        // 0x10: add  x10, x10, x13
        // 0x14: addi x11, x11, 1
        // 0x18: blt  x11, x12, -12  (-> 0x0c)
        // 0x1c: ebreak
        // helper (0x20):
        // 0x20: slli x13, x11, 1    (2*i)
        // 0x24: add  x13, x13, x11  (3*i)
        // 0x28: addi x13, x13, 1    (3*i + 1)
        // 0x2c: jalr x0, 0(x1)      (ret)
        imem[ 0] = 32'h00000513;
        imem[ 1] = 32'h00100593;
        imem[ 2] = 32'h02900613;
        imem[ 3] = 32'h014000ef;
        imem[ 4] = 32'h00d50533;
        imem[ 5] = 32'h00158593;
        imem[ 6] = 32'hfec5cae3;
        imem[ 7] = 32'h00100073;
        imem[ 8] = 32'h00159693;
        imem[ 9] = 32'h00b686b3;
        imem[10] = 32'h00168693;
        imem[11] = 32'h00008067;
        run_bench("B2_call_return_ras", 5000);
        check_eq32("B2 acc", last_rd_val[10], 32'd2500);

        // =====================================================================
        // B3: Speculative Execution Past Unresolved Load-Dependent Branch
        // =====================================================================
        clear_memories();
        mem_latency = 8;
        // Preload 16 words: dmem[i*4] = 1 (so branch lw->bne is NOT taken 15 times, taken on 16th? Let's make all 1 so bne not taken, or beq taken!)
        for (idx = 0; idx < 16; idx = idx + 1) begin
            dmem_bytes[idx*64 + 0] = 8'h01; // distinct cache sets (stride 64 bytes!)
        end
        // 0x00: lui  x10, 0x10010
        // 0x04: addi x11, x0, 16    (count = 16)
        // 0x08: addi x12, x0, 0     (acc = 0)
        // loop (0x0c):
        // 0x0c: lw   x1, 0(x10)     (cache miss 8 cyc -> x1 = 1)
        // 0x10: beq  x1, x0, +24    (depends on lw! predicted NT -> allows independent ops behind it!)
        // 0x14: addi x2, x0, 3
        // 0x18: addi x3, x2, 4
        // 0x1c: add  x12, x12, x3
        // 0x20: addi x10, x10, 64
        // 0x24: addi x11, x11, -1
        // 0x28: bne  x11, x0, -28   (-> 0x0c)
        // 0x2c: ebreak
        imem[ 0] = 32'h10010537;
        imem[ 1] = 32'h01000593;
        imem[ 2] = 32'h00000613;
        imem[ 3] = 32'h00052083;
        imem[ 4] = 32'h00008c63;
        imem[ 5] = 32'h00300113;
        imem[ 6] = 32'h00410193;
        imem[ 7] = 32'h00360633;
        imem[ 8] = 32'h04050513;
        imem[ 9] = 32'hfff58593;
        imem[10] = 32'hfe0592e3;
        imem[11] = 32'h00100073;
        run_bench("B3_spec_load_branch", 5000);
        check_eq32("B3 acc", last_rd_val[12], 32'd112);

        // =====================================================================
        // B4: Multi-Miss Overlap + Store-to-Load Forwarding
        // =====================================================================
        clear_memories();
        mem_latency = 8;
        for (idx = 0; idx < 8; idx = idx + 1) begin
            dmem_bytes[idx*64 + 0] = idx + 10;
        end
        // Store-to-load forwarding behind slow miss + 4 overlapped misses
        // 0x00: lui  x10, 0x10010
        // 0x04: lw   x1, 0(x10)      (miss 1)
        // 0x08: lw   x2, 64(x10)     (miss 2)
        // 0x0c: lw   x3, 128(x10)    (miss 3)
        // 0x10: lw   x4, 192(x10)    (miss 4)
        // 0x14: sw   x1, 256(x10)    (store depends on miss 1)
        // 0x18: lw   x5, 256(x10)    (forwards from sw!)
        // 0x1c: sw   x2, 320(x10)
        // 0x20: lw   x6, 320(x10)    (forwards from sw!)
        // 0x24: add  x7, x3, x4
        // 0x28: add  x7, x7, x5
        // 0x2c: add  x7, x7, x6      (10+11+12+13 = 46)
        // 0x30: ebreak
        imem[ 0] = 32'h10010537;
        imem[ 1] = 32'h00052083;
        imem[ 2] = 32'h04052103;
        imem[ 3] = 32'h08052183;
        imem[ 4] = 32'h0c052203;
        imem[ 5] = 32'h10152023;
        imem[ 6] = 32'h10052283;
        imem[ 7] = 32'h14252023;
        imem[ 8] = 32'h14052303;
        imem[ 9] = 32'h004183b3;
        imem[10] = 32'h005383b3;
        imem[11] = 32'h006383b3;
        imem[12] = 32'h00100073;
        run_bench("B4_mshr_st2ld_fwd", 5000);
        check_eq32("B4 sum", last_rd_val[7], 32'd46);

        // =====================================================================
        // B5: Integer DIV/REM + Independent ALU/MUL Compute
        // =====================================================================
        clear_memories();
        mem_latency = 4;
        // 20 iterations: div + independent mul + alu ops
        // 0x00: lui  x1, 0x12345
        // 0x04: addi x2, x0, 17
        // 0x08: addi x3, x0, 20     (cnt)
        // 0x0c: addi x4, x0, 0      (acc)
        // loop (0x10):
        // 0x10: div  x5, x1, x2     (slow div)
        // 0x14: rem  x6, x1, x2     (2nd div)
        // 0x18: mul  x7, x3, x2     (independent mul!)
        // 0x1c: add  x4, x4, x7     (independent add!)
        // 0x20: add  x4, x4, x5
        // 0x24: add  x4, x4, x6
        // 0x28: addi x1, x1, 13
        // 0x2c: addi x3, x3, -1
        // 0x30: bne  x3, x0, -32    (-> 0x10)
        // 0x34: ebreak
        imem[ 0] = 32'h123450b7;
        imem[ 1] = 32'h01100113;
        imem[ 2] = 32'h01400193;
        imem[ 3] = 32'h00000213;
        imem[ 4] = 32'h0220c2b3;
        imem[ 5] = 32'h0220e333;
        imem[ 6] = 32'h022183b3;
        imem[ 7] = 32'h00720233;
        imem[ 8] = 32'h00520233;
        imem[ 9] = 32'h00620233;
        imem[10] = 32'h00d08093;
        imem[11] = 32'hfff18193;
        imem[12] = 32'hfe0190e3;
        imem[13] = 32'h00100073;
        run_bench("B5_div_mul_alu", 10000);

        // =====================================================================
        // B6: FPU / FDIV / FSQRT / FCSR Heavy
        // =====================================================================
        clear_memories();
        // 0x00: lui  x1, 0x41200     (10.0f)
        // 0x04: lui  x2, 0x40400     (3.0f)
        // 0x08: fmv.w.x f1, x1
        // 0x0c: fmv.w.x f2, x2
        // 0x10: addi x3, x0, 15
        // 0x14: fmv.w.x f0, x0       (acc = 0.0f)
        // loop (0x18):
        // 0x18: fdiv.s  f3, f1, f2   (10/3)
        // 0x1c: fsqrt.s f4, f1       (sqrt(10))
        // 0x20: fadd.s  f0, f0, f3
        // 0x24: fadd.s  f0, f0, f4
        // 0x28: csrrs   x4, fflags, x0
        // 0x2c: addi    x3, x3, -1
        // 0x30: bne     x3, x0, -24  (-> 0x18)
        // 0x34: ebreak
        imem[ 0] = 32'h412000b7;
        imem[ 1] = 32'h40400137;
        imem[ 2] = 32'hf00080d3;
        imem[ 3] = 32'hf0010153;
        imem[ 4] = 32'h00f00193;
        imem[ 5] = 32'hf0000053;
        imem[ 6] = 32'h182081d3;
        imem[ 7] = 32'h58008253;
        imem[ 8] = 32'h00300053;
        imem[ 9] = 32'h00400053;
        imem[10] = 32'h00102273;
        imem[11] = 32'hfff18193;
        imem[12] = 32'hfe0194e3;
        imem[13] = 32'h00100073;
        run_bench("B6_fpu_fdiv_fcsr", 10000);
        check_eq32("B6 fflags NX", last_rd_val[4], 32'h01);

        // =====================================================================
        // B7: Mispredict + Speculative Store/Ebreak/Trap Recovery Torture Test
        // =====================================================================
        clear_memories();
        mem_latency = 8;
        dmem_bytes[0] = 8'h05; // dmem[0x10010000] = 5
        // 0x00: lui  x10, 0x10010
        // 0x04: lw   x1, 0(x10)       (miss -> x1 = 5)
        // 0x08: addi x2, x0, 5
        // 0x0c: beq  x1, x2, +24      (forward branch, predicted NT! MISPREDICTED -> target 0x24!)
        // Wrong-path speculative instructions (MUST ALL BE FLUSHED!):
        // 0x10: addi x3, x0, 99
        // 0x14: sw   x3, 4(x10)       (speculative store -> MUST NOT write memory!)
        // 0x18: lw   x4, 1(x10)       (speculative misaligned trap!)
        // 0x1c: 32'h00000000          (speculative illegal!)
        // 0x20: ebreak                (speculative ebreak!)
        // Correct target (0x24):
        // 0x24: lw   x5, 4(x10)       (must read 0, NOT 99!)
        // 0x28: addi x6, x1, 10       (5 + 10 = 15)
        // 0x2c: add  x6, x6, x5       (15)
        // 0x30: ebreak
        imem[ 0] = 32'h10010537;
        imem[ 1] = 32'h00052083;
        imem[ 2] = 32'h00500113;
        imem[ 3] = 32'h00208c63;
        imem[ 4] = 32'h06300193;
        imem[ 5] = 32'h00352223;
        imem[ 6] = 32'h00152203;
        imem[ 7] = 32'h00000000;
        imem[ 8] = 32'h00100073;
        imem[ 9] = 32'h00452283;
        imem[10] = 32'h00a08313;
        imem[11] = 32'h00530333;
        imem[12] = 32'h00100073;
        run_bench("B7_misp_torture", 5000);
        check_eq32("B7 spec store squashed (x5=0)", last_rd_val[5], 32'd0);
        check_eq32("B7 post-misp x6=15", last_rd_val[6], 32'd15);

        if (trace_fd != 0)
            $fclose(trace_fd);

        $display("Phase 8 Benchmark Suite: %0d checks passed, %0d failed", passed, failed);
        if (failed > 0) $fatal(1, "Benchmarks failed!");
        $finish;
    end

endmodule

`default_nettype wire
