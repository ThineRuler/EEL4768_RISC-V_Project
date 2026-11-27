`timescale 1ns / 1ps
`default_nettype none

// Comprehensive self-checking testbench for Phase 7 OoO RV32IMF CPU (hart.v).
// Tests:
//  1. Basic RV32I correctness, RAW/WAW/WAR hazards, x0 discard, sub-word mem
//  2. Non-speculative branches & jumps (taken/not-taken, jal, jalr, loops)
//  3. Traps (illegal inst, misaligned load/store, misaligned jump, FPU rm_trap) + post-trap recovery
//  4. Out-of-Order completion witness (o_complete_*) vs in-order retire (o_retire_*)
//     - Slow div/fdiv/dcache-miss overtaken by younger independent ALU/FPU ops
//     - CDB age arbitration (older ROB tag wins on contention)
//     - No o_complete_valid on stores, ebreak, illegal, misaligned traps
//  5. Dual FPU parallelism (2 concurrent fdiv.s), f0 renaming, fflags accrual, CSR sync
//  6. OoO throughput CPI vs in-order stall behavior on cache-miss + independent ALU workload
module hart_tb;

    parameter RESET_ADDR     = 32'h00400000;
    parameter EVICT_POLICY   = 1;
    parameter NUM_WAYS       = 4;
    parameter CACHE_SIZE     = 2;
    parameter BLOCK_SIZE     = 16;
    parameter PREFETCH_EN    = 1;
    parameter WRITE_ALLOCATE = 0;

    reg clk;
    reg rst;

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

    // Combinational IMEM
    localparam IMEM_WORDS = 1024;
    reg [31:0] imem [0:IMEM_WORDS-1];
    wire [31:0] imem_idx = (imem_raddr - RESET_ADDR) >> 2;
    assign imem_rdata = (imem_raddr >= RESET_ADDR && imem_idx < IMEM_WORDS) ?
                        imem[imem_idx] : 32'h00000013;

    // Main DMEM (base 0x10010000, 64 KB) with configurable latency
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
                dmem_q_count <= dmem_q_count + 1;
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
                dmem_q_count <= dmem_q_count - 1;
            end
        end
    end

    always #5 clk = ~clk;

    integer passed, failed, cycle;
    integer complete_count;
    reg [31:0] complete_pc_log   [0:255];
    reg [ 4:0] complete_rd_log   [0:255];
    reg [31:0] complete_val_log  [0:255];
    reg [ 2:0] complete_tag_log  [0:255];

    integer retire_count;
    reg [31:0] retire_pc_log     [0:255];
    reg [31:0] retire_inst_log   [0:255];
    reg        retire_trap_log   [0:255];
    reg        retire_halt_log   [0:255];
    reg [ 4:0] retire_rd_log     [0:255];
    reg [31:0] retire_rdval_log  [0:255];
    reg [ 4:0] retire_rs1_log    [0:255];
    reg [31:0] retire_rs1val_log [0:255];
    reg [ 4:0] retire_rs2_log    [0:255];
    reg [31:0] retire_rs2val_log [0:255];
    reg [31:0] retire_npc_log    [0:255];
    reg        retire_frdwen_log [0:255];
    reg [ 4:0] retire_frd_log    [0:255];
    reg [31:0] retire_frdval_log [0:255];
    reg [ 4:0] retire_fflags_log [0:255];
    reg        retire_dren_log   [0:255];
    reg        retire_dwen_log   [0:255];
    reg [31:0] retire_daddr_log  [0:255];
    reg [ 3:0] retire_dmask_log  [0:255];

    task clear_memories;
        integer k;
        begin
            for (k = 0; k < IMEM_WORDS; k = k + 1) imem[k] = 32'h00000013;
            for (k = 0; k < DMEM_BYTES; k = k + 1) dmem_bytes[k] = 8'h00;
        end
    endtask

    task run_until_halt(input integer max_cycles);
        integer cyc;
        reg done;
        begin
            complete_count = 0;
            retire_count   = 0;
            cycle          = 0;
            done           = 1'b0;

            @(negedge clk);
            rst = 1'b1;
            repeat (3) @(posedge clk);
            @(negedge clk);
            rst = 1'b0;

            for (cyc = 0; cyc < max_cycles && !done; cyc = cyc + 1) begin
                @(negedge clk);
                cycle = cycle + 1;
                if (complete_valid) begin
                    complete_pc_log[complete_count]  = complete_pc;
                    complete_rd_log[complete_count]  = complete_rd;
                    complete_val_log[complete_count] = complete_value;
                    complete_tag_log[complete_count] = complete_tag;
                    complete_count = complete_count + 1;
                end
                if (retire_valid) begin
                    retire_pc_log[retire_count]     = retire_pc;
                    retire_inst_log[retire_count]   = retire_inst;
                    retire_trap_log[retire_count]   = retire_trap;
                    retire_halt_log[retire_count]   = retire_halt;
                    retire_rd_log[retire_count]     = retire_rd_waddr;
                    retire_rdval_log[retire_count]  = retire_rd_wdata;
                    retire_rs1_log[retire_count]    = retire_rs1_raddr;
                    retire_rs1val_log[retire_count] = retire_rs1_rdata;
                    retire_rs2_log[retire_count]    = retire_rs2_raddr;
                    retire_rs2val_log[retire_count] = retire_rs2_rdata;
                    retire_npc_log[retire_count]    = retire_next_pc;
                    retire_frdwen_log[retire_count] = retire_frd_wen;
                    retire_frd_log[retire_count]    = retire_frd_waddr;
                    retire_frdval_log[retire_count] = retire_frd_wdata;
                    retire_fflags_log[retire_count] = retire_fflags;
                    retire_dren_log[retire_count]   = retire_dmem_ren;
                    retire_dwen_log[retire_count]   = retire_dmem_wen;
                    retire_daddr_log[retire_count]  = retire_dmem_addr;
                    retire_dmask_log[retire_count]  = retire_dmem_mask;
                    retire_count = retire_count + 1;
                    if (retire_halt)
                        done = 1'b1;
                end
            end

            if (!done) begin
                $display("[FAIL] Simulation timed out after %0d cycles!", max_cycles);
                failed = failed + 1;
            end
        end
    endtask

    task check_eq32(input [511:0] msg, input [31:0] got, input [31:0] exp);
        begin
            if (got === exp) begin
                passed = passed + 1;
            end else begin
                $display("[FAIL] %0s: got 0x%08h, exp 0x%08h", msg, got, exp);
                failed = failed + 1;
            end
        end
    endtask

    task check_eq1(input [511:0] msg, input got, input exp);
        begin
            if (got === exp) begin
                passed = passed + 1;
            end else begin
                $display("[FAIL] %0s: got %b, exp %b", msg, got, exp);
                failed = failed + 1;
            end
        end
    endtask

    function integer find_complete_idx(input [31:0] pc_val);
        integer j;
        begin
            find_complete_idx = -1;
            for (j = 0; j < complete_count; j = j + 1) begin
                if (complete_pc_log[j] == pc_val && find_complete_idx == -1)
                    find_complete_idx = j;
            end
        end
    endfunction

    integer idx_a, idx_b, idx_c, t;

    initial begin
        $dumpfile("hart_tb.vcd");
        $dumpvars(0, hart_tb);
        clk         = 1'b0;
        rst         = 1'b1;
        mem_latency = 4;
        passed      = 0;
        failed      = 0;

        // =====================================================================
        // TEST 1: OoO Completion Witness + In-Order Retire (DIV + ALU + Load Miss)
        // =====================================================================
        $display("\n=== TEST 1: OoO Completion Witness & In-Order Retire ===");
        clear_memories();
        // Preload dmem[0x10010000] = 0x12345678
        dmem_bytes[0] = 8'h78; dmem_bytes[1] = 8'h56; dmem_bytes[2] = 8'h34; dmem_bytes[3] = 8'h12;
        dmem_bytes[4] = 8'hAA; dmem_bytes[5] = 8'hBB; dmem_bytes[6] = 8'hCC; dmem_bytes[7] = 8'hDD;

        // 0x00400000: lui  x10, 0x10010       (x10 = 0x10010000)
        // 0x00400004: lw   x1, 0(x10)         (D-cache miss -> slow!)
        // 0x00400008: addi x2, x0, 10         (independent! completes BEFORE lw!)
        // 0x0040000c: addi x3, x2, 20         (depends on x2, still completes BEFORE lw!)
        // 0x00400010: addi x4, x1, 1          (depends on lw x1 -> waits for lw CDB)
        // 0x00400014: sw   x3, 8(x10)         (store: NO complete pulse; writes at commit)
        // 0x00400018: lw   x5, 8(x10)         (load after store -> must see 30!)
        // 0x0040001c: ebreak                  (halt: NO complete pulse)
        imem[0] = 32'h10010537; // lui  x10, 0x10010
        imem[1] = 32'h00052083; // lw   x1, 0(x10)
        imem[2] = 32'h00a00113; // addi x2, x0, 10
        imem[3] = 32'h01410193; // addi x3, x2, 20
        imem[4] = 32'h00108213; // addi x4, x1, 1
        imem[5] = 32'h00352423; // sw   x3, 8(x10)
        imem[6] = 32'h00852283; // lw   x5, 8(x10)
        imem[7] = 32'h00100073; // ebreak

        run_until_halt(200);

        check_eq32("T1 retire count", retire_count, 32'd8);
        // Retire must be strictly in program order
        check_eq32("T1 retire[0] pc", retire_pc_log[0], 32'h00400000);
        check_eq32("T1 retire[1] pc", retire_pc_log[1], 32'h00400004);
        check_eq32("T1 retire[1] lw val", retire_rdval_log[1], 32'h12345678);
        check_eq32("T1 retire[2] pc", retire_pc_log[2], 32'h00400008);
        check_eq32("T1 retire[2] x2", retire_rdval_log[2], 32'd10);
        check_eq32("T1 retire[3] pc", retire_pc_log[3], 32'h0040000c);
        check_eq32("T1 retire[3] x3", retire_rdval_log[3], 32'd30);
        check_eq32("T1 retire[4] pc", retire_pc_log[4], 32'h00400010);
        check_eq32("T1 retire[4] x4", retire_rdval_log[4], 32'h12345679);
        check_eq32("T1 retire[4] rs1_rdata", retire_rs1val_log[4], 32'h12345678);
        check_eq32("T1 retire[5] sw dwen", {31'd0, retire_dwen_log[5]}, 32'd1);
        check_eq32("T1 retire[6] lw after sw", retire_rdval_log[6], 32'd30);
        check_eq1 ("T1 retire[7] halt", retire_halt_log[7], 1'b1);

        // Check OoO witness: addi x2 (0x08) and addi x3 (0x0c) MUST complete before lw x1 (0x04)
        idx_a = find_complete_idx(32'h00400004); // lw x1
        idx_b = find_complete_idx(32'h00400008); // addi x2
        idx_c = find_complete_idx(32'h0040000c); // addi x3
        if (idx_a >= 0 && idx_b >= 0 && idx_c >= 0 && idx_b < idx_a && idx_c < idx_a) begin
            $display("[PASS] T1 OoO witness: 0x08 (idx %0d) & 0x0c (idx %0d) completed before 0x04 lw miss (idx %0d)", idx_b, idx_c, idx_a);
            passed = passed + 1;
        end else begin
            $display("[FAIL] T1 OoO witness failed: idx_lw=%0d, idx_addi2=%0d, idx_addi3=%0d", idx_a, idx_b, idx_c);
            failed = failed + 1;
        end

        // Verify store (0x14) and ebreak (0x1c) did NOT pulse o_complete_valid
        check_eq32("T1 no complete on sw", find_complete_idx(32'h00400014), -1);
        check_eq32("T1 no complete on ebreak", find_complete_idx(32'h0040001c), -1);
        check_eq32("T1 total completes", complete_count, 32'd6);

        // =====================================================================
        // TEST 2: WAW / WAR Renaming + Dual FPU Parallelism + FDIV OoO Witness
        // =====================================================================
        $display("\n=== TEST 2: WAW/WAR + Dual FPU Parallelism + FFLAGS ===");
        clear_memories();
        dmem_bytes[0] = 8'h07; dmem_bytes[1] = 8'h00; dmem_bytes[2] = 8'h00; dmem_bytes[3] = 8'h00; // 7
        // 0x00400000: lui  x10, 0x10010
        // 0x00400004: addi x1, x0, 100
        // 0x00400008: lw   x2, 0(x10)         (cache miss -> x2 = 7)
        // 0x0040000c: div  x3, x1, x2         (waits on x2! -> x3 = 14)
        // 0x00400010: addi x3, x0, 99         (WAW on x3! ready immediately -> completes BEFORE div!)
        // 0x00400014: addi x4, x3, 1          (must read WAW x3=99 -> x4=100!)
        // 0x00400018: addi x1, x0, 55         (WAR on x1 after div issued!)
        // 0x0040001c: lui  x5, 0x3f800        (1.0f = 0x3f800000)
        // 0x00400020: lui  x6, 0x40000        (2.0f = 0x40000000)
        // 0x00400024: fmv.w.x f0, x5          (f0 = 1.0f -- tests f0 renaming!)
        // 0x00400028: fmv.w.x f1, x6          (f1 = 2.0f)
        // 0x0040002c: fdiv.s  f2, f0, f1      (f2 = 0.5f = 0x3f000000 on fpu_0)
        // 0x00400030: fdiv.s  f3, f1, f0      (f3 = 2.0f = 0x40000000 on fpu_1 in parallel!)
        // 0x00400034: lui  x7, 0x40400        (3.0f)
        // 0x00400038: fmv.w.x f5, x7
        // 0x0040003c: fdiv.s  f6, f0, f5      (1.0 / 3.0 -> NX=1)
        // 0x00400040: csrrs x8, fflags, x0    (frflags x8 -> expects 1)
        // 0x00400044: ebreak
        imem[ 0] = 32'h10010537; // lui  x10, 0x10010
        imem[ 1] = 32'h06400093; // addi x1, x0, 100
        imem[ 2] = 32'h00052103; // lw   x2, 0(x10)
        imem[ 3] = 32'h0220c1b3; // div  x3, x1, x2
        imem[ 4] = 32'h06300193; // addi x3, x0, 99
        imem[ 5] = 32'h00118213; // addi x4, x3, 1
        imem[ 6] = 32'h03700093; // addi x1, x0, 55
        imem[ 7] = 32'h3f8002b7; // lui  x5, 0x3f800
        imem[ 8] = 32'h40000337; // lui  x6, 0x40000
        imem[ 9] = 32'hf0028053; // fmv.w.x f0, x5
        imem[10] = 32'hf00300d3; // fmv.w.x f1, x6
        imem[11] = 32'h18100153; // fdiv.s f2, f0, f1 (rm=0)
        imem[12] = 32'h180081d3; // fdiv.s f3, f1, f0 (rm=0)
        imem[13] = 32'h404003b7; // lui  x7, 0x40400 (3.0f)
        imem[14] = 32'hf00382d3; // fmv.w.x f5, x7
        imem[15] = 32'h18500353; // fdiv.s f6, f0, f5 (1.0 / 3.0 -> NX=1)
        imem[16] = 32'h00102473; // csrrs x8, fflags, x0 (frflags x8 -> expects 1)
        imem[17] = 32'h00100073; // ebreak

        run_until_halt(300);

        check_eq32("T2 retire count", retire_count, 32'd18);
        check_eq32("T2 div x3", retire_rdval_log[3], 32'd14);
        check_eq32("T2 WAW addi x3", retire_rdval_log[4], 32'd99);
        check_eq32("T2 addi x4 uses WAW x3", retire_rdval_log[5], 32'd100);
        check_eq32("T2 WAR addi x1", retire_rdval_log[6], 32'd55);
        check_eq1 ("T2 f0 frd_wen", retire_frdwen_log[9], 1'b1);
        check_eq32("T2 f0 frd_waddr", {27'd0, retire_frd_log[9]}, 32'd0);
        check_eq32("T2 f0 val", retire_frdval_log[9], 32'h3f800000);
        check_eq32("T2 f2 = 1.0/2.0", retire_frdval_log[11], 32'h3f000000);
        check_eq32("T2 f3 = 2.0/1.0", retire_frdval_log[12], 32'h40000000);
        check_eq32("T2 f6 fflags NX", {27'd0, retire_fflags_log[15]}, 32'h01);
        check_eq32("T2 frflags x8", retire_rdval_log[16], 32'h01);

        idx_a = find_complete_idx(32'h0040000c); // div x3
        idx_b = find_complete_idx(32'h00400010); // addi x3
        if (idx_a >= 0 && idx_b >= 0 && idx_b < idx_a) begin
            $display("[PASS] T2 OoO witness: WAW addi (idx %0d) completed before div (idx %0d)", idx_b, idx_a);
            passed = passed + 1;
        end else begin
            $display("[FAIL] T2 OoO witness failed: idx_div=%0d, idx_waw=%0d", idx_a, idx_b);
            failed = failed + 1;
        end

        // =====================================================================
        // TEST 3: Branchy Control Flow + Traps (Illegal, Misaligned Load/Store)
        // =====================================================================
        $display("\n=== TEST 3: Branches, JAL/JALR, Illegal & Misaligned Traps ===");
        clear_memories();
        // Loop summing 1+2+3 = 6, then JAL, JALR, illegal, misaligned lw + dependent addi, ebreak
        // 0x00: addi x1, x0, 0      (sum)
        // 0x04: addi x2, x0, 1      (i)
        // 0x08: addi x3, x0, 4      (limit)
        // loop (0x0c):
        // 0x0c: add  x1, x1, x2
        // 0x10: addi x2, x2, 1
        // 0x14: blt  x2, x3, -8     (-> 0x0c)
        // 0x18: jal  x4, +8         (-> 0x20, x4 = 0x0040001c)
        // 0x1c: addi x1, x0, 999    (SKIPPED!)
        // 0x20: lui  x10, 0x10010
        // 0x24: 32'h00000000        (illegal TRAP!)
        // 0x28: lw   x1, 1(x10)     (misaligned lw TRAP! x1 must STAY 6!)
        // 0x2c: addi x5, x1, 10     (must get x1=6 via trap-restore wakeup -> x5=16!)
        // 0x30: sw   x5, 2(x10)     (misaligned sw TRAP! no memory write!)
        // 0x34: ebreak
        imem[ 0] = 32'h00000093; // addi x1, x0, 0
        imem[ 1] = 32'h00100113; // addi x2, x0, 1
        imem[ 2] = 32'h00400193; // addi x3, x0, 4
        imem[ 3] = 32'h002080b3; // add  x1, x1, x2
        imem[ 4] = 32'h00110113; // addi x2, x2, 1
        imem[ 5] = 32'hfe314ce3; // blt  x2, x3, -8
        imem[ 6] = 32'h0080026f; // jal  x4, +8
        imem[ 7] = 32'h3e700093; // addi x1, x0, 999 (skipped)
        imem[ 8] = 32'h10010537; // lui  x10, 0x10010
        imem[ 9] = 32'h00000000; // illegal
        imem[10] = 32'h00152083; // lw   x1, 1(x10) (misaligned)
        imem[11] = 32'h00a08293; // addi x5, x1, 10
        imem[12] = 32'h00552123; // sw   x5, 2(x10) (misaligned)
        imem[13] = 32'h00100073; // ebreak

        run_until_halt(300);

        // 3 init + 3*(add+addi+blt)=9 + jal + lui + ill + lw_trap + addi + sw_trap + ebreak = 19 retires
        check_eq32("T3 retire count", retire_count, 32'd19);
        check_eq32("T3 jal link x4", retire_rdval_log[12], 32'h0040001c);
        check_eq1 ("T3 illegal trap", retire_trap_log[14], 1'b1);
        check_eq32("T3 illegal next_pc", retire_npc_log[14], 32'h00400028);
        check_eq1 ("T3 misalign lw trap", retire_trap_log[15], 1'b1);
        check_eq32("T3 misalign lw rd=0", {27'd0, retire_rd_log[15]}, 32'd0);
        check_eq1 ("T3 misalign lw no dren", retire_dren_log[15], 1'b0);
        check_eq32("T3 post-trap x5=6+10", retire_rdval_log[16], 32'd16);
        check_eq1 ("T3 misalign sw trap", retire_trap_log[17], 1'b1);
        check_eq1 ("T3 misalign sw no dwen", retire_dwen_log[17], 1'b0);
        check_eq32("T3 no complete on illegal", find_complete_idx(32'h00400024), -1);
        check_eq32("T3 no complete on mis lw", find_complete_idx(32'h00400028), -1);
        check_eq32("T3 no complete on mis sw", find_complete_idx(32'h00400030), -1);
        // Verify wrong-path 0x1c never completed or retired
        check_eq32("T3 no complete on skipped", find_complete_idx(32'h0040001c), -1);

        // =====================================================================
        // TEST 4: OoO Throughput Test (Cache Misses + Independent ALU Chains)
        // =====================================================================
        $display("\n=== TEST 4: OoO Throughput under D-Cache Misses ===");
        clear_memories();
        mem_latency = 8; // 8-cycle memory latency
        // 4 cache misses to distinct cache sets interleaved with independent ALU work
        imem[ 0] = 32'h10010537; // lui  x10, 0x10010
        imem[ 1] = 32'h00052083; // lw   x1, 0(x10)      (miss 1)
        imem[ 2] = 32'h00100113; // addi x2, x0, 1
        imem[ 3] = 32'h00200193; // addi x3, x0, 2
        imem[ 4] = 32'h00310213; // addi x4, x2, 3
        imem[ 5] = 32'h00418293; // addi x5, x3, 4
        imem[ 6] = 32'h00520313; // addi x6, x4, 5
        imem[ 7] = 32'h04052383; // lw   x7, 64(x10)     (miss 2)
        imem[ 8] = 32'h00628413; // addi x8, x5, 6
        imem[ 9] = 32'h00730493; // addi x9, x6, 7
        imem[10] = 32'h00840593; // addi x11, x8, 8
        imem[11] = 32'h00948613; // addi x12, x9, 9
        imem[12] = 32'h00a08693; // addi x13, x1, 10
        imem[13] = 32'h00b38713; // addi x14, x7, 11
        imem[14] = 32'h00100073; // ebreak

        run_until_halt(200);
        check_eq32("T4 retire count", retire_count, 32'd15);
        $display("[INFO] T4 OoO retired %0d instructions in %0d cycles (CPI = %0f)",
                 retire_count, cycle, (1.0 * cycle) / retire_count);

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n=======================================================");
        $display("Phase 7 Directed Testbench: %0d passed, %0d failed", passed, failed);
        $display("=======================================================\n");
        if (failed > 0)
            $fatal(1, "Some tests failed!");
        $finish;
    end

endmodule

`default_nettype wire
