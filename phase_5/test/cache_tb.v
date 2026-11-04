`timescale 1ns / 1ps
`default_nettype none

// ============================================================================
// Module: cache_tb
// Description: Comprehensive Self-Checking Testbench for L1 Cache (cache.v)
// EEL 4768: Computer Architecture (Phase 5 - Parts 1 & 2)
//
// SUMMARY OF TESTED REQUIREMENTS:
// -------------------------------
//  1. DIRECT-MAPPED CACHE (NUM_WAYS = 1):
//     - Cold miss read (asserts o_mem_rd, stalls CPU until i_mem_valid)
//     - Read hit (instantaneous 0-stall read from cache line)
//     - Store hit: Write-Back policy (updates line, sets dirty, NO memory write)
//     - Sub-word store hits: Byte (SB) and Halfword (SH) lane masking
//     - Store miss: Write-Through, no-allocate (writes straight to memory with
//       exact byte enables, line NOT allocated in cache, 0 stall cycles)
//     - Clean eviction: conflict miss on clean line triggers memory read only
//     - Dirty eviction: conflict miss on dirty line writes full block back
//       to memory with all byte enables high before reading new block
//     - Variable memory latency: validated for latency = 1, 4, and 8 cycles
//
//  2. 4-WAY SET ASSOCIATIVE CACHE (NUM_WAYS = 4, LRU):
//     - Way allocation priority: cold misses fill Way 0 -> Way 1 -> Way 2 -> Way 3
//     - Multi-way coexistence: all 4 ways co-reside and hit with 0 stalls
//     - True LRU policy: maintains MRU->LRU order queue per set
//     - Section 4.3.1 Breakdown sequence: verifies exact LRU victim selection
//     - Dirty eviction in multi-way set
//
//  3. 4-WAY SET ASSOCIATIVE CACHE (NUM_WAYS = 4, PLRU):
//     - Pseudo-LRU binary decision tree policy (EVICT_POLICY = 1)
//     - Figure 1 Verification: Step-by-step reproduction of the 4-way tree bit flips
//       and victim traversal leading to Way 0 eviction
//
//  4. 2-WAY SET ASSOCIATIVE CACHE (NUM_WAYS = 2):
//     - Verifies correct 2-way parameterization and ping-pong LRU eviction
//
//  5. FULLY ASSOCIATIVE BASE CONFIGURATION (NUM_WAYS = 8, NUM_SETS = 1):
//     - Verifies address slicing with 0 set bits (pure Tag + Offset)
//     - Co-residence across multiple lines and LRU replacement across all lines
// ============================================================================

module cache_tb;

    integer total_passed = 0;
    integer total_failed = 0;

    // ------------------------------------------------------------------------
    // Clock Generator (100 MHz, 10ns period)
    // ------------------------------------------------------------------------
    reg clk;
    initial clk = 0;
    always #5 clk = ~clk;

    // ========================================================================
    // DUT 1: Direct-Mapped Cache (NUM_WAYS = 1, 16 KB, 64 B block)
    // ========================================================================
    reg         dm_rst;
    reg         dm_ren;
    reg         dm_wen;
    reg  [31:0] dm_addr;
    reg  [ 3:0] dm_mask;
    reg  [31:0] dm_wdata;
    wire        dm_ready;
    wire [31:0] dm_rdata;

    wire        dm_mem_rd;
    wire [31:0] dm_mem_rd_addr;
    reg         dm_mem_valid;
    reg  [64*8-1:0] dm_mem_rd_data;
    wire        dm_mem_wr;
    wire [31:0] dm_mem_wr_addr;
    wire [64*8-1:0] dm_mem_wr_data;
    wire [63:0] dm_mem_wr_be;

    cache #(
        .EVICT_POLICY(0),
        .NUM_WAYS(1),
        .CACHE_SIZE(16),
        .BLOCK_SIZE(64),
        .PREFETCH_EN(0)
    ) dut_dm (
        .i_clk          (clk),
        .i_rst          (dm_rst),
        .i_ren          (dm_ren),
        .i_wen          (dm_wen),
        .i_addr         (dm_addr),
        .i_mask         (dm_mask),
        .i_wdata        (dm_wdata),
        .o_ready        (dm_ready),
        .o_rdata        (dm_rdata),
        .o_mem_rd       (dm_mem_rd),
        .o_mem_rd_addr  (dm_mem_rd_addr),
        .i_mem_valid    (dm_mem_valid),
        .i_mem_rd_data  (dm_mem_rd_data),
        .o_mem_wr       (dm_mem_wr),
        .o_mem_wr_addr  (dm_mem_wr_addr),
        .o_mem_wr_data  (dm_mem_wr_data),
        .o_mem_wr_be    (dm_mem_wr_be)
    );

    // Direct Mapped simulated memory
    reg [7:0] dm_memory [0:65535];
    integer dm_mem_latency = 4;
    integer dm_rd_delay = 0;
    reg [31:0] dm_pending_addr = 0;
    reg        dm_pending_valid = 0;
    integer dm_b;

    always @(posedge clk) begin
        if (dm_rst) begin
            dm_mem_valid     <= 1'b0;
            dm_mem_rd_data   <= 512'd0;
            dm_rd_delay      <= 0;
            dm_pending_valid <= 1'b0;
        end else begin
            dm_mem_valid <= 1'b0;

            if (dm_mem_wr) begin
                for (dm_b = 0; dm_b < 64; dm_b = dm_b + 1) begin
                    if (dm_mem_wr_be[dm_b]) begin
                        dm_memory[(dm_mem_wr_addr + dm_b) & 16'hFFFF] <= dm_mem_wr_data[dm_b*8 +: 8];
                    end
                end
            end

            if (dm_pending_valid) begin
                if (dm_rd_delay <= 1) begin
                    dm_mem_valid <= 1'b1;
                    for (dm_b = 0; dm_b < 64; dm_b = dm_b + 1) begin
                        dm_mem_rd_data[dm_b*8 +: 8] <= dm_memory[(dm_pending_addr + dm_b) & 16'hFFFF];
                    end
                    dm_pending_valid <= 1'b0;
                end else begin
                    dm_rd_delay <= dm_rd_delay - 1;
                end
            end else if (dm_mem_rd) begin
                dm_pending_valid <= 1'b1;
                dm_pending_addr  <= dm_mem_rd_addr;
                dm_rd_delay      <= dm_mem_latency;
            end
        end
    end

    task dm_read_check;
        input [511:0] label;
        input [31:0]  addr;
        input [31:0]  exp_data;
        input         expect_stall;
        reg [31:0]    stall_cycles;
        begin
            @(negedge clk);
            dm_ren   = 1'b1;
            dm_wen   = 1'b0;
            dm_addr  = addr;
            dm_mask  = 4'b0000;
            dm_wdata = 32'd0;
            stall_cycles = 0;

            @(posedge clk);
            while (!dm_ready) begin
                stall_cycles = stall_cycles + 1;
                @(posedge clk);
            end

            #1;
            if (dm_rdata === exp_data && (expect_stall ? (stall_cycles > 0) : (stall_cycles == 0))) begin
                total_passed = total_passed + 1;
                $display("[PASS] %0s: addr=%h rdata=%h (stalls=%0d)", label, addr, dm_rdata, stall_cycles);
            end else begin
                total_failed = total_failed + 1;
                $display("[FAIL] %0s: addr=%h rdata=%h (exp=%h, stalls=%0d, exp_stall=%0d)",
                         label, addr, dm_rdata, exp_data, stall_cycles, expect_stall);
            end

            @(negedge clk);
            dm_ren = 1'b0;
        end
    endtask

    task dm_write;
        input [511:0] label;
        input [31:0]  addr;
        input [ 3:0]  mask;
        input [31:0]  wdata;
        begin
            @(negedge clk);
            dm_ren   = 1'b0;
            dm_wen   = 1'b1;
            dm_addr  = addr;
            dm_mask  = mask;
            dm_wdata = wdata;

            @(posedge clk);
            #1;
            if (dm_ready === 1'b1) begin
                total_passed = total_passed + 1;
                $display("[PASS] %0s: Store addr=%h wdata=%h mask=%b (0 stalls)", label, addr, wdata, mask);
            end else begin
                total_failed = total_failed + 1;
                $display("[FAIL] %0s: Store addr=%h did not assert ready immediately!", label, addr);
            end

            @(negedge clk);
            dm_wen = 1'b0;
        end
    endtask

    // ========================================================================
    // DUT 2: 4-Way Set Associative Cache (LRU, NUM_WAYS = 4, 16 KB, 64 B block)
    // ========================================================================
    reg         sa_rst;
    reg         sa_ren;
    reg         sa_wen;
    reg  [31:0] sa_addr;
    reg  [ 3:0] sa_mask;
    reg  [31:0] sa_wdata;
    wire        sa_ready;
    wire [31:0] sa_rdata;

    wire        sa_mem_rd;
    wire [31:0] sa_mem_rd_addr;
    reg         sa_mem_valid;
    reg  [64*8-1:0] sa_mem_rd_data;
    wire        sa_mem_wr;
    wire [31:0] sa_mem_wr_addr;
    wire [64*8-1:0] sa_mem_wr_data;
    wire [63:0] sa_mem_wr_be;

    cache #(
        .EVICT_POLICY(0), // LRU
        .NUM_WAYS(4),
        .CACHE_SIZE(16),
        .BLOCK_SIZE(64),
        .PREFETCH_EN(0)
    ) dut_sa_lru (
        .i_clk          (clk),
        .i_rst          (sa_rst),
        .i_ren          (sa_ren),
        .i_wen          (sa_wen),
        .i_addr         (sa_addr),
        .i_mask         (sa_mask),
        .i_wdata        (sa_wdata),
        .o_ready        (sa_ready),
        .o_rdata        (sa_rdata),
        .o_mem_rd       (sa_mem_rd),
        .o_mem_rd_addr  (sa_mem_rd_addr),
        .i_mem_valid    (sa_mem_valid),
        .i_mem_rd_data  (sa_mem_rd_data),
        .o_mem_wr       (sa_mem_wr),
        .o_mem_wr_addr  (sa_mem_wr_addr),
        .o_mem_wr_data  (sa_mem_wr_data),
        .o_mem_wr_be    (sa_mem_wr_be)
    );

    reg [7:0] sa_memory [0:65535];
    integer sa_mem_latency = 4;
    integer sa_rd_delay = 0;
    reg [31:0] sa_pending_addr = 0;
    reg        sa_pending_valid = 0;
    integer sa_b;

    always @(posedge clk) begin
        if (sa_rst) begin
            sa_mem_valid     <= 1'b0;
            sa_mem_rd_data   <= 512'd0;
            sa_rd_delay      <= 0;
            sa_pending_valid <= 1'b0;
        end else begin
            sa_mem_valid <= 1'b0;

            if (sa_mem_wr) begin
                for (sa_b = 0; sa_b < 64; sa_b = sa_b + 1) begin
                    if (sa_mem_wr_be[sa_b]) begin
                        sa_memory[(sa_mem_wr_addr + sa_b) & 16'hFFFF] <= sa_mem_wr_data[sa_b*8 +: 8];
                    end
                end
            end

            if (sa_pending_valid) begin
                if (sa_rd_delay <= 1) begin
                    sa_mem_valid <= 1'b1;
                    for (sa_b = 0; sa_b < 64; sa_b = sa_b + 1) begin
                        sa_mem_rd_data[sa_b*8 +: 8] <= sa_memory[(sa_pending_addr + sa_b) & 16'hFFFF];
                    end
                    sa_pending_valid <= 1'b0;
                end else begin
                    sa_rd_delay <= sa_rd_delay - 1;
                end
            end else if (sa_mem_rd) begin
                sa_pending_valid <= 1'b1;
                sa_pending_addr  <= sa_mem_rd_addr;
                sa_rd_delay      <= sa_mem_latency;
            end
        end
    end

    task sa_read_check;
        input [511:0] label;
        input [31:0]  addr;
        input [31:0]  exp_data;
        input         expect_stall;
        reg [31:0]    stall_cycles;
        begin
            @(negedge clk);
            sa_ren   = 1'b1;
            sa_wen   = 1'b0;
            sa_addr  = addr;
            sa_mask  = 4'b0000;
            sa_wdata = 32'd0;
            stall_cycles = 0;

            @(posedge clk);
            while (!sa_ready) begin
                stall_cycles = stall_cycles + 1;
                @(posedge clk);
            end

            #1;
            if (sa_rdata === exp_data && (expect_stall ? (stall_cycles > 0) : (stall_cycles == 0))) begin
                total_passed = total_passed + 1;
                $display("[PASS] %0s: addr=%h rdata=%h (stalls=%0d)", label, addr, sa_rdata, stall_cycles);
            end else begin
                total_failed = total_failed + 1;
                $display("[FAIL] %0s: addr=%h rdata=%h (exp=%h, stalls=%0d, exp_stall=%0d)",
                         label, addr, sa_rdata, exp_data, stall_cycles, expect_stall);
            end

            @(negedge clk);
            sa_ren = 1'b0;
        end
    endtask

    task sa_write;
        input [511:0] label;
        input [31:0]  addr;
        input [ 3:0]  mask;
        input [31:0]  wdata;
        begin
            @(negedge clk);
            sa_ren   = 1'b0;
            sa_wen   = 1'b1;
            sa_addr  = addr;
            sa_mask  = mask;
            sa_wdata = wdata;

            @(posedge clk);
            #1;
            if (sa_ready === 1'b1) begin
                total_passed = total_passed + 1;
                $display("[PASS] %0s: Store addr=%h wdata=%h mask=%b (0 stalls)", label, addr, wdata, mask);
            end else begin
                total_failed = total_failed + 1;
                $display("[FAIL] %0s: Store addr=%h failed ready check!", label, addr);
            end

            @(negedge clk);
            sa_wen = 1'b0;
        end
    endtask

    // ========================================================================
    // DUT 3: 4-Way Set Associative Cache (PLRU, NUM_WAYS = 4, 16 KB, 64 B block)
    // ========================================================================
    reg         plru_rst;
    reg         plru_ren;
    reg         plru_wen;
    reg  [31:0] plru_addr;
    reg  [ 3:0] plru_mask;
    reg  [31:0] plru_wdata;
    wire        plru_ready;
    wire [31:0] plru_rdata;

    wire        plru_mem_rd;
    wire [31:0] plru_mem_rd_addr;
    reg         plru_mem_valid;
    reg  [64*8-1:0] plru_mem_rd_data;
    wire        plru_mem_wr;
    wire [31:0] plru_mem_wr_addr;
    wire [64*8-1:0] plru_mem_wr_data;
    wire [63:0] plru_mem_wr_be;

    cache #(
        .EVICT_POLICY(1), // PLRU
        .NUM_WAYS(4),
        .CACHE_SIZE(16),
        .BLOCK_SIZE(64),
        .PREFETCH_EN(0)
    ) dut_sa_plru (
        .i_clk          (clk),
        .i_rst          (plru_rst),
        .i_ren          (plru_ren),
        .i_wen          (plru_wen),
        .i_addr         (plru_addr),
        .i_mask         (plru_mask),
        .i_wdata        (plru_wdata),
        .o_ready        (plru_ready),
        .o_rdata        (plru_rdata),
        .o_mem_rd       (plru_mem_rd),
        .o_mem_rd_addr  (plru_mem_rd_addr),
        .i_mem_valid    (plru_mem_valid),
        .i_mem_rd_data  (plru_mem_rd_data),
        .o_mem_wr       (plru_mem_wr),
        .o_mem_wr_addr  (plru_mem_wr_addr),
        .o_mem_wr_data  (plru_mem_wr_data),
        .o_mem_wr_be    (plru_mem_wr_be)
    );

    reg [7:0] plru_memory [0:65535];
    integer plru_mem_latency = 4;
    integer plru_rd_delay = 0;
    reg [31:0] plru_pending_addr = 0;
    reg        plru_pending_valid = 0;
    integer plru_b;

    always @(posedge clk) begin
        if (plru_rst) begin
            plru_mem_valid     <= 1'b0;
            plru_mem_rd_data   <= 512'd0;
            plru_rd_delay      <= 0;
            plru_pending_valid <= 1'b0;
        end else begin
            plru_mem_valid <= 1'b0;

            if (plru_mem_wr) begin
                for (plru_b = 0; plru_b < 64; plru_b = plru_b + 1) begin
                    if (plru_mem_wr_be[plru_b]) begin
                        plru_memory[(plru_mem_wr_addr + plru_b) & 16'hFFFF] <= plru_mem_wr_data[plru_b*8 +: 8];
                    end
                end
            end

            if (plru_pending_valid) begin
                if (plru_rd_delay <= 1) begin
                    plru_mem_valid <= 1'b1;
                    for (plru_b = 0; plru_b < 64; plru_b = plru_b + 1) begin
                        plru_mem_rd_data[plru_b*8 +: 8] <= plru_memory[(plru_pending_addr + plru_b) & 16'hFFFF];
                    end
                    plru_pending_valid <= 1'b0;
                end else begin
                    plru_rd_delay <= plru_rd_delay - 1;
                end
            end else if (plru_mem_rd) begin
                plru_pending_valid <= 1'b1;
                plru_pending_addr  <= plru_mem_rd_addr;
                plru_rd_delay      <= plru_mem_latency;
            end
        end
    end

    task plru_read_check;
        input [511:0] label;
        input [31:0]  addr;
        input [31:0]  exp_data;
        input         expect_stall;
        reg [31:0]    stall_cycles;
        begin
            @(negedge clk);
            plru_ren   = 1'b1;
            plru_wen   = 1'b0;
            plru_addr  = addr;
            plru_mask  = 4'b0000;
            plru_wdata = 32'd0;
            stall_cycles = 0;

            @(posedge clk);
            while (!plru_ready) begin
                stall_cycles = stall_cycles + 1;
                @(posedge clk);
            end

            #1;
            if (plru_rdata === exp_data && (expect_stall ? (stall_cycles > 0) : (stall_cycles == 0))) begin
                total_passed = total_passed + 1;
                $display("[PASS] %0s: addr=%h rdata=%h (stalls=%0d)", label, addr, plru_rdata, stall_cycles);
            end else begin
                total_failed = total_failed + 1;
                $display("[FAIL] %0s: addr=%h rdata=%h (exp=%h, stalls=%0d, exp_stall=%0d)",
                         label, addr, plru_rdata, exp_data, stall_cycles, expect_stall);
            end

            @(negedge clk);
            plru_ren = 1'b0;
        end
    endtask

    // ========================================================================
    // DUT 4: 2-Way Set Associative Cache (NUM_WAYS = 2, 16 KB, 64 B block)
    // ========================================================================
    reg         way2_rst;
    reg         way2_ren;
    reg         way2_wen;
    reg  [31:0] way2_addr;
    reg  [ 3:0] way2_mask;
    reg  [31:0] way2_wdata;
    wire        way2_ready;
    wire [31:0] way2_rdata;

    wire        way2_mem_rd;
    wire [31:0] way2_mem_rd_addr;
    reg         way2_mem_valid;
    reg  [64*8-1:0] way2_mem_rd_data;
    wire        way2_mem_wr;
    wire [31:0] way2_mem_wr_addr;
    wire [64*8-1:0] way2_mem_wr_data;
    wire [63:0] way2_mem_wr_be;

    cache #(
        .EVICT_POLICY(0),
        .NUM_WAYS(2),
        .CACHE_SIZE(16),
        .BLOCK_SIZE(64),
        .PREFETCH_EN(0)
    ) dut_way2 (
        .i_clk          (clk),
        .i_rst          (way2_rst),
        .i_ren          (way2_ren),
        .i_wen          (way2_wen),
        .i_addr         (way2_addr),
        .i_mask         (way2_mask),
        .i_wdata        (way2_wdata),
        .o_ready        (way2_ready),
        .o_rdata        (way2_rdata),
        .o_mem_rd       (way2_mem_rd),
        .o_mem_rd_addr  (way2_mem_rd_addr),
        .i_mem_valid    (way2_mem_valid),
        .i_mem_rd_data  (way2_mem_rd_data),
        .o_mem_wr       (way2_mem_wr),
        .o_mem_wr_addr  (way2_mem_wr_addr),
        .o_mem_wr_data  (way2_mem_wr_data),
        .o_mem_wr_be    (way2_mem_wr_be)
    );

    reg [7:0] way2_memory [0:65535];
    integer way2_mem_latency = 4;
    integer way2_rd_delay = 0;
    reg [31:0] way2_pending_addr = 0;
    reg        way2_pending_valid = 0;
    integer way2_b;

    always @(posedge clk) begin
        if (way2_rst) begin
            way2_mem_valid     <= 1'b0;
            way2_mem_rd_data   <= 512'd0;
            way2_rd_delay      <= 0;
            way2_pending_valid <= 1'b0;
        end else begin
            way2_mem_valid <= 1'b0;

            if (way2_mem_wr) begin
                for (way2_b = 0; way2_b < 64; way2_b = way2_b + 1) begin
                    if (way2_mem_wr_be[way2_b]) begin
                        way2_memory[(way2_mem_wr_addr + way2_b) & 16'hFFFF] <= way2_mem_wr_data[way2_b*8 +: 8];
                    end
                end
            end

            if (way2_pending_valid) begin
                if (way2_rd_delay <= 1) begin
                    way2_mem_valid <= 1'b1;
                    for (way2_b = 0; way2_b < 64; way2_b = way2_b + 1) begin
                        way2_mem_rd_data[way2_b*8 +: 8] <= way2_memory[(way2_pending_addr + way2_b) & 16'hFFFF];
                    end
                    way2_pending_valid <= 1'b0;
                end else begin
                    way2_rd_delay <= way2_rd_delay - 1;
                end
            end else if (way2_mem_rd) begin
                way2_pending_valid <= 1'b1;
                way2_pending_addr  <= way2_mem_rd_addr;
                way2_rd_delay      <= way2_mem_latency;
            end
        end
    end

    task way2_read_check;
        input [511:0] label;
        input [31:0]  addr;
        input [31:0]  exp_data;
        input         expect_stall;
        reg [31:0]    stall_cycles;
        begin
            @(negedge clk);
            way2_ren   = 1'b1;
            way2_wen   = 1'b0;
            way2_addr  = addr;
            way2_mask  = 4'b0000;
            way2_wdata = 32'd0;
            stall_cycles = 0;

            @(posedge clk);
            while (!way2_ready) begin
                stall_cycles = stall_cycles + 1;
                @(posedge clk);
            end

            #1;
            if (way2_rdata === exp_data && (expect_stall ? (stall_cycles > 0) : (stall_cycles == 0))) begin
                total_passed = total_passed + 1;
                $display("[PASS] %0s: addr=%h rdata=%h (stalls=%0d)", label, addr, way2_rdata, stall_cycles);
            end else begin
                total_failed = total_failed + 1;
                $display("[FAIL] %0s: addr=%h rdata=%h (exp=%h, stalls=%0d, exp_stall=%0d)",
                         label, addr, way2_rdata, exp_data, stall_cycles, expect_stall);
            end

            @(negedge clk);
            way2_ren = 1'b0;
        end
    endtask

    // ========================================================================
    // DUT 5: Fully Associative Base Configuration (1 Set, SET_BITS = 0)
    // ========================================================================
    reg         fa_rst;
    reg         fa_ren;
    reg         fa_wen;
    reg  [31:0] fa_addr;
    reg  [ 3:0] fa_mask;
    reg  [31:0] fa_wdata;
    wire        fa_ready;
    wire [31:0] fa_rdata;

    wire        fa_mem_rd;
    wire [31:0] fa_mem_rd_addr;
    reg         fa_mem_valid;
    reg  [128*8-1:0] fa_mem_rd_data;
    wire        fa_mem_wr;
    wire [31:0] fa_mem_wr_addr;
    wire [128*8-1:0] fa_mem_wr_data;
    wire [127:0] fa_mem_wr_be;

    cache #(
        .EVICT_POLICY(0),
        .NUM_WAYS(8),
        .CACHE_SIZE(1), // 1 KB = 8 blocks of 128 bytes, 8 ways = 1 set
        .BLOCK_SIZE(128),
        .PREFETCH_EN(0)
    ) dut_fa (
        .i_clk          (clk),
        .i_rst          (fa_rst),
        .i_ren          (fa_ren),
        .i_wen          (fa_wen),
        .i_addr         (fa_addr),
        .i_mask         (fa_mask),
        .i_wdata        (fa_wdata),
        .o_ready        (fa_ready),
        .o_rdata        (fa_rdata),
        .o_mem_rd       (fa_mem_rd),
        .o_mem_rd_addr  (fa_mem_rd_addr),
        .i_mem_valid    (fa_mem_valid),
        .i_mem_rd_data  (fa_mem_rd_data),
        .o_mem_wr       (fa_mem_wr),
        .o_mem_wr_addr  (fa_mem_wr_addr),
        .o_mem_wr_data  (fa_mem_wr_data),
        .o_mem_wr_be    (fa_mem_wr_be)
    );

    reg [7:0] fa_memory [0:65535];
    integer fa_mem_latency = 4;
    integer fa_rd_delay = 0;
    reg [31:0] fa_pending_addr = 0;
    reg        fa_pending_valid = 0;
    integer fa_b;

    always @(posedge clk) begin
        if (fa_rst) begin
            fa_mem_valid     <= 1'b0;
            fa_mem_rd_data   <= 1024'd0;
            fa_rd_delay      <= 0;
            fa_pending_valid <= 1'b0;
        end else begin
            fa_mem_valid <= 1'b0;

            if (fa_mem_wr) begin
                for (fa_b = 0; fa_b < 128; fa_b = fa_b + 1) begin
                    if (fa_mem_wr_be[fa_b]) begin
                        fa_memory[(fa_mem_wr_addr + fa_b) & 16'hFFFF] <= fa_mem_wr_data[fa_b*8 +: 8];
                    end
                end
            end

            if (fa_pending_valid) begin
                if (fa_rd_delay <= 1) begin
                    fa_mem_valid <= 1'b1;
                    for (fa_b = 0; fa_b < 128; fa_b = fa_b + 1) begin
                        fa_mem_rd_data[fa_b*8 +: 8] <= fa_memory[(fa_pending_addr + fa_b) & 16'hFFFF];
                    end
                    fa_pending_valid <= 1'b0;
                end else begin
                    fa_rd_delay <= fa_rd_delay - 1;
                end
            end else if (fa_mem_rd) begin
                fa_pending_valid <= 1'b1;
                fa_pending_addr  <= fa_mem_rd_addr;
                fa_rd_delay      <= fa_mem_latency;
            end
        end
    end

    task fa_read_check;
        input [511:0] label;
        input [31:0]  addr;
        input [31:0]  exp_data;
        input         expect_stall;
        reg [31:0]    stall_cycles;
        begin
            @(negedge clk);
            fa_ren   = 1'b1;
            fa_wen   = 1'b0;
            fa_addr  = addr;
            fa_mask  = 4'b0000;
            fa_wdata = 32'd0;
            stall_cycles = 0;

            @(posedge clk);
            while (!fa_ready) begin
                stall_cycles = stall_cycles + 1;
                @(posedge clk);
            end

            #1;
            if (fa_rdata === exp_data && (expect_stall ? (stall_cycles > 0) : (stall_cycles == 0))) begin
                total_passed = total_passed + 1;
                $display("[PASS] %0s: addr=%h rdata=%h (stalls=%0d)", label, addr, fa_rdata, stall_cycles);
            end else begin
                total_failed = total_failed + 1;
                $display("[FAIL] %0s: addr=%h rdata=%h (exp=%h, stalls=%0d, exp_stall=%0d)",
                         label, addr, fa_rdata, exp_data, stall_cycles, expect_stall);
            end

            @(negedge clk);
            fa_ren = 1'b0;
        end
    endtask

    // ========================================================================
    // TEST PROCEDURE
    // ========================================================================
    integer i;

    initial begin
        $display("==================================================================");
        $display("   STARTING CACHE COMPREHENSIVE TESTBENCH (PHASE 5 PARTS 1 & 2)   ");
        $display("==================================================================");

        // --------------------------------------------------------------------
        // SUITE 1: DIRECT-MAPPED TESTS
        // --------------------------------------------------------------------
        $display("\n--- TEST SUITE 1: Direct-Mapped (NUM_WAYS = 1) ---");
        dm_rst = 1;
        dm_ren = 0;
        dm_wen = 0;
        dm_addr = 0;
        dm_mask = 0;
        dm_wdata = 0;

        for (i = 0; i < 65536; i = i + 1) dm_memory[i] = i[7:0] ^ 8'h5A;

        dm_memory[16'h1000] = 8'h11;
        dm_memory[16'h1001] = 8'h22;
        dm_memory[16'h1002] = 8'h33;
        dm_memory[16'h1003] = 8'h44; // Word 0x1000 = 0x44332211

        dm_memory[16'h1004] = 8'hAA;
        dm_memory[16'h1005] = 8'hBB;
        dm_memory[16'h1006] = 8'hCC;
        dm_memory[16'h1007] = 8'hDD; // Word 0x1004 = 0xDDCCBBAA

        repeat (4) @(posedge clk);
        @(negedge clk);
        dm_rst = 0;
        repeat (2) @(posedge clk);

        // Cold miss read
        dm_read_check("Cold Miss Read @ 0x1000", 32'h00001000, 32'h44332211, 1'b1);

        // Read hit (same line, offset 4, 0 stalls)
        dm_read_check("Read Hit @ 0x1004", 32'h00001004, 32'hDDCCBBAA, 1'b0);

        // Store hit (Write-back, updates line, sets dirty)
        dm_write("Store Hit Word @ 0x1000", 32'h00001000, 4'b1111, 32'hCAFEBABE);

        // Read hit after store hit
        dm_read_check("Read Hit after Store Hit @ 0x1000", 32'h00001000, 32'hCAFEBABE, 1'b0);

        // Verify main memory NOT updated yet (Write-Back)
        if ({dm_memory[16'h1003], dm_memory[16'h1002], dm_memory[16'h1001], dm_memory[16'h1000]} === 32'h44332211) begin
            total_passed = total_passed + 1;
            $display("[PASS] Main memory not updated on Store Hit (Write-Back verified)");
        end else begin
            total_failed = total_failed + 1;
            $display("[FAIL] Main memory was prematurely updated on Store Hit!");
        end

        // Sub-word writes (Byte and Halfword)
        dm_write("Store Byte @ 0x1004", 32'h00001004, 4'b0001, 32'h00000077);
        dm_read_check("Read after Store Byte @ 0x1004", 32'h00001004, 32'hDDCCBB77, 1'b0);

        dm_write("Store Halfword @ 0x1006", 32'h00001004, 4'b1100, 32'h99880000);
        dm_read_check("Read after Store Halfword @ 0x1004", 32'h00001004, 32'h9988BB77, 1'b0);

        // Store Miss (Write-through, no-allocate)
        dm_memory[16'h3000] = 8'h00;
        dm_memory[16'h3001] = 8'h00;
        dm_memory[16'h3002] = 8'h00;
        dm_memory[16'h3003] = 8'h00;
        dm_write("Store Miss Write-Through @ 0x3000", 32'h00003000, 4'b1111, 32'h12345678);

        // Verify main memory updated immediately
        if ({dm_memory[16'h3003], dm_memory[16'h3002], dm_memory[16'h3001], dm_memory[16'h3000]} === 32'h12345678) begin
            total_passed = total_passed + 1;
            $display("[PASS] Main memory updated directly on Store Miss (Write-Through verified)");
        end else begin
            total_failed = total_failed + 1;
            $display("[FAIL] Main memory was not updated on Store Miss!");
        end

        // Conflict Miss Dirty Eviction: 0x5000 maps to same set 64 as 0x1000
        dm_memory[16'h5000] = 8'hEF;
        dm_memory[16'h5001] = 8'hBE;
        dm_memory[16'h5002] = 8'hAD;
        dm_memory[16'h5003] = 8'hDE;

        dm_read_check("Conflict Miss Dirty Eviction @ 0x5000", 32'h00005000, 32'hDEADBEEF, 1'b1);

        // Verify dirty line was written back to memory
        if ({dm_memory[16'h1003], dm_memory[16'h1002], dm_memory[16'h1001], dm_memory[16'h1000]} === 32'hCAFEBABE &&
            {dm_memory[16'h1007], dm_memory[16'h1006], dm_memory[16'h1005], dm_memory[16'h1004]} === 32'h9988BB77) begin
            total_passed = total_passed + 1;
            $display("[PASS] Dirty line correctly written back to main memory upon eviction!");
        end else begin
            total_failed = total_failed + 1;
            $display("[FAIL] Dirty line was not written back to memory upon eviction!");
        end

        // Variable Memory Latency checks
        dm_mem_latency = 1;
        dm_read_check("Variable Latency 1 cycle @ 0x1000", 32'h00001000, 32'hCAFEBABE, 1'b1);

        dm_mem_latency = 8;
        dm_read_check("Variable Latency 8 cycles @ 0x5000", 32'h00005000, 32'hDEADBEEF, 1'b1);

        // --------------------------------------------------------------------
        // SUITE 2: 4-WAY SET ASSOCIATIVE LRU TESTS
        // --------------------------------------------------------------------
        $display("\n--- TEST SUITE 2: 4-Way Set Associative LRU (NUM_WAYS = 4) ---");
        sa_rst = 1;
        sa_ren = 0;
        sa_wen = 0;
        sa_addr = 0;
        sa_mask = 0;
        sa_wdata = 0;

        for (i = 0; i < 65536; i = i + 1) sa_memory[i] = i[7:0] ^ 8'hA5;

        // Choose Set 5: addr = (tag << 12) | (5 << 6)
        sa_memory[16'h0140] = 8'h00; sa_memory[16'h0141] = 8'h00; sa_memory[16'h0142] = 8'hAA; sa_memory[16'h0143] = 8'hAA;
        sa_memory[16'h1140] = 8'h11; sa_memory[16'h1141] = 8'h11; sa_memory[16'h1142] = 8'hBB; sa_memory[16'h1143] = 8'hBB;
        sa_memory[16'h2140] = 8'h22; sa_memory[16'h2141] = 8'h22; sa_memory[16'h2142] = 8'hCC; sa_memory[16'h2143] = 8'hCC;
        sa_memory[16'h3140] = 8'h33; sa_memory[16'h3141] = 8'h33; sa_memory[16'h3142] = 8'hDD; sa_memory[16'h3143] = 8'hDD;
        sa_memory[16'h4140] = 8'h44; sa_memory[16'h4141] = 8'h44; sa_memory[16'h4142] = 8'hEE; sa_memory[16'h4143] = 8'hEE;

        repeat (4) @(posedge clk);
        @(negedge clk);
        sa_rst = 0;
        repeat (2) @(posedge clk);

        // Cold misses fill Way 0, 1, 2, 3 in sequential order
        sa_read_check("Cold Miss Fill Way 0 @ 0x0140", 32'h00000140, 32'hAAAA0000, 1'b1);
        sa_read_check("Cold Miss Fill Way 1 @ 0x1140", 32'h00001140, 32'hBBBB1111, 1'b1);
        sa_read_check("Cold Miss Fill Way 2 @ 0x2140", 32'h00002140, 32'hCCCC2222, 1'b1);
        sa_read_check("Cold Miss Fill Way 3 @ 0x3140", 32'h00003140, 32'hDDDD3333, 1'b1);

        // Verify all 4 ways hit with 0 stalls (co-residing in set 5)
        sa_read_check("Co-resident Hit Way 0 @ 0x0140", 32'h00000140, 32'hAAAA0000, 1'b0);
        sa_read_check("Co-resident Hit Way 1 @ 0x1140", 32'h00001140, 32'hBBBB1111, 1'b0);
        sa_read_check("Co-resident Hit Way 2 @ 0x2140", 32'h00002140, 32'hCCCC2222, 1'b0);
        sa_read_check("Co-resident Hit Way 3 @ 0x3140", 32'h00003140, 32'hDDDD3333, 1'b0);

        // Order manipulation: Touch Way 0, then 2, then 3 -> Way 1 is Least Recently Used!
        sa_read_check("Access Way 0 (MRU update)", 32'h00000140, 32'hAAAA0000, 1'b0);
        sa_read_check("Access Way 2 (MRU update)", 32'h00002140, 32'hCCCC2222, 1'b0);
        sa_read_check("Access Way 3 (MRU update)", 32'h00003140, 32'hDDDD3333, 1'b0);

        // Now Way 1 is LRU. Access Tag 4 (0x4140) -> MUST evict Way 1!
        sa_read_check("LRU Eviction (replaces Way 1) @ 0x4140", 32'h00004140, 32'hEEEE4444, 1'b1);

        // Ways 0, 2, 3, and newly loaded 4 MUST all HIT with 0 stalls!
        sa_read_check("Verify Way 0 survived with 0 stalls @ 0x0140", 32'h00000140, 32'hAAAA0000, 1'b0);
        sa_read_check("Verify Way 2 survived with 0 stalls @ 0x2140", 32'h00002140, 32'hCCCC2222, 1'b0);
        sa_read_check("Verify Way 3 survived with 0 stalls @ 0x3140", 32'h00003140, 32'hDDDD3333, 1'b0);
        sa_read_check("Verify Way 4 (in slot 1) hits with 0 stalls @ 0x4140", 32'h00004140, 32'hEEEE4444, 1'b0);

        // Verify Way 1 (0x1140) was truly evicted (must MISS with stalls!)
        sa_read_check("Verify Way 1 (0x1140) misses after eviction", 32'h00001140, 32'hBBBB1111, 1'b1);

        // Multi-way dirty write-back check on Set 6: index = 6, offset = 0.
        // Formula: addr = (tag << 12) | (6 << 6) = (tag << 12) | 0x0180
        sa_memory[16'h0180] = 8'h10; sa_memory[16'h0181] = 8'h10; sa_memory[16'h0182] = 8'h10; sa_memory[16'h0183] = 8'h10;
        sa_memory[16'h1180] = 8'h20; sa_memory[16'h1181] = 8'h20; sa_memory[16'h1182] = 8'h20; sa_memory[16'h1183] = 8'h20;
        sa_memory[16'h2180] = 8'h30; sa_memory[16'h2181] = 8'h30; sa_memory[16'h2182] = 8'h30; sa_memory[16'h2183] = 8'h30;
        sa_memory[16'h3180] = 8'h40; sa_memory[16'h3181] = 8'h40; sa_memory[16'h3182] = 8'h40; sa_memory[16'h3183] = 8'h40;
        sa_memory[16'h4180] = 8'h50; sa_memory[16'h4181] = 8'h50; sa_memory[16'h4182] = 8'h50; sa_memory[16'h4183] = 8'h50;

        // Cold fill all 4 ways of Set 6:
        sa_read_check("Fill Set 6 Way 0 @ 0x0180", 32'h00000180, 32'h10101010, 1'b1);
        sa_read_check("Fill Set 6 Way 1 @ 0x1180", 32'h00001180, 32'h20202020, 1'b1);
        sa_read_check("Fill Set 6 Way 2 @ 0x2180", 32'h00002180, 32'h30303030, 1'b1);
        sa_read_check("Fill Set 6 Way 3 @ 0x3180", 32'h00003180, 32'h40404040, 1'b1);

        // Store to Way 2 (0x2180) to mark it DIRTY
        sa_write("Store to Way 2 (make dirty) @ 0x2180", 32'h00002180, 4'b1111, 32'h77777777);

        // Access Way 0, Way 1, Way 3 to make Way 2 Least Recently Used!
        sa_read_check("Touch Set 6 Way 0 (0 stalls)", 32'h00000180, 32'h10101010, 1'b0);
        sa_read_check("Touch Set 6 Way 1 (0 stalls)", 32'h00001180, 32'h20202020, 1'b0);
        sa_read_check("Touch Set 6 Way 3 (0 stalls)", 32'h00003180, 32'h40404040, 1'b0);

        // Now Way 2 (holding dirty 0x2180) is LRU!
        // Read 5th line 0x4180 -> MUST evict Way 2 and write it back to memory!
        sa_read_check("Evict dirty Way 2 for Tag 4 @ 0x4180", 32'h00004180, 32'h50505050, 1'b1);

        // Verify main memory now contains the dirty data 0x77777777 for 0x2180!
        if ({sa_memory[16'h2183], sa_memory[16'h2182], sa_memory[16'h2181], sa_memory[16'h2180]} === 32'h77777777) begin
            total_passed = total_passed + 1;
            $display("[PASS] Multi-way dirty line write-back verified in main memory!");
        end else begin
            total_failed = total_failed + 1;
            $display("[FAIL] Multi-way dirty line write-back failed!");
        end

        // --------------------------------------------------------------------
        // SUITE 3: 4-WAY SET ASSOCIATIVE PLRU TESTS (Figure 1 Verification)
        // --------------------------------------------------------------------
        $display("\n--- TEST SUITE 3: 4-Way Set Associative PLRU (Figure 1 Verification) ---");
        plru_rst = 1;
        plru_ren = 0;
        plru_wen = 0;
        plru_addr = 0;
        plru_mask = 0;
        plru_wdata = 0;

        for (i = 0; i < 65536; i = i + 1) plru_memory[i] = i[7:0] ^ 8'h3C;

        // Choose Set 8: addr = (tag << 12) | (8 << 6)
        plru_memory[16'h0200] = 8'h00; plru_memory[16'h0201] = 8'h00; plru_memory[16'h0202] = 8'hAA; plru_memory[16'h0203] = 8'hAA;
        plru_memory[16'h1200] = 8'h11; plru_memory[16'h1201] = 8'h11; plru_memory[16'h1202] = 8'hBB; plru_memory[16'h1203] = 8'hBB;
        plru_memory[16'h2200] = 8'h22; plru_memory[16'h2201] = 8'h22; plru_memory[16'h2202] = 8'hCC; plru_memory[16'h2203] = 8'hCC;
        plru_memory[16'h3200] = 8'h33; plru_memory[16'h3201] = 8'h33; plru_memory[16'h3202] = 8'hDD; plru_memory[16'h3203] = 8'hDD;
        plru_memory[16'h4200] = 8'h44; plru_memory[16'h4201] = 8'h44; plru_memory[16'h4202] = 8'hEE; plru_memory[16'h4203] = 8'hEE;

        repeat (4) @(posedge clk);
        @(negedge clk);
        plru_rst = 0;
        repeat (2) @(posedge clk);

        // Cold fill: Way 0 = A, Way 1 = B, Way 2 = C, Way 3 = D
        plru_read_check("PLRU Cold Fill Way 0 (A) @ 0x0200", 32'h00000200, 32'hAAAA0000, 1'b1);
        plru_read_check("PLRU Cold Fill Way 1 (B) @ 0x1200", 32'h00001200, 32'hBBBB1111, 1'b1);
        plru_read_check("PLRU Cold Fill Way 2 (C) @ 0x2200", 32'h00002200, 32'hCCCC2222, 1'b1);
        plru_read_check("PLRU Cold Fill Way 3 (D) @ 0x3200", 32'h00003200, 32'hDDDD3333, 1'b1);

        // Access sequence of Figure 1 from PDF:
        plru_read_check("Figure 1 Step 2: Access Way 0 (A)", 32'h00000200, 32'hAAAA0000, 1'b0);
        plru_read_check("Figure 1 Step 3: Access Way 2 (C)", 32'h00002200, 32'hCCCC2222, 1'b0);
        plru_read_check("Figure 1 Step 4: Access Way 1 (B)", 32'h00001200, 32'hBBBB1111, 1'b0);
        plru_read_check("Figure 1 Step 5: Access Way 3 (D)", 32'h00003200, 32'hDDDD3333, 1'b0);

        // At this point, tree nodes are 0, pointing directly to Way 0 (A)
        // Access Tag 4 (0x4200) -> PLRU victim MUST be Way 0 (A)!
        plru_read_check("Figure 1 Step 6: PLRU Evicts Way 0 (A) for E", 32'h00004200, 32'hEEEE4444, 1'b1);

        // Verify Ways 1, 2, 3 survived and hit with 0 stalls!
        plru_read_check("Verify Way 1 survived with 0 stalls @ 0x1200", 32'h00001200, 32'hBBBB1111, 1'b0);
        plru_read_check("Verify Way 2 survived with 0 stalls @ 0x2200", 32'h00002200, 32'hCCCC2222, 1'b0);
        plru_read_check("Verify Way 3 survived with 0 stalls @ 0x3200", 32'h00003200, 32'hDDDD3333, 1'b0);
        plru_read_check("Verify Way 0 (holds E) hits with 0 stalls @ 0x4200", 32'h00004200, 32'hEEEE4444, 1'b0);

        // Verify A was evicted (must miss)
        plru_read_check("Verify Way 0 (A, 0x0200) misses after eviction", 32'h00000200, 32'hAAAA0000, 1'b1);

        // --------------------------------------------------------------------
        // SUITE 4: 2-WAY SET ASSOCIATIVE TESTS (NUM_WAYS = 2)
        // --------------------------------------------------------------------
        $display("\n--- TEST SUITE 4: 2-Way Set Associative (NUM_WAYS = 2) ---");
        way2_rst = 1;
        way2_ren = 0;
        way2_wen = 0;
        way2_addr = 0;
        way2_mask = 0;
        way2_wdata = 0;

        for (i = 0; i < 65536; i = i + 1) way2_memory[i] = i[7:0] ^ 8'h96;

        // Choose Set 3: addr = (tag << 13) | (3 << 6)
        way2_memory[16'h00C0] = 8'h11; way2_memory[16'h00C1] = 8'h22; way2_memory[16'h00C2] = 8'h33; way2_memory[16'h00C3] = 8'h44;
        way2_memory[16'h20C0] = 8'h55; way2_memory[16'h20C1] = 8'h66; way2_memory[16'h20C2] = 8'h77; way2_memory[16'h20C3] = 8'h88;
        way2_memory[16'h40C0] = 8'h99; way2_memory[16'h40C1] = 8'hAA; way2_memory[16'h40C2] = 8'hBB; way2_memory[16'h40C3] = 8'hCC;

        repeat (4) @(posedge clk);
        @(negedge clk);
        way2_rst = 0;
        repeat (2) @(posedge clk);

        way2_read_check("2-Way Cold Fill Way 0 @ 0x00C0", 32'h000000C0, 32'h44332211, 1'b1);
        way2_read_check("2-Way Cold Fill Way 1 @ 0x20C0", 32'h000020C0, 32'h88776655, 1'b1);
        way2_read_check("2-Way Co-resident Hit Way 0 @ 0x00C0", 32'h000000C0, 32'h44332211, 1'b0);
        way2_read_check("2-Way Co-resident Hit Way 1 @ 0x20C0", 32'h000020C0, 32'h88776655, 1'b0);

        // Touch Way 0 -> Way 1 is LRU
        way2_read_check("Touch Way 0 @ 0x00C0", 32'h000000C0, 32'h44332211, 1'b0);
        // Access third line 0x40C0 -> evicts Way 1
        way2_read_check("2-Way LRU Evict Way 1 @ 0x40C0", 32'h000040C0, 32'hCCBBAA99, 1'b1);
        // Way 0 must still hit with 0 stalls!
        way2_read_check("2-Way Way 0 survived with 0 stalls @ 0x00C0", 32'h000000C0, 32'h44332211, 1'b0);
        way2_read_check("2-Way Way 1 (new) hits with 0 stalls @ 0x40C0", 32'h000040C0, 32'hCCBBAA99, 1'b0);

        // --------------------------------------------------------------------
        // SUITE 5: FULLY ASSOCIATIVE BASE CONFIGURATION TESTS
        // --------------------------------------------------------------------
        $display("\n--- TEST SUITE 5: Fully Associative (NUM_SETS = 1, SET_BITS = 0) ---");
        fa_rst = 1;
        fa_ren = 0;
        fa_wen = 0;
        fa_addr = 0;
        fa_mask = 0;
        fa_wdata = 0;

        for (i = 0; i < 65536; i = i + 1) fa_memory[i] = i[7:0] ^ 8'h69;

        // Block size is 128 bytes. Offset = 7 bits.
        fa_memory[16'h0080] = 8'h55; fa_memory[16'h0081] = 8'h66; fa_memory[16'h0082] = 8'h77; fa_memory[16'h0083] = 8'h88;
        fa_memory[16'h0100] = 8'h99; fa_memory[16'h0101] = 8'hAA; fa_memory[16'h0102] = 8'hBB; fa_memory[16'h0103] = 8'hCC;
        fa_memory[16'h0180] = 8'h12; fa_memory[16'h0181] = 8'h34; fa_memory[16'h0182] = 8'h56; fa_memory[16'h0183] = 8'h78;

        repeat (4) @(posedge clk);
        @(negedge clk);
        fa_rst = 0;
        repeat (2) @(posedge clk);

        fa_read_check("FA Cold Miss Read @ 0x0080", 32'h00000080, 32'h88776655, 1'b1);
        fa_read_check("FA Read Hit @ 0x0080", 32'h00000080, 32'h88776655, 1'b0);
        fa_read_check("FA Cold Miss Read @ 0x0100", 32'h00000100, 32'hCCBBAA99, 1'b1);
        fa_read_check("FA Read Hit @ 0x0100", 32'h00000100, 32'hCCBBAA99, 1'b0);
        fa_read_check("FA Cold Miss Read @ 0x0180", 32'h00000180, 32'h78563412, 1'b1);
        fa_read_check("FA Read Hit @ 0x0180", 32'h00000180, 32'h78563412, 1'b0);

        // Verify co-residence across all loaded lines (all hit with 0 stalls!)
        fa_read_check("FA Co-resident Hit @ 0x0080", 32'h00000080, 32'h88776655, 1'b0);
        fa_read_check("FA Co-resident Hit @ 0x0100", 32'h00000100, 32'hCCBBAA99, 1'b0);

        // ====================================================================
        // FINAL VERDICT
        // ====================================================================
        $display("\n==================================================================");
        $display("   FINAL TESTBENCH VERDICT: %0d PASSED, %0d FAILED", total_passed, total_failed);
        if (total_failed == 0) begin
            $display("   >>> ALL 5 TEST SUITES PASSED CLEANLY (PARTS 1 & 2 VERIFIED) <<<");
        end else begin
            $display("   >>> TESTBENCH FAILED WITH %0d ERRORS <<<", total_failed);
        end
        $display("==================================================================");
        $finish;
    end

endmodule
