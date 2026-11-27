`timescale 1ns / 1ps
`default_nettype none

// Phase 6 full RV32IMF + cache stress testbench adapted for Phase 7 OoO hart.v.
// Runs ../phase_6/traces/random_program.hex (36,088 words) and verifies all
// 32,768 words of final DMEM against ../phase_6/traces/random_dmem.hex.
module hart_p6_tb;

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

    reg [31:0] raw_imem_words [0:65535];
    reg [7:0]  dmem_bytes     [0:131071];
    reg [31:0] expected_dmem  [0:32767];

    wire [31:0] imem_idx = (imem_raddr - RESET_ADDR) >> 2;
    assign imem_rdata = (imem_raddr >= RESET_ADDR && imem_idx < 65536) ?
                        raw_imem_words[imem_idx] : 32'h00000013;

    localparam LATENCY = 4;
    reg [31:0] dmem_q_addr  [0:15];
    integer    dmem_q_timer [0:15];
    integer    dmem_q_head, dmem_q_tail, dmem_q_count;
    integer    i, b, didx, retired_count, cycle_count, mismatch_count;
    reg [31:0] dmem_offset, actual_word, exp_word;

    always #5 clk = ~clk;

    initial begin
        clk = 0; rst = 1;
        mem_valid = 0; mem_rd_data = 0;
        dmem_q_head = 0; dmem_q_tail = 0; dmem_q_count = 0;
        retired_count = 0; cycle_count = 0; mismatch_count = 0;

        for (i = 0; i < 65536; i = i + 1) raw_imem_words[i] = 32'h00000013;
        for (i = 0; i < 131072; i = i + 1) dmem_bytes[i] = 8'h00;

        $readmemh("../phase_6/traces/random_program.hex", raw_imem_words);
        $readmemh("../phase_6/traces/random_dmem.hex", expected_dmem);

        repeat (10) @(posedge clk);
        @(negedge clk);
        rst = 0;
    end

    always @(posedge clk) begin
        if (rst) begin
            mem_valid    <= 0;
            mem_rd_data  <= 0;
            dmem_q_head  <= 0;
            dmem_q_tail  <= 0;
            dmem_q_count <= 0;
        end else begin
            mem_valid <= 0;

            if (mem_wr) begin
                dmem_offset = mem_wr_addr - 32'h10010000;
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    if (mem_wr_be[b])
                        dmem_bytes[dmem_offset + b] = mem_wr_data[b*8 +: 8];
                end
            end

            if (mem_rd) begin
                dmem_q_addr[dmem_q_tail]  <= mem_rd_addr;
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
                    mem_rd_data[b*8 +: 8] <= dmem_bytes[dmem_offset + b];
                end
                mem_valid    <= 1;
                dmem_q_head  <= (dmem_q_head + 1) % 16;
                dmem_q_count <= dmem_q_count - 1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst) begin
            cycle_count <= cycle_count + 1;

            if (retire_valid) begin
                retired_count <= retired_count + 1;
                if (retire_halt) begin
                    $display("[HALT] EBREAK retired at cycle %0d | Retired: %0d | CPI = %0f",
                             cycle_count, retired_count + 1, (1.0 * cycle_count) / (retired_count + 1));
                    repeat (50) @(posedge clk);
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
                                $display("  [MISMATCH] Addr 0x%08h | Exp 0x%08h | Got 0x%08h",
                                         32'h10010000 + i*4, exp_word, actual_word);
                            end
                        end
                    end

                    if (mismatch_count == 0) begin
                        $display(">>> SUCCESS: ALL 32,768 WORDS MATCH PERFECTLY! <<<");
                        $finish;
                    end else begin
                        $display(">>> FAILURE: %0d WORDS MISMATCHED! <<<", mismatch_count);
                        $fatal(1);
                    end
                end
            end

            if (cycle_count > 5000000) begin
                $display("[ERROR] Timeout after 5,000,000 cycles (retired %0d, last pc %08h)", retired_count, retire_pc);
                $fatal(1);
            end
        end
    end

endmodule

`default_nettype wire
