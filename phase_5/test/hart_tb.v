`timescale 1ns / 1ps
`default_nettype none

module hart_tb;

    parameter RESET_ADDR   = 32'h00400000;
    parameter EVICT_POLICY = 0;
    parameter NUM_WAYS     = 128;
    parameter CACHE_SIZE   = 16;
    parameter BLOCK_SIZE   = 128;
    parameter PREFETCH_EN  = 0;

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
        .o_retire_next_pc    (retire_next_pc)
    );

    // 64 KB memory model:
    // IMEM: 0x00400000 to 0x0040FFFF (64 KB)
    // DMEM: 0x10010000 to 0x1001FFFF (64 KB)
    reg [7:0] imem_bytes [0:65535];
    reg [7:0] dmem_bytes [0:65535];
    reg [31:0] expected_dmem [0:16383];

    // Pipelined read queues (delay = 4 cycles)
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
    reg [15:0] base_offset, wr_offset, d_base_offset;
    reg [31:0] actual_word;

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

        for (i = 0; i < 65536; i = i + 1) begin
            imem_bytes[i] = 8'h00;
            dmem_bytes[i] = 8'h00;
        end

        // Load random_program.hex into imem
        $readmemh("traces/random_program.hex", expected_dmem); // reuse buffer to load words
        for (i = 0; i < 16113; i = i + 1) begin
            prog_word = expected_dmem[i];
            imem_bytes[i*4 + 0] = prog_word[ 7: 0];
            imem_bytes[i*4 + 1] = prog_word[15: 8];
            imem_bytes[i*4 + 2] = prog_word[23:16];
            imem_bytes[i*4 + 3] = prog_word[31:24];
        end

        // Load expected dmem
        $readmemh("traces/random_dmem.hex", expected_dmem);

        repeat (5) @(posedge clk);
        @(negedge clk);
        rst = 0;
        $display("Reset released. Executing random_program.hex...");
    end

    // Instruction Memory Controller (Read Only)
    always @(posedge clk) begin
        if (rst) begin
            imem_valid   <= 0;
            imem_rd_data <= 0;
            imem_q_head  <= 0;
            imem_q_tail  <= 0;
            imem_q_count <= 0;
        end else begin
            imem_valid <= 0;

            // Enqueue new read
            if (imem_rd) begin
                imem_q_addr[imem_q_tail]  <= imem_rd_addr;
                imem_q_timer[imem_q_tail] <= LATENCY;
                imem_q_tail  <= (imem_q_tail + 1) % 16;
                imem_q_count <= imem_q_count + 1;
            end

            // Decrement timers
            for (i = 0; i < 16; i = i + 1) begin
                if (i < imem_q_count) begin
                    idx = (imem_q_head + i) % 16;
                    if (imem_q_timer[idx] > 0)
                        imem_q_timer[idx] <= imem_q_timer[idx] - 1;
                end
            end

            // Check if head is ready
            if (imem_q_count > 0 && imem_q_timer[imem_q_head] <= 1) begin
                base_offset = imem_q_addr[imem_q_head][15:0];
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    imem_rd_data[b*8 +: 8] <= imem_bytes[base_offset + b];
                end
                imem_valid   <= 1;
                imem_q_head  <= (imem_q_head + 1) % 16;
                imem_q_count <= imem_q_count - 1;
            end
        end
    end

    // Data Memory Controller (Read and Write)
    always @(posedge clk) begin
        if (rst) begin
            dmem_valid   <= 0;
            dmem_rd_data <= 0;
            dmem_q_head  <= 0;
            dmem_q_tail  <= 0;
            dmem_q_count <= 0;
        end else begin
            dmem_valid <= 0;

            // Handle Writes: applied on clock edge
            if (dmem_wr) begin
                wr_offset = dmem_wr_addr[15:0];
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    if (dmem_wr_be[b]) begin
                        dmem_bytes[wr_offset + b] = dmem_wr_data[b*8 +: 8];
                    end
                end
            end

            // Enqueue new read
            if (dmem_rd) begin
                dmem_q_addr[dmem_q_tail]  <= dmem_rd_addr;
                dmem_q_timer[dmem_q_tail] <= LATENCY;
                dmem_q_tail  <= (dmem_q_tail + 1) % 16;
                dmem_q_count <= dmem_q_count + 1;
            end

            // Decrement timers
            for (i = 0; i < 16; i = i + 1) begin
                if (i < dmem_q_count) begin
                    didx = (dmem_q_head + i) % 16;
                    if (dmem_q_timer[didx] > 0)
                        dmem_q_timer[didx] <= dmem_q_timer[didx] - 1;
                end
            end

            // Check if head is ready
            if (dmem_q_count > 0 && dmem_q_timer[dmem_q_head] <= 1) begin
                d_base_offset = dmem_q_addr[dmem_q_head][15:0];
                for (b = 0; b < BLOCK_SIZE; b = b + 1) begin
                    dmem_rd_data[b*8 +: 8] <= dmem_bytes[d_base_offset + b];
                end
                dmem_valid   <= 1;
                dmem_q_head  <= (dmem_q_head + 1) % 16;
                dmem_q_count <= dmem_q_count - 1;
            end
        end
    end

    // Instruction Retirement & Halt Tracking
    always @(posedge clk) begin
        if (!rst) begin
            cycle_count <= cycle_count + 1;

            if (retire_valid) begin
                retired_count <= retired_count + 1;
                if (retire_halt) begin
                    $display("\n[HALT DETECTED] Instruction retired with ebreak at PC = 0x%08x after %0d cycles (%0d instructions retired).",
                             retire_pc, cycle_count, retired_count + 1);

                    // Wait a couple cycles for any final writebacks to flush
                    repeat (10) @(posedge clk);

                    $display("\nChecking dirty lines remaining in dcache:");
                    for (i = 0; i < NUM_WAYS; i = i + 1) begin
                        if (dut.dcache_i.valid[0][i] && dut.dcache_i.dirty[0][i]) begin
                            $display("dcache way %0d is DIRTY: tag=0x%08x", i, dut.dcache_i.tags[0][i]);
                        end
                    end

                    // Compare data memory with random_dmem.hex
                    $display("Comparing data memory (16,384 words) against random_dmem.hex...");
                    for (i = 0; i < 16384; i = i + 1) begin
                        actual_word = {dmem_bytes[i*4 + 3], dmem_bytes[i*4 + 2], dmem_bytes[i*4 + 1], dmem_bytes[i*4 + 0]};
                        if (actual_word !== expected_dmem[i]) begin
                            if (mismatch_count < 10) begin
                                $display("[MISMATCH] DMEM @ 0x%08x: actual = 0x%08x, expected = 0x%08x",
                                         32'h10010000 + i*4, actual_word, expected_dmem[i]);
                            end
                            mismatch_count = mismatch_count + 1;
                        end
                    end

                    if (mismatch_count == 0) begin
                        $display("==================================================================");
                        $display("   FULL TRACE TEST RESULT: ALL 16,384 WORDS MATCHED PERFECTLY!   ");
                        $display("   >>> VERIFICATION SUCCESSFUL (100%% PASS) <<<                   ");
                        $display("==================================================================");
                    end else begin
                        $display("==================================================================");
                        $display("   FULL TRACE TEST RESULT: %0d MISMATCHES FOUND!                ", mismatch_count);
                        $display("==================================================================");
                    end

                    $finish;
                end
            end

            // Watchdog: max 200,000 cycles
            if (cycle_count > 250000) begin
                $display("[TIMEOUT] Reached cycle limit %0d without ebreak halt! Retired %0d instructions.",
                         cycle_count, retired_count);
                $finish;
            end
        end
    end

endmodule
