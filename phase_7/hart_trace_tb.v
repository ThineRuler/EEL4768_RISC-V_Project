`timescale 1ns / 1ps
`default_nettype none

// Trace verification testbench for Phase 7 hart.v against Phase 4 traces:
//  1) ../phase_4/traces/no_hazard_program.hex vs ../phase_4/traces/no_hazard.trace (12,927 vectors)
//  2) ../phase_4/traces/hazard_program.hex (compares retire trace + reports CPI)
module hart_trace_tb;

    parameter RESET_ADDR   = 32'h00400000;
    parameter EVICT_POLICY = 1;
    parameter NUM_WAYS     = 4;
    parameter CACHE_SIZE   = 2;
    parameter BLOCK_SIZE   = 16;
    parameter PREFETCH_EN  = 1;

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
        .WRITE_ALLOCATE (0)
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

    localparam IMEM_WORDS = 32768;
    reg [31:0] imem [0:IMEM_WORDS-1];
    wire [31:0] imem_idx = (imem_raddr - RESET_ADDR) >> 2;
    assign imem_rdata = (imem_raddr >= RESET_ADDR && imem_idx < IMEM_WORDS) ?
                        imem[imem_idx] : 32'h00000013;

    // 1MB DMEM covering 0x10000000..0x10100000 (trace uses 0x10010000+)
    localparam DMEM_BYTES = 262144;
    reg [7:0] dmem_bytes [0:DMEM_BYTES-1];
    localparam LATENCY = 4;

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
                dmem_q_timer[dmem_q_tail] <= LATENCY;
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

    integer fd, rc, k;
    integer vec_num, fail_num, cyc, wait_cyc;
    reg [1023:0] line;
    reg [31:0] exp_pc, exp_inst, exp_rs1_val, exp_rs2_val, exp_rd_val;
    reg [31:0] exp_maddr, exp_mwdata, exp_npc;
    reg        exp_trap, exp_halt;
    reg [ 4:0] exp_rs1, exp_rs2, exp_rd;
    reg [ 1:0] exp_mop;
    reg [ 3:0] exp_mmask;
    reg [63:0] s_rs1, s_rs1v, s_rs2, s_rs2v, s_rdv, s_ma, s_mm, s_mw, s_npc;
    reg        vec_ok;

    task parse_hex_or_x(input [63:0] tok, output reg [31:0] val, output reg is_x);
        integer tmp;
        begin
            if (tok[7:0] == "x" || tok[7:0] == "X") begin
                is_x = 1'b1;
                val  = 32'd0;
            end else begin
                is_x = 1'b0;
                tmp  = $sscanf(tok, "%h", val);
            end
        end
    endtask

    reg is_x_rs1, is_x_rs1v, is_x_rs2, is_x_rs2v, is_x_rdv, is_x_ma, is_x_mm, is_x_mw, is_x_npc;

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        fail_num = 0;

        // -----------------------------------------------------------------
        // 1. Run no_hazard_program.hex against no_hazard.trace (12,927 ops)
        // -----------------------------------------------------------------
        for (k = 0; k < IMEM_WORDS; k = k + 1) imem[k] = 32'h00000013;
        for (k = 0; k < DMEM_BYTES; k = k + 1) dmem_bytes[k] = 8'h00;

        $readmemh("../phase_4/traces/no_hazard_program.hex", imem);
        fd = $fopen("../phase_4/traces/no_hazard.trace", "r");
        if (fd == 0) begin
            $display("ERROR: cannot open ../phase_4/traces/no_hazard.trace");
            $fatal(1);
        end

        @(negedge clk);
        rst = 1'b1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;

        vec_num = 0;
        cyc     = 0;

        while (!$feof(fd)) begin
            line = 0;
            rc = $fgets(line, fd);
            if (rc > 0) begin
                // Skip comments and blank lines (first non-space char '#' or empty)
                if ($sscanf(line, "%h %h %d %d %s %s %s %s %h %s %d %s %s %s %s",
                            exp_pc, exp_inst, exp_trap, exp_halt,
                            s_rs1, s_rs1v, s_rs2, s_rs2v,
                            exp_rd, s_rdv, exp_mop, s_ma, s_mm, s_mw, s_npc) == 15) begin

                    parse_hex_or_x(s_rs1,  exp_rs1,     is_x_rs1);
                    parse_hex_or_x(s_rs1v, exp_rs1_val, is_x_rs1v);
                    parse_hex_or_x(s_rs2,  exp_rs2,     is_x_rs2);
                    parse_hex_or_x(s_rs2v, exp_rs2_val, is_x_rs2v);
                    parse_hex_or_x(s_rdv,  exp_rd_val,  is_x_rdv);
                    parse_hex_or_x(s_ma,   exp_maddr,   is_x_ma);
                    parse_hex_or_x(s_mm,   exp_mmask,   is_x_mm);
                    parse_hex_or_x(s_mw,   exp_mwdata,  is_x_mw);
                    parse_hex_or_x(s_npc,  exp_npc,     is_x_npc);

                    wait_cyc = 0;
                    @(negedge clk);
                    cyc = cyc + 1;
                    while (!retire_valid) begin
                        wait_cyc = wait_cyc + 1;
                        if (wait_cyc > 10000) begin
                            $display("[FAIL] Hang waiting for vector %0d (pc=%08h)", vec_num, exp_pc);
                            $fatal(1);
                        end
                        @(negedge clk);
                        cyc = cyc + 1;
                    end

                    vec_ok = 1'b1;
                    if (retire_pc !== exp_pc)         vec_ok = 1'b0;
                    if (retire_inst !== exp_inst)     vec_ok = 1'b0;
                    if (retire_trap !== exp_trap)     vec_ok = 1'b0;
                    if (retire_halt !== exp_halt)     vec_ok = 1'b0;
                    if (!is_x_rs1  && retire_rs1_raddr !== exp_rs1)     vec_ok = 1'b0;
                    if (!is_x_rs1v && retire_rs1_rdata !== exp_rs1_val) vec_ok = 1'b0;
                    if (!is_x_rs2  && retire_rs2_raddr !== exp_rs2)     vec_ok = 1'b0;
                    if (!is_x_rs2v && retire_rs2_rdata !== exp_rs2_val) vec_ok = 1'b0;
                    if (retire_rd_waddr !== exp_rd)   vec_ok = 1'b0;
                    if (!is_x_rdv  && exp_rd != 5'd0 && retire_rd_wdata !== exp_rd_val) vec_ok = 1'b0;
                    if (exp_mop == 2'd0 && (retire_dmem_ren || retire_dmem_wen)) vec_ok = 1'b0;
                    if (exp_mop == 2'd1 && (!retire_dmem_ren || retire_dmem_wen)) vec_ok = 1'b0;
                    if (exp_mop == 2'd2 && (!retire_dmem_wen || retire_dmem_ren)) vec_ok = 1'b0;
                    if (exp_mop != 2'd0) begin
                        if (!is_x_ma && retire_dmem_addr !== exp_maddr) vec_ok = 1'b0;
                        if (!is_x_mm && retire_dmem_mask !== exp_mmask) vec_ok = 1'b0;
                        if (exp_mop == 2'd2 && !is_x_mw) begin
                            if (exp_mmask[0] && retire_dmem_wdata[ 7: 0] !== exp_mwdata[ 7: 0]) vec_ok = 1'b0;
                            if (exp_mmask[1] && retire_dmem_wdata[15: 8] !== exp_mwdata[15: 8]) vec_ok = 1'b0;
                            if (exp_mmask[2] && retire_dmem_wdata[23:16] !== exp_mwdata[23:16]) vec_ok = 1'b0;
                            if (exp_mmask[3] && retire_dmem_wdata[31:24] !== exp_mwdata[31:24]) vec_ok = 1'b0;
                        end
                    end

                    if (!vec_ok) begin
                        fail_num = fail_num + 1;
                        if (fail_num <= 20) begin
                            $display("[FAIL] vec %0d pc=%08h inst=%08h | got pc=%08h inst=%08h trap=%b halt=%b rs1=(%0d:%08h vs %0d:%08h) rs2=(%0d:%08h vs %0d:%08h) rd=(%0d:%08h vs %0d:%08h) mem=(%b/%b %08h %b %08h vs mop=%0d %08h %b %08h)",
                                     vec_num, exp_pc, exp_inst,
                                     retire_pc, retire_inst, retire_trap, retire_halt,
                                     retire_rs1_raddr, retire_rs1_rdata, exp_rs1, exp_rs1_val,
                                     retire_rs2_raddr, retire_rs2_rdata, exp_rs2, exp_rs2_val,
                                     retire_rd_waddr, retire_rd_wdata, exp_rd, exp_rd_val,
                                     retire_dmem_ren, retire_dmem_wen, retire_dmem_addr, retire_dmem_mask, retire_dmem_wdata,
                                     exp_mop, exp_maddr, exp_mmask, exp_mwdata);
                        end
                    end
                    vec_num = vec_num + 1;
                end
            end
        end
        $fclose(fd);

        $display("no_hazard trace: %0d vectors, %0d failed, %0d cycles (CPI = %0f)",
                 vec_num, fail_num, cyc, (1.0 * cyc) / vec_num);

        // -----------------------------------------------------------------
        // 2. Run hazard_program.hex (40 instructions; Phase 4+fwd = 49 cycles)
        // -----------------------------------------------------------------
        for (k = 0; k < IMEM_WORDS; k = k + 1) imem[k] = 32'h00000013;
        for (k = 0; k < DMEM_BYTES; k = k + 1) dmem_bytes[k] = 8'h00;
        $readmemh("../phase_4/traces/hazard_program.hex", imem);

        @(negedge clk);
        rst = 1'b1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;

        vec_num = 0;
        cyc     = 0;
        begin : hazard_loop
            reg halted_flag;
            halted_flag = 1'b0;
            while (!halted_flag && cyc < 5000) begin
                @(negedge clk);
                cyc = cyc + 1;
                if (retire_valid) begin
                    vec_num = vec_num + 1;
                    if (retire_halt) halted_flag = 1'b1;
                end
            end
        end
        $display("hazard_program: %0d retired in %0d cycles (CPI = %0f)",
                 vec_num, cyc, (1.0 * cyc) / vec_num);

        if (fail_num > 0) $fatal(1, "Trace verification failed!");
        $display(">>> ALL PHASE 4 TRACE VECTORS PASSED <<<");
        $finish;
    end

endmodule

`default_nettype wire
