`timescale 1ns / 1ps
`default_nettype none

// ============================================================================
// Module: hart_tb
// Description: Comprehensive self-checking testbench for 5-stage pipelined
//              RV32I processor (hart.v) for EEL 4768 Phase 4.
//
// Tests all rubric requirements specified in phase_4.pdf:
//   1. Pipelined Processor (3.0 pts):
//      - 5-stage pipeline behavior (IF, ID, EX, MEM, WB)
//      - RESET_ADDR = 32'h00400000 initialization and fill latency
//      - Correct retire interface reporting on retirement
//      - Sub-word memory accesses (SB, LB) with byte-lane masking & sign ext
//      - ebreak instruction halt
//   2. Hazard Detection (2.0 pts):
//      - Load-use hazard: 1-cycle stall bubble insertion, PC/IF_ID freeze
//      - Load-store hazard: 1-cycle stall bubble insertion
//      - Control hazard: Taken branch resolved in EX flushes 2 instructions (2 bubbles)
//      - Control hazard: Not-taken branch resolved in EX with 0 flushes (0 bubbles)
//      - Control hazard: JAL unconditional jump flushes 2 instructions and links PC+4
//      - False dependency immunity: Immediate overlaps, non-reading ops, x0 discard
//      - Traps: Illegal instruction and misaligned memory access retirement with
//        trap=1, suppression of side effects (no RF write, no DMEM access), and
//        continuation at PC+4
//   3. Forwarding Logic (2.0 pts):
//      - EX -> EX forwarding (back-to-back RAW dependencies without stalls)
//      - MEM -> EX forwarding (RAW dependencies across 2 instructions)
//      - Simultaneous dual-operand forwarding (rs1 and rs2 simultaneously)
//      - Forwarding priority: EX/MEM (newer) over MEM/WB (older)
//      - MEM -> MEM store data forwarding
//
// How to compile and run:
//   From project root:
//     iverilog -s hart_tb -o sim phase_4/test/hart_tb.v phase_4/test/hart.v \
//         phase_4/test/alu.v phase_4/test/imm.v phase_4/test/rf.v phase_4/test/decoder.v
//     vvp sim
//
//   From phase_4/test/ directory:
//     iverilog -s hart_tb -o sim hart_tb.v hart.v alu.v imm.v rf.v decoder.v
//     vvp sim
// ============================================================================

module hart_tb;

    // -------------------------------------------------------------------------
    // 1. Signals
    // -------------------------------------------------------------------------
    reg         clk;
    reg         rst;

    wire [31:0] imem_raddr;
    wire [31:0] imem_rdata;

    wire [31:0] dmem_addr;
    wire        dmem_ren;
    wire        dmem_wen;
    wire [31:0] dmem_wdata;
    wire [ 3:0] dmem_mask;
    wire [31:0] dmem_rdata;

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

    integer passed;
    integer failed;
    integer cycle;

    // -------------------------------------------------------------------------
    // 2. Device Under Test (DUT)
    // -------------------------------------------------------------------------
    hart #(
        .RESET_ADDR(32'h00400000),
        .FWD_EN(1),
        .BYPASS_EN(1)
    ) dut (
        .i_clk              (clk),
        .i_rst              (rst),
        .o_imem_raddr       (imem_raddr),
        .i_imem_rdata       (imem_rdata),
        .o_dmem_addr        (dmem_addr),
        .o_dmem_ren         (dmem_ren),
        .o_dmem_wen         (dmem_wen),
        .o_dmem_wdata       (dmem_wdata),
        .o_dmem_mask        (dmem_mask),
        .i_dmem_rdata       (dmem_rdata),
        .o_retire_valid     (retire_valid),
        .o_retire_inst      (retire_inst),
        .o_retire_trap      (retire_trap),
        .o_retire_halt      (retire_halt),
        .o_retire_rs1_raddr (retire_rs1_raddr),
        .o_retire_rs2_raddr (retire_rs2_raddr),
        .o_retire_rs1_rdata (retire_rs1_rdata),
        .o_retire_rs2_rdata (retire_rs2_rdata),
        .o_retire_rd_waddr  (retire_rd_waddr),
        .o_retire_rd_wdata  (retire_rd_wdata),
        .o_retire_dmem_addr (retire_dmem_addr),
        .o_retire_dmem_mask (retire_dmem_mask),
        .o_retire_dmem_ren  (retire_dmem_ren),
        .o_retire_dmem_wen  (retire_dmem_wen),
        .o_retire_dmem_rdata(retire_dmem_rdata),
        .o_retire_dmem_wdata(retire_dmem_wdata),
        .o_retire_pc        (retire_pc),
        .o_retire_next_pc   (retire_next_pc)
    );

    // -------------------------------------------------------------------------
    // 3. Simulated Memories
    // -------------------------------------------------------------------------
    // Instruction memory: indexed relative to RESET_ADDR (0x00400000)
    localparam IMEM_SIZE = 1024;
    reg [31:0] imem [0:IMEM_SIZE-1];
    wire [31:0] imem_idx = (imem_raddr - 32'h00400000) >> 2;
    assign imem_rdata = (imem_raddr >= 32'h00400000 && imem_idx < IMEM_SIZE) ?
                        imem[imem_idx] : 32'h00000013; // default NOP

    // Data memory: word-addressed with byte-lane masking
    localparam DMEM_SIZE = 2048;
    reg [31:0] dmem [0:DMEM_SIZE-1];
    wire [31:0] dmem_idx = (dmem_addr >> 2) & 32'h7ff;
    assign dmem_rdata = dmem[dmem_idx];

    always @(posedge clk) begin
        if (dmem_wen) begin
            if (dmem_mask[0]) dmem[dmem_idx][ 7: 0] <= dmem_wdata[ 7: 0];
            if (dmem_mask[1]) dmem[dmem_idx][15: 8] <= dmem_wdata[15: 8];
            if (dmem_mask[2]) dmem[dmem_idx][23:16] <= dmem_wdata[23:16];
            if (dmem_mask[3]) dmem[dmem_idx][31:24] <= dmem_wdata[31:24];
        end
    end

    // Clock: 10ns period (100 MHz)
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // 4. Verification Check Task
    // -------------------------------------------------------------------------
    task check_retire;
        input [511:0] test_name;
        input [ 31:0] exp_pc;
        input [ 31:0] exp_inst;
        input [  4:0] exp_rd;
        input [ 31:0] exp_rd_val;
        input [ 31:0] exp_next_pc;
        input         exp_halt;
        input         exp_trap;
        input integer exp_bubbles; // Expected bubble cycles (stalls or flushes) before retiring

        integer bubbles;
        reg ok;
        begin
            bubbles = 0;
            @(negedge clk);
            cycle = cycle + 1;

            // Advance through any bubble cycles until an instruction retires
            while (!retire_valid) begin
                bubbles = bubbles + 1;
                @(negedge clk);
                cycle = cycle + 1;
            end

            ok = 1'b1;

            // 1. Verify stall / flush bubble count
            if (bubbles !== exp_bubbles) begin
                $display("[FAIL] cycle %0d: %0s -> bubble mismatch: got %0d bubbles, exp %0d bubbles",
                         cycle, test_name, bubbles, exp_bubbles);
                ok = 1'b0;
            end

            // 2. Verify Program Counter
            if (retire_pc !== exp_pc) begin
                $display("[FAIL] cycle %0d: %0s -> PC mismatch: got %08h, exp %08h",
                         cycle, test_name, retire_pc, exp_pc);
                ok = 1'b0;
            end

            // 3. Verify Instruction Word
            if (retire_inst !== exp_inst) begin
                $display("[FAIL] cycle %0d: %0s -> inst mismatch: got %08h, exp %08h",
                         cycle, test_name, retire_inst, exp_inst);
                ok = 1'b0;
            end

            // 4. Verify Destination Register Address
            if (retire_rd_waddr !== exp_rd) begin
                $display("[FAIL] cycle %0d: %0s -> rd_waddr mismatch: got %0d, exp %0d",
                         cycle, test_name, retire_rd_waddr, exp_rd);
                ok = 1'b0;
            end

            // 5. Verify Destination Register Data
            if (exp_rd != 5'd0 && retire_rd_wdata !== exp_rd_val) begin
                $display("[FAIL] cycle %0d: %0s -> rd_wdata mismatch: got %08h, exp %08h",
                         cycle, test_name, retire_rd_wdata, exp_rd_val);
                ok = 1'b0;
            end

            // 6. Verify Next PC
            if (retire_next_pc !== exp_next_pc) begin
                $display("[FAIL] cycle %0d: %0s -> next_pc mismatch: got %08h, exp %08h",
                         cycle, test_name, retire_next_pc, exp_next_pc);
                ok = 1'b0;
            end

            // 7. Verify Halt Flag (ebreak)
            if (retire_halt !== exp_halt) begin
                $display("[FAIL] cycle %0d: %0s -> halt mismatch: got %b, exp %b",
                         cycle, test_name, retire_halt, exp_halt);
                ok = 1'b0;
            end

            // 8. Verify Trap Flag
            if (retire_trap !== exp_trap) begin
                $display("[FAIL] cycle %0d: %0s -> trap mismatch: got %b, exp %b",
                         cycle, test_name, retire_trap, exp_trap);
                ok = 1'b0;
            end

            if (ok) begin
                passed = passed + 1;
                $display("[PASS] cycle %0d: %0s (PC=%08h, rd=%0d, wdata=%08h, bubbles=%0d)",
                         cycle, test_name, exp_pc, exp_rd, exp_rd_val, bubbles);
            end else begin
                failed = failed + 1;
            end
        end
    endtask

    // Check retire DMEM ports for memory instructions
    task check_retire_dmem;
        input [511:0] test_name;
        input         exp_ren;
        input         exp_wen;
        input [ 31:0] exp_addr;
        input [  3:0] exp_mask;

        reg ok;
        begin
            ok = 1'b1;
            if (retire_dmem_ren !== exp_ren) begin
                $display("[FAIL] cycle %0d: %0s -> retire_dmem_ren mismatch: got %b, exp %b",
                         cycle, test_name, retire_dmem_ren, exp_ren);
                ok = 1'b0;
            end
            if (retire_dmem_wen !== exp_wen) begin
                $display("[FAIL] cycle %0d: %0s -> retire_dmem_wen mismatch: got %b, exp %b",
                         cycle, test_name, retire_dmem_wen, exp_wen);
                ok = 1'b0;
            end
            if ((exp_ren || exp_wen) && retire_dmem_addr !== exp_addr) begin
                $display("[FAIL] cycle %0d: %0s -> retire_dmem_addr mismatch: got %08h, exp %08h",
                         cycle, test_name, retire_dmem_addr, exp_addr);
                ok = 1'b0;
            end
            if ((exp_ren || exp_wen) && retire_dmem_mask !== exp_mask) begin
                $display("[FAIL] cycle %0d: %0s -> retire_dmem_mask mismatch: got %b, exp %b",
                         cycle, test_name, retire_dmem_mask, exp_mask);
                ok = 1'b0;
            end
            if (!ok) failed = failed + 1;
        end
    endtask

    // Check retire source register addresses and forwarded data
    task check_retire_rs;
        input [511:0] test_name;
        input [  4:0] exp_rs1;
        input [ 31:0] exp_rs1_val;
        input [  4:0] exp_rs2;
        input [ 31:0] exp_rs2_val;

        reg ok;
        begin
            ok = 1'b1;
            if (retire_rs1_raddr !== exp_rs1) begin
                $display("[FAIL] cycle %0d: %0s -> rs1_raddr mismatch: got %0d, exp %0d",
                         cycle, test_name, retire_rs1_raddr, exp_rs1);
                ok = 1'b0;
            end
            if (exp_rs1 != 5'd0 && retire_rs1_rdata !== exp_rs1_val) begin
                $display("[FAIL] cycle %0d: %0s -> rs1_rdata mismatch: got %08h, exp %08h",
                         cycle, test_name, retire_rs1_rdata, exp_rs1_val);
                ok = 1'b0;
            end
            if (retire_rs2_raddr !== exp_rs2) begin
                $display("[FAIL] cycle %0d: %0s -> rs2_raddr mismatch: got %0d, exp %0d",
                         cycle, test_name, retire_rs2_raddr, exp_rs2);
                ok = 1'b0;
            end
            if (exp_rs2 != 5'd0 && retire_rs2_rdata !== exp_rs2_val) begin
                $display("[FAIL] cycle %0d: %0s -> rs2_rdata mismatch: got %08h, exp %08h",
                         cycle, test_name, retire_rs2_rdata, exp_rs2_val);
                ok = 1'b0;
            end
            if (!ok) failed = failed + 1;
        end
    endtask

    // -------------------------------------------------------------------------
    // 5. Test Execution
    // -------------------------------------------------------------------------
    integer i;

    initial begin
        $dumpfile("hart.vcd");
        $dumpvars(0, hart_tb);

        clk    = 1'b0;
        rst    = 1'b1;
        passed = 0;
        failed = 0;
        cycle  = 0;

        for (i = 0; i < IMEM_SIZE; i = i + 1) imem[i] = 32'h00000013; // default NOP
        for (i = 0; i < DMEM_SIZE; i = i + 1) dmem[i] = 32'd0;

        // ---------------------------------------------------------------------
        // Load Test Program
        // ---------------------------------------------------------------------
        // Address      Instruction          Assembly                Description
        imem[ 0] = 32'h10010537; // PC=00400000: lui  x10, 0x10010   Base DMEM pointer (0x10010000)
        imem[ 1] = 32'h00f00093; // PC=00400004: addi x1, x0, 15     x1 = 15
        imem[ 2] = 32'h01b00113; // PC=00400008: addi x2, x0, 27     x2 = 27
        imem[ 3] = 32'h002081b3; // PC=0040000c: add  x3, x1, x2     x3 = 42 (RAW: EX->EX fwd from x2)
        imem[ 4] = 32'h40118233; // PC=00400010: sub  x4, x3, x1     x4 = 27 (RAW: EX->EX fwd from x3)
        imem[ 5] = 32'h0041f2b3; // PC=00400014: and  x5, x3, x4     x5 = 10 (RAW: MEM->EX fwd x3, EX->EX fwd x4)
        imem[ 6] = 32'h06400313; // PC=00400018: addi x6, x0, 100    x6 = 100
        imem[ 7] = 32'h0c800313; // PC=0040001c: addi x6, x0, 200    x6 = 200 (overwrites x6)
        imem[ 8] = 32'h00530393; // PC=00400020: addi x7, x6, 5      x7 = 205 (Priority: newer x6=200 fwd)
        imem[ 9] = 32'h00a38013; // PC=00400024: addi x0, x7, 10     x0 = 0 (writes to x0 discarded)
        imem[10] = 32'h00100433; // PC=00400028: add  x8, x0, x1     x8 = 15 (x0 reads 0, no false fwd)
        imem[11] = 32'h00100493; // PC=0040002c: addi x9, x0, 1      x9 = 1 (imm=1 bitfield matches x1, no stall)
        imem[12] = 32'h00352023; // PC=00400030: sw   x3, 0(x10)     store 42 to dmem[0x10010000]
        imem[13] = 32'h00052583; // PC=00400034: lw   x11, 0(x10)    load 42 into x11
        imem[14] = 32'h00858613; // PC=00400038: addi x12, x11, 8    x12 = 50 (LOAD-USE HAZARD: 1-cycle stall!)
        imem[15] = 32'h00052683; // PC=0040003c: lw   x13, 0(x10)    load 42 into x13
        imem[16] = 32'h00d52223; // PC=00400040: sw   x13, 4(x10)    store x13 to dmem[4] (LOAD-STORE: MEM->MEM fwd!)
        imem[17] = 32'h00452703; // PC=00400044: lw   x14, 4(x10)    load back from dmem[4] -> x14 = 42
        imem[18] = 32'h05500793; // PC=00400048: addi x15, x0, 85    x15 = 0x55
        imem[19] = 32'h00f50423; // PC=0040004c: sb   x15, 8(x10)    store byte 0x55 to dmem[8]
        imem[20] = 32'h00850803; // PC=00400050: lb   x16, 8(x10)    load byte -> x16 = 0x55
        imem[21] = 32'h00208463; // PC=00400054: beq  x1, x2, +8     branch not taken (15 != 27) -> 0 bubbles!
        imem[22] = 32'h00100893; // PC=00400058: addi x17, x0, 1     x17 = 1 (continues seamlessly)
        imem[23] = 32'h00108663; // PC=0040005c: beq  x1, x1, +12    branch taken (15 == 15) -> flushes 2 ops!
        imem[24] = 32'h06300913; // PC=00400060: addi x18, x0, 99    (SKIPPED - Flushed in ID/EX)
        imem[25] = 32'h06300913; // PC=00400064: addi x18, x0, 99    (SKIPPED - Flushed in IF/ID)
        imem[26] = 32'h04d00993; // PC=00400068: addi x19, x0, 77    x19 = 77 (branch target)
        imem[27] = 32'h00c00a6f; // PC=0040006c: jal  x20, +12       jump to 0x78, link PC+4 (0x70) to x20
        imem[28] = 32'h06300a93; // PC=00400070: addi x21, x0, 99    (SKIPPED - Flushed)
        imem[29] = 32'h06300a93; // PC=00400074: addi x21, x0, 99    (SKIPPED - Flushed)
        imem[30] = 32'h000a0b13; // PC=00400078: addi x22, x20, 0    x22 = 0x00400070 (verifies link reg value)
        imem[31] = 32'h00000000; // PC=0040007c: illegal instruction TRAP: retires with trap=1, no rd write
        imem[32] = 32'h02100b93; // PC=00400080: addi x23, x0, 33    x23 = 33 (continues after trap at PC+4)
        imem[33] = 32'h00152c03; // PC=00400084: lw   x24, 1(x10)    MISALIGNED TRAP: no dmem read, no rd write
        imem[34] = 32'h02c00c93; // PC=00400088: addi x25, x0, 44    x25 = 44 (continues after trap at PC+4)
        imem[35] = 32'h00100073; // PC=0040008c: ebreak               HALT: retires with halt=1

        $display("================================================================================");
        $display("                Phase 4 Pipelined RV32I Processor Self-Checking Test            ");
        $display("================================================================================");

        // Reset the processor
        @(posedge clk);
        cycle = cycle + 1;
        @(negedge clk);
        rst = 1'b0;

        // ---------------------------------------------------------------------
        // Check 1: Pipeline Initialization & Fill Latency
        // ---------------------------------------------------------------------
        $display("\n--- 1. Pipeline Initialization & Fill Latency (Section 4.1 & 4.3) ---");
        // Instruction 0 (lui x10) is fetched at cycle 2, traverses IF->ID->EX->MEM->WB and
        // retires at cycle 5. The 3 cycles before it are initial fill bubbles.
        check_retire("lui  x10, 0x10010 (Init Base)", 32'h00400000, 32'h10010537, 5'd10, 32'h10010000, 32'h00400004, 1'b0, 1'b0, 3);
        check_retire("addi x1, x0, 15",               32'h00400004, 32'h00f00093, 5'd1,  32'd15,        32'h00400008, 1'b0, 1'b0, 0);
        check_retire("addi x2, x0, 27",               32'h00400008, 32'h01b00113, 5'd2,  32'd27,        32'h0040000c, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 2: RAW Hazards & Forwarding (EX->EX and MEM->EX)
        // ---------------------------------------------------------------------
        $display("\n--- 2. RAW Hazards & Forwarding Logic (Section 4.2.1 & 4.2.5) ---");
        // add x3 uses x2 produced immediately prior in EX/MEM -> EX->EX forwarding
        check_retire("add  x3, x1, x2 (EX->EX fwd x2)", 32'h0040000c, 32'h002081b3, 5'd3, 32'd42, 32'h00400010, 1'b0, 1'b0, 0);
        check_retire_rs("add x3 rs verification", 5'd1, 32'd15, 5'd2, 32'd27);

        // sub x4 uses x3 produced immediately prior in EX/MEM -> EX->EX forwarding
        check_retire("sub  x4, x3, x1 (EX->EX fwd x3)", 32'h00400010, 32'h40118233, 5'd4, 32'd27, 32'h00400014, 1'b0, 1'b0, 0);

        // and x5 uses x3 (from MEM/WB) and x4 (from EX/MEM) -> Dual simultaneous forwarding
        check_retire("and  x5, x3, x4 (Dual fwd)",      32'h00400014, 32'h0041f2b3, 5'd5, 32'd10, 32'h00400018, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 3: Forwarding Priority (EX/MEM > MEM/WB)
        // ---------------------------------------------------------------------
        $display("\n--- 3. Forwarding Priority Resolution ---");
        check_retire("addi x6, x0, 100",                32'h00400018, 32'h06400313, 5'd6, 32'd100, 32'h0040001c, 1'b0, 1'b0, 0);
        check_retire("addi x6, x0, 200 (overwrite)",    32'h0040001c, 32'h0c800313, 5'd6, 32'd200, 32'h00400020, 1'b0, 1'b0, 0);
        // addi x7 must take x6=200 from EX/MEM, NOT stale x6=100 from MEM/WB
        check_retire("addi x7, x6, 5 (Priority: 200)",  32'h00400020, 32'h00530393, 5'd7, 32'd205, 32'h00400024, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 4: False Dependency Immunity & Register x0 Discard
        // ---------------------------------------------------------------------
        $display("\n--- 4. False Dependency Immunity (Section 4.2.3) ---");
        // Writes to x0 must be silently discarded
        check_retire("addi x0, x7, 10 (Discard x0)",    32'h00400024, 32'h00a38013, 5'd0, 32'd0,   32'h00400028, 1'b0, 1'b0, 0);
        // Reading x0 must yield 0, not forwarded value
        check_retire("add  x8, x0, x1 (Read x0=0)",     32'h00400028, 32'h00100433, 5'd8, 32'd15,  32'h0040002c, 1'b0, 1'b0, 0);
        // Immediate value 1 has bitfield matching x1, must NOT cause false stall
        check_retire("addi x9, x0, 1 (Imm bit overlap)",32'h0040002c, 32'h00100493, 5'd9, 32'd1,   32'h00400030, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 5: Load-Use Hazard & Stall Insertion
        // ---------------------------------------------------------------------
        $display("\n--- 5. Load-Use Hazard Detection & Stall (Section 4.2.1 & 4.2.4) ---");
        check_retire("sw   x3, 0(x10)",                 32'h00400030, 32'h00352023, 5'd0,  32'd0,   32'h00400034, 1'b0, 1'b0, 0);
        check_retire_dmem("sw x3 dmem check", 1'b0, 1'b1, 32'h10010000, 4'b1111);

        check_retire("lw   x11, 0(x10) (Load 42)",      32'h00400034, 32'h00052583, 5'd11, 32'd42,  32'h00400038, 1'b0, 1'b0, 0);
        check_retire_dmem("lw x11 dmem check", 1'b1, 1'b0, 32'h10010000, 4'b1111);

        // addi x12 immediately uses x11: CPU MUST detect load-use and stall 1 cycle!
        check_retire("addi x12, x11, 8 (Stall 1 cycle)",32'h00400038, 32'h00858613, 5'd12, 32'd50,  32'h0040003c, 1'b0, 1'b0, 1);

        // ---------------------------------------------------------------------
        // Check 6: Load-Store Hazard & MEM->MEM Store Forwarding
        // ---------------------------------------------------------------------
        $display("\n--- 6. Load-Store Hazard & MEM->MEM Forwarding (Section 4.2.5) ---");
        check_retire("lw   x13, 0(x10) (Load 42)",      32'h0040003c, 32'h00052683, 5'd13, 32'd42,  32'h00400040, 1'b0, 1'b0, 0);
        // sw x13 immediately uses x13 as store data: stalls 1 cycle and forwards MEM->MEM
        check_retire("sw   x13, 4(x10) (MEM->MEM fwd)", 32'h00400040, 32'h00d52223, 5'd0,  32'd0,   32'h00400044, 1'b0, 1'b0, 1);
        check_retire_dmem("sw x13 dmem check", 1'b0, 1'b1, 32'h10010004, 4'b1111);
        check_retire_rs("sw x13 rs check", 5'd10, 32'h10010000, 5'd13, 32'd42);

        // Read stored value back
        check_retire("lw   x14, 4(x10) (Read back 42)", 32'h00400044, 32'h00452703, 5'd14, 32'd42,  32'h00400048, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 7: Sub-Word Memory Accesses (SB & LB)
        // ---------------------------------------------------------------------
        $display("\n--- 7. Sub-Word Memory Accesses (SB & LB) (Section 4.5) ---");
        check_retire("addi x15, x0, 85 (0x55)",         32'h00400048, 32'h05500793, 5'd15, 32'd85,  32'h0040004c, 1'b0, 1'b0, 0);
        check_retire("sb   x15, 8(x10) (Store byte)",   32'h0040004c, 32'h00f50423, 5'd0,  32'd0,   32'h00400050, 1'b0, 1'b0, 0);
        check_retire_dmem("sb x15 dmem check", 1'b0, 1'b1, 32'h10010008, 4'b0001);

        check_retire("lb   x16, 8(x10) (Load byte)",    32'h00400050, 32'h00850803, 5'd16, 32'd85,  32'h00400054, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 8: Control Hazards: Branch Not-Taken (0 Bubbles)
        // ---------------------------------------------------------------------
        $display("\n--- 8. Control Hazards: Branch Not-Taken (Section 4.2.2) ---");
        // Branch not taken: 15 != 27 -> must NOT flush, 0 bubbles!
        check_retire("beq  x1, x2 (Not-Taken)",         32'h00400054, 32'h00208463, 5'd0,  32'd0,   32'h00400058, 1'b0, 1'b0, 0);
        check_retire("addi x17, x0, 1 (Seamless)",      32'h00400058, 32'h00100893, 5'd17, 32'd1,   32'h0040005c, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 9: Control Hazards: Branch Taken (2 Flushes, Exactly 2 Bubbles)
        // ---------------------------------------------------------------------
        $display("\n--- 9. Control Hazards: Branch Taken & Flushing (Section 4.1.3 & 4.2.2) ---");
        // Branch taken: 15 == 15 -> resolved in EX stage, flushes IF/ID and ID/EX!
        check_retire("beq  x1, x1 (Taken -> 0x68)",     32'h0040005c, 32'h00108663, 5'd0,  32'd0,   32'h00400068, 1'b0, 1'b0, 0);
        // Exactly 2 bubbles must be observed before target instruction retires!
        check_retire("addi x19, x0, 77 (Target hit)",   32'h00400068, 32'h04d00993, 5'd19, 32'd77,  32'h0040006c, 1'b0, 1'b0, 2);

        // ---------------------------------------------------------------------
        // Check 10: Control Hazards: Unconditional Jump (JAL) & Return Link
        // ---------------------------------------------------------------------
        $display("\n--- 10. Unconditional Jump (JAL) & Link Register ---");
        // JAL: jumps to 0x78, flushes 2 instructions, writes return address (0x70) to x20
        check_retire("jal  x20, +12 (Link=0x70)",       32'h0040006c, 32'h00c00a6f, 5'd20, 32'h00400070, 32'h00400078, 1'b0, 1'b0, 0);
        // Exactly 2 bubbles, then target executes with forwarded link value
        check_retire("addi x22, x20, 0 (Link verified)",32'h00400078, 32'h000a0b13, 5'd22, 32'h00400070, 32'h0040007c, 1'b0, 1'b0, 2);

        // ---------------------------------------------------------------------
        // Check 11: Traps: Illegal Instruction Encoding
        // ---------------------------------------------------------------------
        $display("\n--- 11. Traps: Illegal Instruction (Section 4.5.1) ---");
        // Illegal instruction: retires with trap=1, no rd write, rs1/rs2 raddr=0, continues at PC+4
        check_retire("illegal instruction trap",        32'h0040007c, 32'h00000000, 5'd0,  32'd0,   32'h00400080, 1'b0, 1'b1, 0);
        check_retire_rs("illegal inst rsaddr=0 check", 5'd0, 32'd0, 5'd0, 32'd0);
        // Execution continues normally at PC+4
        check_retire("addi x23, x0, 33 (Post-trap)",    32'h00400080, 32'h02100b93, 5'd23, 32'd33,  32'h00400084, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 12: Traps: Misaligned Memory Access
        // ---------------------------------------------------------------------
        $display("\n--- 12. Traps: Misaligned Memory Access (Section 4.5.1) ---");
        // Misaligned word load: address 0x10010001 (not 4-byte aligned)
        // Must retire with trap=1, NO memory access, NO register write, continues at PC+4
        check_retire("misaligned lw trap",              32'h00400084, 32'h00152c03, 5'd0,  32'd0,   32'h00400088, 1'b0, 1'b1, 0);
        check_retire_dmem("misaligned lw dmem check", 1'b0, 1'b0, 32'h10010000, 4'b0000);
        // Execution continues normally at PC+4
        check_retire("addi x25, x0, 44 (Post-trap)",    32'h00400088, 32'h02c00c93, 5'd25, 32'd44,  32'h0040008c, 1'b0, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Check 13: Halt Instruction (ebreak)
        // ---------------------------------------------------------------------
        $display("\n--- 13. System Halt Instruction (Section 4.5) ---");
        check_retire("ebreak (Halt)",                   32'h0040008c, 32'h00100073, 5'd0,  32'd0,   32'h00400090, 1'b1, 1'b0, 0);

        // ---------------------------------------------------------------------
        // Final Summary
        // ---------------------------------------------------------------------
        $display("\n================================================================================");
        $display("Directed Test Summary: %0d passed, %0d failed across %0d simulated clock cycles",
                 passed, failed, cycle);
        if (failed == 0) begin
            $display(">>> ALL DIRECTED CHECKS PASSED SUCCESSFULLY! <<<");
        end else begin
            $display(">>> TEST SUITE FAILED (%0d ERRORS) <<<", failed);
        end
        $display("================================================================================\n");

        $finish;
    end

endmodule

`default_nettype wire
