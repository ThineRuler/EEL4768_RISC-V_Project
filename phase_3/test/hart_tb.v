`timescale 1ns / 1ps
`default_nettype none

// ============================================================================
// Module: hart_tb
// Description: Self-checking testbench for single-cycle RV32I CPU (hart.v).
//              Modeled after phase_2/example/opmux_tb.v.
//
// To compile and run:
//   iverilog -s hart_tb -o sim test/hart_tb.v test/hart.v test/alu.v test/imm.v test/rf.v test/decoder.v
//   vvp sim
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
    wire [31:0] retire_rs1_rdata;
    wire [ 4:0] retire_rs2_raddr;
    wire [31:0] retire_rs2_rdata;
    wire [ 4:0] retire_rd_waddr;
    wire [31:0] retire_rd_wdata;
    wire [31:0] retire_pc;
    wire [31:0] retire_next_pc;

    integer passed;
    integer failed;
    integer cycle;

    // -------------------------------------------------------------------------
    // 2. Device Under Test (DUT)
    // -------------------------------------------------------------------------
    hart #(.RESET_ADDR(32'h00000000)) dut (
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
        .o_retire_rs1_rdata (retire_rs1_rdata),
        .o_retire_rs2_raddr (retire_rs2_raddr),
        .o_retire_rs2_rdata (retire_rs2_rdata),
        .o_retire_rd_waddr  (retire_rd_waddr),
        .o_retire_rd_wdata  (retire_rd_wdata),
        .o_retire_pc        (retire_pc),
        .o_retire_next_pc   (retire_next_pc)
    );

    // -------------------------------------------------------------------------
    // 3. Simulated Memories
    // -------------------------------------------------------------------------
    // Instruction memory: combinational read
    localparam IMEM_SIZE = 1024;
    reg [31:0] imem [0:IMEM_SIZE-1];
    assign imem_rdata = (imem_raddr[31:2] < IMEM_SIZE) ? imem[imem_raddr[31:2]] : 32'h00000013; // default nop

    // Data memory: combinational read, byte-lane-masked synchronous write
    localparam DMEM_SIZE = 1024;
    reg [31:0] dmem [0:DMEM_SIZE-1];
    assign dmem_rdata = (dmem_addr[31:2] < DMEM_SIZE) ? dmem[dmem_addr[31:2]] : 32'h0;

    always @(posedge clk) begin
        if (dmem_wen && (dmem_addr[31:2] < DMEM_SIZE)) begin
            if (dmem_mask[0]) dmem[dmem_addr[31:2]][ 7: 0] <= dmem_wdata[ 7: 0];
            if (dmem_mask[1]) dmem[dmem_addr[31:2]][15: 8] <= dmem_wdata[15: 8];
            if (dmem_mask[2]) dmem[dmem_addr[31:2]][23:16] <= dmem_wdata[23:16];
            if (dmem_mask[3]) dmem[dmem_addr[31:2]][31:24] <= dmem_wdata[31:24];
        end
    end

    // Clock: 10ns period (100 MHz)
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // 4. Verification Check Task
    // -------------------------------------------------------------------------
    task check_step;
        input [511:0] test_name;
        input [ 31:0] exp_pc;
        input [ 31:0] exp_inst;
        input [  4:0] exp_rd;
        input [ 31:0] exp_rd_val;
        input [ 31:0] exp_next_pc;
        input         exp_halt;
        input         exp_trap;

        reg ok;
        begin
            // Sample midway through the clock cycle when all signals are stable
            @(negedge clk);
            cycle = cycle + 1;
            ok = 1'b1;

            if (retire_valid !== 1'b1) begin
                $display("[FAIL] cycle %0d: %0s -> retire_valid is not 1", cycle, test_name);
                ok = 1'b0;
            end
            if (retire_pc !== exp_pc) begin
                $display("[FAIL] cycle %0d: %0s -> PC mismatch: got %08h, exp %08h", cycle, test_name, retire_pc, exp_pc);
                ok = 1'b0;
            end
            if (retire_inst !== exp_inst) begin
                $display("[FAIL] cycle %0d: %0s -> inst mismatch: got %08h, exp %08h", cycle, test_name, retire_inst, exp_inst);
                ok = 1'b0;
            end
            if (retire_rd_waddr !== exp_rd) begin
                $display("[FAIL] cycle %0d: %0s -> rd_waddr mismatch: got %0d, exp %0d", cycle, test_name, retire_rd_waddr, exp_rd);
                ok = 1'b0;
            end
            if (exp_rd != 5'd0 && retire_rd_wdata !== exp_rd_val) begin
                $display("[FAIL] cycle %0d: %0s -> rd_wdata mismatch: got %08h, exp %08h", cycle, test_name, retire_rd_wdata, exp_rd_val);
                ok = 1'b0;
            end
            if (retire_next_pc !== exp_next_pc) begin
                $display("[FAIL] cycle %0d: %0s -> next_pc mismatch: got %08h, exp %08h", cycle, test_name, retire_next_pc, exp_next_pc);
                ok = 1'b0;
            end
            if (retire_halt !== exp_halt) begin
                $display("[FAIL] cycle %0d: %0s -> halt mismatch: got %b, exp %b", cycle, test_name, retire_halt, exp_halt);
                ok = 1'b0;
            end
            if (retire_trap !== exp_trap) begin
                $display("[FAIL] cycle %0d: %0s -> trap mismatch: got %b, exp %b", cycle, test_name, retire_trap, exp_trap);
                ok = 1'b0;
            end

            if (ok) begin
                passed = passed + 1;
                $display("[PASS] %0s (PC=%08h)", test_name, exp_pc);
            end else begin
                failed = failed + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // 5. Test Program Execution
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

        for (i = 0; i < IMEM_SIZE; i = i + 1) imem[i] = 32'h00000013; // nop (addi x0, x0, 0)
        for (i = 0; i < DMEM_SIZE; i = i + 1) dmem[i] = 32'h00000000;

        // Load a test program into instruction memory
        // Address  Instruction          Assembly
        imem[0]  = 32'h00f00093;      // addi x1, x0, 15
        imem[1]  = 32'h01b00113;      // addi x2, x0, 27
        imem[2]  = 32'h002081b3;      // add  x3, x1, x2        (15 + 27 = 42)
        imem[3]  = 32'h40118233;      // sub  x4, x3, x1        (42 - 15 = 27)
        imem[4]  = 32'h0020f2b3;      // and  x5, x1, x2        (15 & 27 = 11)
        imem[5]  = 32'h0020e333;      // or   x6, x1, x2        (15 | 27 = 31)
        imem[6]  = 32'h0020c3b3;      // xor  x7, x1, x2        (15 ^ 27 = 20)
        imem[7]  = 32'h00209413;      // slli x8, x1, 2         (15 << 2 = 60)
        imem[8]  = 32'h00245493;      // srli x9, x8, 2         (60 >> 2 = 15)
        imem[9]  = 32'h12345537;      // lui  x10, 20'h12345    (0x12345000)
        imem[10] = 32'h00302223;      // sw   x3, 4(x0)         (store 42 to dmem[4])
        imem[11] = 32'h00402583;      // lw   x11, 4(x0)        (load dmem[4] -> x11 = 42)
        imem[12] = 32'h00100423;      // sb   x1, 8(x0)         (store byte 15 to dmem[8])
        imem[13] = 32'h00800603;      // lb   x12, 8(x0)        (load byte -> x12 = 15)
        imem[14] = 32'h00208463;      // beq  x1, x2, +8        (not taken: 15 != 27)
        imem[15] = 32'h00209463;      // bne  x1, x2, +8        (taken: 15 != 27 -> jumps to PC + 8 = 0x44)
        imem[16] = 32'h06300713;      // addi x14, x0, 99       (SKIPPED)
        imem[17] = 32'h008007ef;      // jal  x15, +8           (jumps to PC + 8 = 0x4c, x15 = 0x48)
        imem[18] = 32'h06300813;      // addi x16, x0, 99       (SKIPPED)
        imem[19] = 32'h00100073;      // ebreak                 (halt execution)

        $display("========================================");
        $display("       hart.v Self-Checking Testbench   ");
        $display("========================================");

        // Apply reset for 2 cycles
        @(posedge clk);
        @(posedge clk);
        rst = 1'b0;

        // Step through program and verify each retired instruction
        $display("--- Testing Arithmetic & Logical Ops ---");
        check_step("addi x1, x0, 15",  32'h00, 32'h00f00093, 5'd1,  32'd15,        32'h04, 1'b0, 1'b0);
        check_step("addi x2, x0, 27",  32'h04, 32'h01b00113, 5'd2,  32'd27,        32'h08, 1'b0, 1'b0);
        check_step("add  x3, x1, x2",  32'h08, 32'h002081b3, 5'd3,  32'd42,        32'h0c, 1'b0, 1'b0);
        check_step("sub  x4, x3, x1",  32'h0c, 32'h40118233, 5'd4,  32'd27,        32'h10, 1'b0, 1'b0);
        check_step("and  x5, x1, x2",  32'h10, 32'h0020f2b3, 5'd5,  32'd11,        32'h14, 1'b0, 1'b0);
        check_step("or   x6, x1, x2",  32'h14, 32'h0020e333, 5'd6,  32'd31,        32'h18, 1'b0, 1'b0);
        check_step("xor  x7, x1, x2",  32'h18, 32'h0020c3b3, 5'd7,  32'd20,        32'h1c, 1'b0, 1'b0);
        check_step("slli x8, x1, 2",   32'h1c, 32'h00209413, 5'd8,  32'd60,        32'h20, 1'b0, 1'b0);
        check_step("srli x9, x8, 2",   32'h20, 32'h00245493, 5'd9,  32'd15,        32'h24, 1'b0, 1'b0);
        check_step("lui  x10, 0x12345",32'h24, 32'h12345537, 5'd10, 32'h12345000,  32'h28, 1'b0, 1'b0);

        $display("--- Testing Data Memory (Store & Load) ---");
        check_step("sw   x3, 4(x0)",   32'h28, 32'h00302223, 5'd0,  32'd0,         32'h2c, 1'b0, 1'b0);
        check_step("lw   x11, 4(x0)",  32'h2c, 32'h00402583, 5'd11, 32'd42,        32'h30, 1'b0, 1'b0);
        check_step("sb   x1, 8(x0)",   32'h30, 32'h00100423, 5'd0,  32'd0,         32'h34, 1'b0, 1'b0);
        check_step("lb   x12, 8(x0)",  32'h34, 32'h00800603, 5'd12, 32'd15,        32'h38, 1'b0, 1'b0);

        $display("--- Testing Control Flow (Branch & Jump) ---");
        check_step("beq  x1, x2 (NT)", 32'h38, 32'h00208463, 5'd0,  32'd0,         32'h3c, 1'b0, 1'b0);
        check_step("bne  x1, x2 (T)",  32'h3c, 32'h00209463, 5'd0,  32'd0,         32'h44, 1'b0, 1'b0);
        check_step("jal  x15 (link)",  32'h44, 32'h008007ef, 5'd15, 32'h48,        32'h4c, 1'b0, 1'b0);

        $display("--- Testing System Instruction ---");
        check_step("ebreak (halt)",    32'h4c, 32'h00100073, 5'd0,  32'd0,         32'h50, 1'b1, 1'b0);

        // Final Summary
        $display("========================================");
        $display("Test Summary: %0d passed, %0d failed", passed, failed);
        if (failed == 0) begin
            $display("ALL TESTS PASSED");
        end else begin
            $display("TEST FAILED");
        end
        $display("========================================");

        $finish;
    end

endmodule

`default_nettype wire
