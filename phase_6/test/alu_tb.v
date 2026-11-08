`timescale 1ns / 1ps
`default_nettype none

module alu_tb;

    reg         clk;
    reg         rst;
    reg  [ 2:0] opsel;
    reg         sub;
    reg         is_unsigned;
    reg         arith;
    reg         mext;
    reg         advance;
    reg  [31:0] op1;
    reg  [31:0] op2;

    wire [31:0] result;
    wire        eq;
    wire        slt;
    wire        busy;

    // Instantiate Unit Under Test (UUT)
    alu dut (
        .i_clk      (clk),
        .i_rst      (rst),
        .i_opsel    (opsel),
        .i_sub      (sub),
        .i_unsigned (is_unsigned),
        .i_arith    (arith),
        .i_mext     (mext),
        .i_advance  (advance),
        .i_op1      (op1),
        .i_op2      (op2),
        .o_result   (result),
        .o_eq       (eq),
        .o_slt      (slt),
        .o_busy     (busy)
    );

    // Clock generator (10ns period)
    always #5 clk = ~clk;

    integer pass_count = 0;
    integer fail_count = 0;

    task check_comb(
        input [31:0] exp_res,
        input        exp_eq,
        input        exp_slt,
        input [127:0] test_name
    );
        begin
            #1; // Wait for combinational settling
            if (result === exp_res && eq === exp_eq && slt === exp_slt && busy === 1'b0) begin
                $display("[PASS] %0s -> res: 0x%08h, eq: %b, slt: %b", test_name, result, eq, slt);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s -> Got res: 0x%08h (exp: 0x%08h), eq: %b (%b), slt: %b (%b), busy: %b (0)",
                         test_name, result, exp_res, eq, exp_eq, slt, exp_slt, busy);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check_m_mul(
        input [ 2:0] funct3,
        input [31:0] in1,
        input [31:0] in2,
        input [31:0] exp_res,
        input [127:0] test_name
    );
        begin
            @(posedge clk);
            mext    <= 1'b1;
            opsel   <= funct3;
            sub     <= 1'b0;
            arith   <= 1'b0;
            is_unsigned <= 1'b0;
            op1     <= in1;
            op2     <= in2;
            advance <= 1'b1;
            @(posedge clk);
            #1;
            if (result === exp_res) begin
                $display("[PASS] MUL %0s -> res: 0x%08h", test_name, result);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] MUL %0s -> Got res: 0x%08h (exp: 0x%08h)", test_name, result, exp_res);
                fail_count = fail_count + 1;
            end
            mext <= 1'b0;
            advance <= 1'b0;
        end
    endtask

    task check_m_div(
        input [ 2:0] funct3,
        input [31:0] in1,
        input [31:0] in2,
        input [31:0] exp_res,
        input [127:0] test_name
    );
        integer cycles;
        begin
            @(posedge clk);
            mext    <= 1'b1;
            opsel   <= funct3;
            sub     <= 1'b0;
            arith   <= 1'b0;
            is_unsigned <= 1'b0;
            op1     <= in1;
            op2     <= in2;
            advance <= 1'b0;

            // Wait for divider to assert busy and then deassert busy
            @(posedge clk);
            cycles = 0;
            while (busy && cycles < 100) begin
                @(posedge clk);
                cycles = cycles + 1;
            end

            #1;
            // Now result must be ready and valid
            if (result === exp_res) begin
                $display("[PASS] DIV %0s in %0d cycles -> res: 0x%08h", test_name, cycles, result);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] DIV %0s -> Got res: 0x%08h (exp: 0x%08h)", test_name, result, exp_res);
                fail_count = fail_count + 1;
            end

            // Advance to complete handshake
            advance <= 1'b1;
            @(posedge clk);
            advance <= 1'b0;
            mext    <= 1'b0;
        end
    endtask

    initial begin
        clk = 0;
        rst = 1;
        opsel = 0;
        sub = 0;
        is_unsigned = 0;
        arith = 0;
        mext = 0;
        advance = 0;
        op1 = 0;
        op2 = 0;

        #20;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        $display("==================================================");
        $display("        TESTING RV32I BASE ALU OPERATIONS        ");
        $display("==================================================");
        // ADD
        opsel = 3'b000; sub = 0; op1 = 32'd15; op2 = 32'd27;
        check_comb(32'd42, 1'b0, 1'b0, "ADD 15 + 27");

        // SUB
        opsel = 3'b000; sub = 1; op1 = 32'd50; op2 = 32'd20;
        check_comb(32'd30, 1'b0, 1'b0, "SUB 50 - 20");

        // SLL
        opsel = 3'b001; op1 = 32'h00000001; op2 = 32'd4;
        check_comb(32'h00000010, 1'b0, 1'b0, "SLL 1 << 4");

        // SLT signed
        opsel = 3'b010; op1 = -32'd5; op2 = 32'd2;
        check_comb(32'd1, 1'b0, 1'b1, "SLT -5 < 2");

        // SLTU unsigned
        opsel = 3'b011; op1 = 32'd2; op2 = -32'd5;
        check_comb(32'd1, 1'b0, 1'b1, "SLTU 2 < 0xFFFFFFFB");

        // XOR, OR, AND
        opsel = 3'b100; op1 = 32'hAAAA5555; op2 = 32'hFFFF0000;
        check_comb(32'h55555555, 1'b0, 1'b0, "XOR");

        opsel = 3'b110; op1 = 32'h12340000; op2 = 32'h00005678;
        check_comb(32'h12345678, 1'b0, 1'b0, "OR");

        opsel = 3'b111; op1 = 32'hFFFF00FF; op2 = 32'h0FF0FF00;
        check_comb(32'h0FF00000, 1'b0, 1'b0, "AND");

        $display("\n==================================================");
        $display("          TESTING RV32M MULTIPLICATION           ");
        $display("==================================================");
        // mul (funct3 = 000)
        check_m_mul(3'b000, 32'd11, 32'd13, 32'd143, "mul 11 * 13");
        check_m_mul(3'b000, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'h00000001, "mul -1 * -1");
        check_m_mul(3'b000, 32'd100000, 32'd500000, 32'd50000000000 % (2**32), "mul large");

        // mulh (funct3 = 001) signed * signed
        check_m_mul(3'b001, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'h00000000, "mulh -1 * -1");
        check_m_mul(3'b001, 32'hFFFFFFFF, 32'd5, 32'hFFFFFFFF, "mulh -1 * 5");
        check_m_mul(3'b001, 32'h80000000, 32'd2, 32'hFFFFFFFF, "mulh -2^31 * 2");

        // mulhsu (funct3 = 010) signed * unsigned
        check_m_mul(3'b010, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'hFFFFFFFF, "mulhsu -1 * 0xFFFFFFFF");
        check_m_mul(3'b010, 32'd5, 32'hFFFFFFFF, 32'd0, "mulhsu 5 * 0xFFFFFFFF");

        // mulhu (funct3 = 011) unsigned * unsigned
        check_m_mul(3'b011, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'hFFFFFFFE, "mulhu 0xFFFFFFFF * 0xFFFFFFFF");
        check_m_mul(3'b011, 32'h80000000, 32'h80000000, 32'h40000000, "mulhu 2^31 * 2^31");

        $display("\n==================================================");
        $display("             TESTING RV32M DIVISION              ");
        $display("==================================================");
        // div (funct3 = 100) signed rs1 / rs2
        check_m_div(3'b100, 32'd13, 32'd3, 32'd4, "div 13 / 3");
        check_m_div(3'b100, 32'd7, -32'd3, -32'd2, "div 7 / -3");
        check_m_div(3'b100, -32'd7, 32'd3, -32'd2, "div -7 / 3");
        check_m_div(3'b100, -32'd7, -32'd3, 32'd2, "div -7 / -3");

        // div special corner cases:
        // Divide by zero: quotient is -1 (0xFFFFFFFF)
        check_m_div(3'b100, 32'd25, 32'd0, 32'hFFFFFFFF, "div 25 / 0 (div-by-zero)");
        check_m_div(3'b100, -32'd25, 32'd0, 32'hFFFFFFFF, "div -25 / 0 (div-by-zero)");
        // Overflow: -2^31 / -1 -> -2^31 (0x80000000)
        check_m_div(3'b100, 32'h80000000, 32'hFFFFFFFF, 32'h80000000, "div -2^31 / -1 (overflow)");

        // divu (funct3 = 101) unsigned rs1 / rs2
        check_m_div(3'b101, 32'd13, 32'd3, 32'd4, "divu 13 / 3");
        check_m_div(3'b101, 32'hFFFFFFFF, 32'd2, 32'h7FFFFFFF, "divu 0xFFFFFFFF / 2");
        check_m_div(3'b101, 32'd50, 32'd0, 32'hFFFFFFFF, "divu 50 / 0 (div-by-zero)");

        // rem (funct3 = 110) signed remainder
        check_m_div(3'b110, 32'd7, -32'd3, 32'd1, "rem 7 % -3 (takes sign of dividend)");
        check_m_div(3'b110, -32'd7, 32'd3, -32'd1, "rem -7 % 3 (takes sign of dividend)");
        check_m_div(3'b110, -32'd7, -32'd3, -32'd1, "rem -7 % -3");
        check_m_div(3'b110, 32'd25, 32'd0, 32'd25, "rem 25 % 0 (returns dividend)");
        check_m_div(3'b110, -32'd25, 32'd0, -32'd25, "rem -25 % 0 (returns dividend)");
        check_m_div(3'b110, 32'h80000000, 32'hFFFFFFFF, 32'd0, "rem -2^31 % -1 (overflow returns 0)");

        // remu (funct3 = 111) unsigned remainder
        check_m_div(3'b111, 32'd13, 32'd3, 32'd1, "remu 13 % 3");
        check_m_div(3'b111, 32'hFFFFFFFF, 32'd10, 32'd5, "remu 0xFFFFFFFF % 10");
        check_m_div(3'b111, 32'd42, 32'd0, 32'd42, "remu 42 % 0 (returns dividend)");

        $display("\n==================================================");
        $display("   TESTING MULTI-CYCLE RESULT HOLD & i_advance    ");
        $display("==================================================");
        // Test that result holds indefinitely until i_advance goes high
        @(posedge clk);
        mext    <= 1'b1;
        opsel   <= 3'b100; // div
        op1     <= 32'd100;
        op2     <= 32'd10;
        advance <= 1'b0;

        @(posedge clk);
        while (busy) @(posedge clk);

        // Keep advance low for 5 cycles, verify result does not corrupt
        repeat(5) begin
            @(posedge clk);
            #1;
            if (result !== 32'd10 || busy !== 1'b0) begin
                $display("[FAIL] Hold failed! result: 0x%08h, busy: %b", result, busy);
                fail_count = fail_count + 1;
            end
        end
        $display("[PASS] Multi-cycle divider held result 0x%08h across 5 frozen cycles", result);
        pass_count = pass_count + 1;

        advance <= 1'b1;
        @(posedge clk);
        advance <= 1'b0;
        mext    <= 1'b0;

        $display("\n==================================================");
        $display("                 SUMMARY REPORT                   ");
        $display("==================================================");
        $display("Total Passed: %0d", pass_count);
        $display("Total Failed: %0d", fail_count);
        if (fail_count == 0) begin
            $display(">>> ALL ALU & M-EXTENSION TESTS PASSED! <<<");
        end else begin
            $display(">>> SOME TESTS FAILED! CHECK LOGS ABOVE. <<<");
        end
        $finish;
    end

endmodule
`default_nettype wire
