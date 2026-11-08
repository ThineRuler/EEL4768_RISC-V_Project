`timescale 1ns / 1ps
`default_nettype none

module fpu_tb;

    reg         clk;
    reg         rst;
    reg  [31:0] inst;
    reg         valid;
    reg         advance;
    reg  [31:0] frs1;
    reg  [31:0] frs2;
    reg  [31:0] frs3;
    reg  [31:0] xrs1;

    wire [31:0] fp_result;
    wire [31:0] int_result;
    wire [ 4:0] fflags;
    wire        busy;
    wire        rm_trap;

    // Instantiate FPU
    fpu dut (
        .i_clk        (clk),
        .i_rst        (rst),
        .i_inst       (inst),
        .i_valid      (valid),
        .i_advance    (advance),
        .i_frs1       (frs1),
        .i_frs2       (frs2),
        .i_frs3       (frs3),
        .i_xrs1       (xrs1),
        .o_fp_result  (fp_result),
        .o_int_result (int_result),
        .o_fflags     (fflags),
        .o_busy       (busy),
        .o_rm_trap    (rm_trap)
    );

    always #5 clk = ~clk;

    integer pass_count = 0;
    integer fail_count = 0;

    // IEEE 754 Constants for Testing
    localparam [31:0] FP_PLUS_ZERO   = 32'h00000000;
    localparam [31:0] FP_MINUS_ZERO  = 32'h80000000;
    localparam [31:0] FP_ONE         = 32'h3F800000; // 1.0
    localparam [31:0] FP_TWO         = 32'h40000000; // 2.0
    localparam [31:0] FP_THREE       = 32'h40400000; // 3.0
    localparam [31:0] FP_FOUR        = 32'h40800000; // 4.0
    localparam [31:0] FP_SIX         = 32'h40C00000; // 6.0
    localparam [31:0] FP_TEN         = 32'h41200000; // 10.0
    localparam [31:0] FP_NEG_SIX_PT_25 = 32'hC0C80000; // -6.25
    localparam [31:0] FP_PLUS_INF    = 32'h7F800000; // +inf
    localparam [31:0] FP_MINUS_INF   = 32'hFF800000; // -inf
    localparam [31:0] FP_CANON_NAN   = 32'h7FC00000; // Canonical NaN
    localparam [31:0] FP_SNAN        = 32'h7F800001; // Signaling NaN
    localparam [31:0] FP_POS_SUBNORM = 32'h00000001; // Smallest positive subnormal
    localparam [31:0] FP_NEG_SUBNORM = 32'h80000001; // Smallest negative subnormal

    task step_inst(
        input [31:0] instruction,
        input [31:0] in_frs1,
        input [31:0] in_frs2,
        input [31:0] in_frs3,
        input [31:0] in_xrs1
    );
        begin
            @(posedge clk);
            inst    <= instruction;
            valid   <= 1'b1;
            advance <= 1'b0;
            frs1    <= in_frs1;
            frs2    <= in_frs2;
            frs3    <= in_frs3;
            xrs1    <= in_xrs1;
            @(posedge clk);
            while (busy) @(posedge clk);
            #1; // Output valid
        end
    endtask

    task commit_inst;
        begin
            advance <= 1'b1;
            @(posedge clk);
            advance <= 1'b0;
            valid   <= 1'b0;
            #1;
        end
    endtask

    task check_fp(
        input [31:0] exp_res,
        input [ 4:0] exp_flags,
        input [127:0] name
    );
        begin
            if (fp_result === exp_res && fflags === exp_flags) begin
                $display("[PASS] FP: %0s -> res: 0x%08h, flags: %05b", name, fp_result, fflags);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] FP: %0s -> Got res: 0x%08h (exp: 0x%08h), flags: %05b (exp: %05b)",
                         name, fp_result, exp_res, fflags, exp_flags);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check_int(
        input [31:0] exp_res,
        input [ 4:0] exp_flags,
        input [127:0] name
    );
        begin
            if (int_result === exp_res && fflags === exp_flags) begin
                $display("[PASS] INT: %0s -> res: 0x%08h, flags: %05b", name, int_result, fflags);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] INT: %0s -> Got res: 0x%08h (exp: 0x%08h), flags: %05b (exp: %05b)",
                         name, int_result, exp_res, fflags, exp_flags);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        clk = 0;
        rst = 1;
        inst = 0;
        valid = 0;
        advance = 0;
        frs1 = 0;
        frs2 = 0;
        frs3 = 0;
        xrs1 = 0;

        #20;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        $display("==================================================");
        $display("          TESTING FP MOVES & SIGN INJECTION       ");
        $display("==================================================");
        // fmv.x.w x5, f1 (bits copied directly, no flags)
        step_inst(32'b1110000_00000_00001_000_00101_1010011, FP_ONE, 32'd0, 32'd0, 32'd0);
        check_int(FP_ONE, 5'b00000, "fmv.x.w 1.0 -> 0x3F800000");
        commit_inst();

        // fmv.w.x f2, x8 (bits copied directly)
        step_inst(32'b1111000_00000_01000_000_00010_1010011, 32'd0, 32'd0, 32'd0, FP_NEG_SIX_PT_25);
        check_fp(FP_NEG_SIX_PT_25, 5'b00000, "fmv.w.x -6.25 -> 0xC0C80000");
        commit_inst();

        // fsgnj.s f3, f1, f2 (sign of f2)
        step_inst(32'b0010000_00010_00001_000_00011_1010011, 32'h40E00000, FP_MINUS_ZERO, 32'd0, 32'd0);
        check_fp(32'hC0E00000, 5'b00000, "fsgnj 7.0 with sign(-) -> -7.0");
        commit_inst();

        // fsgnjx.s f3, f1, f1 (fabs: sign XOR sign = 0)
        step_inst(32'b0010000_00001_00001_010_00011_1010011, FP_NEG_SIX_PT_25, FP_NEG_SIX_PT_25, 32'd0, 32'd0);
        check_fp(32'h40C80000, 5'b00000, "fsgnjx.s fabs(-6.25) -> +6.25");
        commit_inst();

        $display("\n==================================================");
        $display("             TESTING FP CLASSIFY & COMPARE        ");
        $display("==================================================");
        // fclass.s: test -inf -> bit 0 (0x001)
        step_inst(32'b1110000_00000_00001_001_00101_1010011, FP_MINUS_INF, 32'd0, 32'd0, 32'd0);
        check_int(32'h001, 5'b00000, "fclass -inf");
        commit_inst();

        // fclass.s: test -0 -> bit 3 (0x008)
        step_inst(32'b1110000_00000_00001_001_00101_1010011, FP_MINUS_ZERO, 32'd0, 32'd0, 32'd0);
        check_int(32'h008, 5'b00000, "fclass -0");
        commit_inst();

        // fclass.s: test +0 -> bit 4 (0x010)
        step_inst(32'b1110000_00000_00001_001_00101_1010011, FP_PLUS_ZERO, 32'd0, 32'd0, 32'd0);
        check_int(32'h010, 5'b00000, "fclass +0");
        commit_inst();

        // fclass.s: test sNaN -> bit 8 (0x100)
        step_inst(32'b1110000_00000_00001_001_00101_1010011, FP_SNAN, 32'd0, 32'd0, 32'd0);
        check_int(32'h100, 5'b00000, "fclass signaling NaN");
        commit_inst();

        // feq.s (+0 == -0) -> true (1)
        step_inst(32'b1010000_00010_00001_010_00101_1010011, FP_PLUS_ZERO, FP_MINUS_ZERO, 32'd0, 32'd0);
        check_int(32'd1, 5'b00000, "feq.s +0 == -0 -> 1");
        commit_inst();

        // flt.s (-0 < +0) -> false (0)
        step_inst(32'b1010000_00010_00001_001_00101_1010011, FP_MINUS_ZERO, FP_PLUS_ZERO, 32'd0, 32'd0);
        check_int(32'd0, 5'b00000, "flt.s -0 < +0 -> 0");
        commit_inst();

        // flt.s with NaN operand -> raises NV flag (bit 4: 5'b10000)
        step_inst(32'b1010000_00010_00001_001_00101_1010011, FP_ONE, FP_CANON_NAN, 32'd0, 32'd0);
        check_int(32'd0, 5'b10000, "flt.s 1.0 < NaN -> 0 with NV flag");
        commit_inst();

        $display("\n==================================================");
        $display("             TESTING FP CONVERSIONS               ");
        $display("==================================================");
        // Clear fflags first via csrrw x0, fflags, x0
        step_inst({12'h001, 5'd0, 3'b001, 5'd0, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'd0);
        commit_inst();

        // fcvt.w.s: -6.25 to signed int under RTZ (rm=001) -> -6 (0xFFFFFFFA), sets NX flag (bit 0)
        step_inst(32'b1100000_00000_00001_001_00101_1010011, FP_NEG_SIX_PT_25, 32'd0, 32'd0, 32'd0);
        check_int(-32'd6, 5'b00001, "fcvt.w.s -6.25 -> -6 (NX)");
        commit_inst();

        // fcvt.w.s: NaN operand -> 0x7FFFFFFF with NV flag
        step_inst(32'b1100000_00000_00001_000_00101_1010011, FP_CANON_NAN, 32'd0, 32'd0, 32'd0);
        check_int(32'h7FFFFFFF, 5'b10001, "fcvt.w.s NaN -> 0x7FFFFFFF (NV accrued)");
        commit_inst();

        // fcvt.s.w: 2 to float -> 2.0 (0x40000000)
        step_inst(32'b1101000_00000_00001_000_00010_1010011, 32'd0, 32'd0, 32'd0, 32'd2);
        check_fp(FP_TWO, 5'b10001, "fcvt.s.w 2 -> 2.0");
        commit_inst();

        $display("\n==================================================");
        $display("      TESTING FP ARITHMETIC, FMA & ROUNDING       ");
        $display("==================================================");
        // Clear fflags
        step_inst({12'h001, 5'd0, 3'b001, 5'd0, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'd0);
        commit_inst();

        // fadd.s: 1.0 + 2.0 = 3.0 (exact, no flags)
        step_inst(32'b0000000_00010_00001_000_00011_1010011, FP_ONE, FP_TWO, 32'd0, 32'd0);
        check_fp(FP_THREE, 5'b00000, "fadd.s 1.0 + 2.0 = 3.0");
        commit_inst();

        // fmul.s: 2.0 * 3.0 = 6.0
        step_inst(32'b0001000_00010_00001_000_00011_1010011, FP_TWO, FP_THREE, 32'd0, 32'd0);
        check_fp(FP_SIX, 5'b00000, "fmul.s 2.0 * 3.0 = 6.0");
        commit_inst();

        // fmadd.s: (2.0 * 3.0) + 4.0 = 10.0
        step_inst({5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1000011}, FP_TWO, FP_THREE, FP_FOUR, 32'd0);
        check_fp(FP_TEN, 5'b00000, "fmadd.s (2.0 * 3.0) + 4.0 = 10.0");
        commit_inst();

        // Inexact Rounding Test from Spec Section 4.5.2 Breakdown:
        // 1.0 / 3.0 under RNE -> 0x3EAAAAAB (NX)
        step_inst(32'b0001100_00010_00001_000_00011_1010011, FP_ONE, FP_THREE, 32'd0, 32'd0);
        check_fp(32'h3EAAAAAB, 5'b00001, "fdiv.s 1.0 / 3.0 under RNE");
        commit_inst();

        // 1.0 / 3.0 under RTZ (rm=001) -> 0x3EAAAAAA (NX)
        step_inst(32'b0001100_00010_00001_001_00011_1010011, FP_ONE, FP_THREE, 32'd0, 32'd0);
        check_fp(32'h3EAAAAAA, 5'b00001, "fdiv.s 1.0 / 3.0 under RTZ");
        commit_inst();

        // Divide by zero: 1.0 / 0.0 -> +inf with DZ flag (bit 3: 5'b01000)
        step_inst(32'b0001100_00010_00001_000_00011_1010011, FP_ONE, FP_PLUS_ZERO, 32'd0, 32'd0);
        check_fp(FP_PLUS_INF, 5'b01001, "fdiv.s 1.0 / 0.0 -> +inf (DZ accrued)");
        commit_inst();

        // fsqrt.s: sqrt(4.0) = 2.0
        step_inst(32'b0101100_00000_00001_000_00011_1010011, FP_FOUR, 32'd0, 32'd0, 32'd0);
        check_fp(FP_TWO, 5'b01001, "fsqrt.s 4.0 = 2.0");
        commit_inst();

        // fsqrt.s: sqrt(-1.0) -> canonical NaN with NV flag
        step_inst(32'b0101100_00000_00001_000_00011_1010011, 32'hBF800000, 32'd0, 32'd0, 32'd0);
        check_fp(FP_CANON_NAN, 5'b11001, "fsqrt.s -1.0 -> canonical NaN (NV accrued)");
        commit_inst();

        $display("\n==================================================");
        $display("          TESTING CSR TIMING & TRAPS              ");
        $display("==================================================");
        // Test csrrw x5, frm, 1 (sets frm to RTZ = 1)
        step_inst({12'h002, 5'd0, 3'b001, 5'd5, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'd1);
        check_int(32'd0, 5'b11001, "csrrw reads old frm (0)");
        // Commit write
        commit_inst();

        // Read frm back to confirm it was written on advance
        step_inst({12'h002, 5'd0, 3'b010, 5'd6, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'd0);
        check_int(32'd1, 5'b11001, "csrrs reads updated frm (1)");
        commit_inst();

        // Set frm to reserved value 5 (3'b101)
        step_inst({12'h002, 5'd0, 3'b001, 5'd0, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'd5);
        commit_inst();

        // Execute instruction with dynamic rounding mode (rm = 111): must assert o_rm_trap!
        step_inst(32'b0000000_00010_00001_111_00011_1010011, FP_ONE, FP_TWO, 32'd0, 32'd0);
        #1;
        if (rm_trap === 1'b1) begin
            $display("[PASS] FPU: Dynamic rm with reserved frm (5) asserted o_rm_trap");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] FPU: o_rm_trap was NOT asserted on reserved dynamic frm!");
            fail_count = fail_count + 1;
        end
        commit_inst();

        $display("\n==================================================");
        $display("                 SUMMARY REPORT                   ");
        $display("==================================================");
        $display("Total Passed: %0d", pass_count);
        $display("Total Failed: %0d", fail_count);
        if (fail_count == 0) begin
            $display(">>> ALL FPU TESTS PASSED! <<<");
        end else begin
            $display(">>> SOME TESTS FAILED! CHECK LOGS ABOVE. <<<");
        end
        $finish;
    end

endmodule
`default_nettype wire
