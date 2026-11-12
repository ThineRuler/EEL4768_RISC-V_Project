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
            begin : wait_for_fpu
                integer elapsed;
                elapsed = 0;
                while (busy && elapsed < 32) begin
                    @(posedge clk);
                    elapsed = elapsed + 1;
                end
                if (busy) $fatal(1, "FPU did not finish");
            end
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

    task check_trap(input expected, input [127:0] name);
        begin
            if (rm_trap === expected) begin
                $display("[PASS] Trap: %0s", name);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Trap: %0s, got %b expected %b", name, rm_trap, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask


    task conversion_case(
        input to_fp,
        input unsigned_operand,
        input [2:0] mode,
        input [31:0] fp_input, int_input, result,
        input [4:0] flags,
        input [127:0] name
    );
        begin
            // Clear accrued flags so each check observes this conversion alone.
            step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
            commit_inst();
            step_inst({(to_fp ? 7'b1101000 : 7'b1100000), 4'd0, unsigned_operand,
                       5'd1, mode, 5'd2, 7'b1010011}, fp_input, 0, 0, int_input);
            if (to_fp)
                check_fp(result, flags, name);
            else
                check_int(result, flags, name);
            commit_inst();
        end
    endtask

    reg round_sign;
    reg [2:0] round_mode;
    reg signed [9:0] round_exp;
    reg [26:0] round_sig;
    wire [31:0] round_fp;
    wire [4:0] round_flags;
    fpu_round round_probe (
        .i_sign(round_sign), .i_rm(round_mode), .i_to_int(1'b0),
        .i_exp(round_exp), .i_mag_grs({8'd0, round_sig}),
        .o_fp_result(round_fp), .o_int_mag(), .o_flags(round_flags)
    );

    task fp_round_case(
        input sign, input [2:0] mode, input signed [9:0] exp,
        input [23:0] sig, input [31:0] result, input [4:0] flags,
        input [127:0] name
    );
        begin
            round_sign = sign;
            round_mode = mode;
            round_exp = exp;
            round_sig = {sig, 3'b000};
            #1;
            if (round_fp === result && round_flags === flags) begin
                $display("[PASS] Round: %0s", name);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Round: %0s -> %h flags %b (expected %h flags %b)",
                         name, round_fp, round_flags, result, flags);
                fail_count = fail_count + 1;
            end
        end
    endtask


    task div_sqrt_case(
        input sqrt_op,
        input [2:0] mode,
        input [31:0] a, b, expected,
        input [4:0] expected_flags,
        input expect_busy,
        input [127:0] name
    );
        integer busy_cycles;
        begin
            step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
            commit_inst();
            @(negedge clk);
            inst = {(sqrt_op ? 7'b0101100 : 7'b0001100),
                    (sqrt_op ? 5'd0 : 5'd2), 5'd1, mode, 5'd3, 7'b1010011};
            frs1 = a;
            frs2 = b;
            valid = 1'b1;
            busy_cycles = 0;
            #1;
            if (busy !== expect_busy)
                $fatal(1, "%0s: wrong busy signal on entry", name);
            while (busy && busy_cycles < 32) begin
                if (fflags !== 5'd0)
                    $fatal(1, "%0s: flags visible before completion", name);
                @(negedge clk);
                busy_cycles = busy_cycles + 1;
            end
            if (busy || (expect_busy && busy_cycles < 2))
                $fatal(1, "%0s: wrong busy duration", name);
            check_fp(expected, expected_flags, name);
            // Emulate a cache freeze after the result is ready.
            repeat (4) begin
                @(negedge clk);
                if (busy !== 1'b0 || fp_result !== expected || fflags !== expected_flags)
                    $fatal(1, "%0s: result changed before advance", name);
            end
            commit_inst();
            step_inst({12'h001, 5'd0, 3'b010, 5'd4, 7'b1110011}, 0, 0, 0, 0);
            check_int({27'd0, expected_flags}, expected_flags, "flags on advance");
            commit_inst();
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

        // Sign injection changes only the sign, even for a signaling NaN.
        step_inst(32'b0010000_00010_00001_001_00011_1010011, FP_SNAN, FP_PLUS_ZERO, 32'd0, 32'd0);
        check_fp(32'hFF800001, 5'b00000, "fsgnjn NaN bits");
        commit_inst();

        step_inst(32'b1010000_00010_00001_010_00101_1010011, FP_CANON_NAN, FP_ONE, 32'd0, 32'd0);
        check_int(32'd0, 5'b00000, "feq quiet NaN");
        commit_inst();

        step_inst(32'b1010000_00010_00001_000_00101_1010011, FP_NEG_SIX_PT_25, FP_MINUS_ZERO, 32'd0, 32'd0);
        check_int(32'd1, 5'b00000, "fle negative");
        commit_inst();

        step_inst(32'b0010100_00010_00001_000_00011_1010011, FP_PLUS_ZERO, FP_MINUS_ZERO, 32'd0, 32'd0);
        check_fp(FP_MINUS_ZERO, 5'b00000, "fmin signed zero");
        commit_inst();

        step_inst(32'b0010100_00010_00001_001_00011_1010011, FP_MINUS_ZERO, FP_PLUS_ZERO, 32'd0, 32'd0);
        check_fp(FP_PLUS_ZERO, 5'b00000, "fmax signed zero");
        commit_inst();

        step_inst(32'b0010100_00010_00001_000_00011_1010011, FP_ONE, FP_CANON_NAN, 32'd0, 32'd0);
        check_fp(FP_ONE, 5'b00000, "fmin one NaN");
        commit_inst();

        step_inst(32'b0010100_00010_00001_001_00011_1010011, 32'h7FC00001, FP_CANON_NAN, 32'd0, 32'd0);
        check_fp(FP_CANON_NAN, 5'b00000, "fmax two NaNs");
        commit_inst();

        // An instruction held in EX reports NV but must not commit it yet.
        step_inst(32'b0010100_00010_00001_000_00011_1010011, FP_SNAN, FP_ONE, 32'd0, 32'd0);
        check_fp(FP_ONE, 5'b10000, "fmin sNaN");
        repeat (2) @(negedge clk);
        valid = 1'b0;
        #1;
        check_int(32'd0, 5'b00000, "uncommitted NV");

        // flt.s with NaN operand -> raises NV flag (bit 4: 5'b10000)
        step_inst(32'b1010000_00010_00001_001_00101_1010011, FP_ONE, FP_CANON_NAN, 32'd0, 32'd0);
        check_int(32'd0, 5'b10000, "flt.s 1.0 < NaN -> 0 with NV flag");
        commit_inst();

        step_inst(32'b1110000_00000_00001_001_00101_1010011, FP_CANON_NAN, 32'd0, 32'd0, 32'd0);
        check_int(32'h200, 5'b10000, "NV stays accrued");
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

        conversion_case(1'b0, 1'b0, 3'd0, 32'h40200000, 32'h0, 32'h00000002, 5'b00001, "RNE 2.5 to 2");
        conversion_case(1'b0, 1'b0, 3'd4, 32'h40200000, 32'h0, 32'h00000003, 5'b00001, "RMM 2.5 to 3");
        conversion_case(1'b0, 1'b0, 3'd2, 32'hc0200000, 32'h0, 32'hfffffffd, 5'b00001, "RDN -2.5 to -3");
        conversion_case(1'b0, 1'b0, 3'd3, 32'hc0200000, 32'h0, 32'hfffffffe, 5'b00001, "RUP -2.5 to -2");
        conversion_case(1'b0, 1'b1, 3'd1, 32'hbf000000, 32'h0, 32'h0, 5'b00001, "RTZ unsigned -0.5");
        conversion_case(1'b0, 1'b1, 3'd2, 32'hbf000000, 32'h0, 32'h0, 5'b10000, "RDN unsigned -0.5");
        conversion_case(1'b0, 1'b0, 3'd0, 32'hcf000000, 32'h0, 32'h80000000, 5'b00000, "exact INT_MIN");
        conversion_case(1'b0, 1'b0, 3'd0, 32'h4f000000, 32'h0, 32'h7fffffff, 5'b10000, "signed overflow");
        conversion_case(1'b0, 1'b1, 3'd0, 32'h4f7fffff, 32'h0, 32'hffffff00, 5'b00000, "large uint exact");
        conversion_case(1'b0, 1'b1, 3'd0, 32'h4f800000, 32'h0, 32'hffffffff, 5'b10000, "uint overflow");
        conversion_case(1'b0, 1'b1, 3'd0, 32'hffc00001, 32'h0, 32'hffffffff, 5'b10000, "negative NaN");
        conversion_case(1'b0, 1'b0, 3'd3, 32'h00000001, 32'h0, 32'h1, 5'b00001, "subnormal to 1");
        conversion_case(1'b1, 1'b0, 3'd0, 32'h0, 32'h80000000, 32'hcf000000, 5'b00000, "INT_MIN to FP");
        conversion_case(1'b1, 1'b1, 3'd0, 32'h0, 32'hffffffff, 32'h4f800000, 5'b00001, "UINT_MAX RNE");
        conversion_case(1'b1, 1'b1, 3'd1, 32'h0, 32'hffffffff, 32'h4f7fffff, 5'b00001, "UINT_MAX RTZ");
        conversion_case(1'b1, 1'b0, 3'd0, 32'h0, 32'h01000001, 32'h4b800000, 5'b00001, "int tie RNE");
        conversion_case(1'b1, 1'b0, 3'd4, 32'h0, 32'h01000001, 32'h4b800001, 5'b00001, "int tie RMM");

        // Exercise dynamic rounding with a mode that changes the result.
        step_inst({12'h002, 5'd4, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        conversion_case(1'b0, 1'b0, 3'd7, 32'h40200000, 32'd0, 32'd3, 5'b00001, "dynamic RMM");
        step_inst({12'h002, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();

        fp_round_case(1'b0, 3'd0, -10'sd149, 24'h800000, 32'h00000001, 5'b00000, "exact subnormal");
        fp_round_case(1'b1, 3'd0, -10'sd150, 24'h800000, 32'h80000000, 5'b00011, "tiny tie to -0");
        fp_round_case(1'b0, 3'd0, -10'sd127, 24'hffffff, 32'h00800000, 5'b00001, "tiny becomes normal");
        fp_round_case(1'b0, 3'd1,  10'sd128, 24'h800000, 32'h7f7fffff, 5'b00101, "RTZ overflow");


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
        // Check arithmetic flags per instruction, independent of earlier cases.
        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0000100_00010_00001_000_00011_1010011, FP_ONE, FP_TWO, 0, 0);
        check_fp(32'hbf800000, 5'b00000, "fsub 1 - 2");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst({5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1000111}, FP_TWO, FP_THREE, FP_FOUR, 0);
        check_fp(FP_TWO, 5'b00000, "fmsub 6 - 4");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst({5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1001011}, FP_TWO, FP_THREE, FP_FOUR, 0);
        check_fp(32'hc0000000, 5'b00000, "fnmsub -6 + 4");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst({5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1001111}, FP_TWO, FP_THREE, FP_FOUR, 0);
        check_fp(32'hc1200000, 5'b00000, "fnmadd -6 - 4");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst({5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1000011}, 32'h3f800001, 32'h3f7ffffe, 32'hbf800000, 0);
        check_fp(32'ha8800000, 5'b00000, "one rounding");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0001000_00010_00001_000_00011_1010011, FP_MINUS_ZERO, FP_TWO, 0, 0);
        check_fp(FP_MINUS_ZERO, 5'b00000, "fmul negative 0");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0000100_00010_00001_010_00011_1010011, FP_ONE, FP_ONE, 0, 0);
        check_fp(FP_MINUS_ZERO, 5'b00000, "RDN cancel zero");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0000000_00010_00001_000_00011_1010011, FP_POS_SUBNORM, FP_POS_SUBNORM, 0, 0);
        check_fp(32'h00000002, 5'b00000, "exact subnormals");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0000100_00010_00001_001_00011_1010011, FP_ONE, FP_POS_SUBNORM, 0, 0);
        check_fp(32'h3f7fffff, 5'b00001, "distant subtract");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0001000_00010_00001_000_00011_1010011, FP_POS_SUBNORM, 32'h3f000000, 0, 0);
        check_fp(FP_PLUS_ZERO, 5'b00011, "underflow tie");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0001000_00010_00001_001_00011_1010011, 32'h7f7fffff, FP_TWO, 0, 0);
        check_fp(32'h7f7fffff, 5'b00101, "RTZ overflow");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst(32'b0000000_00010_00001_000_00011_1010011, FP_PLUS_INF, FP_MINUS_INF, 0, 0);
        check_fp(FP_CANON_NAN, 5'b10000, "infinity cancel");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();
        step_inst({5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1000011}, FP_PLUS_INF, FP_PLUS_ZERO, FP_CANON_NAN, 0);
        check_fp(FP_CANON_NAN, 5'b10000, "inf*0 with qNaN");
        commit_inst();

        step_inst({12'h001, 5'd0, 3'b101, 5'd0, 7'b1110011}, 0, 0, 0, 0);
        commit_inst();


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

        div_sqrt_case(1'b0, 3'd0, FP_ONE, FP_THREE, 32'h3eaaaaab, 5'b00001, 1'b1, "divide done hold");
        div_sqrt_case(1'b1, 3'd3, FP_TWO, 0, 32'h3fb504f4, 5'b00001, 1'b1, "sqrt2 RUP");
        div_sqrt_case(1'b1, 3'd0, 32'h00000002, 0, 32'h1a800000, 5'b00000, 1'b1, "subnormal square");
        div_sqrt_case(1'b1, 3'd0, FP_MINUS_ZERO, FP_SNAN, FP_MINUS_ZERO, 5'b00000, 1'b0, "sqrt -0");
        div_sqrt_case(1'b1, 3'd0, FP_MINUS_INF, 0, FP_CANON_NAN, 5'b10000, 1'b0, "sqrt -inf");
        div_sqrt_case(1'b0, 3'd0, FP_POS_SUBNORM, FP_TWO, FP_PLUS_ZERO, 5'b00011, 1'b1, "tiny divide tie");
        div_sqrt_case(1'b0, 3'd3, FP_POS_SUBNORM, FP_TWO, FP_POS_SUBNORM, 5'b00011, 1'b1, "tiny divide RUP");
        div_sqrt_case(1'b0, 3'd0, FP_ONE, FP_MINUS_ZERO, FP_MINUS_INF, 5'b01000, 1'b0, "divide by -0");
        div_sqrt_case(1'b0, 3'd0, FP_PLUS_INF, FP_PLUS_ZERO, FP_PLUS_INF, 5'b00000, 1'b0, "inf / zero");
        div_sqrt_case(1'b0, 3'd0, FP_PLUS_ZERO, FP_PLUS_ZERO, FP_CANON_NAN, 5'b10000, 1'b0, "zero / zero");
        div_sqrt_case(1'b0, 3'd0, FP_ONE, FP_MINUS_INF, FP_MINUS_ZERO, 5'b00000, 1'b0, "one / -inf");

        $display("\n==================================================");
        $display("          TESTING CSR TIMING & TRAPS              ");
        $display("==================================================");
        // Establish a known fcsr independent of which arithmetic tests ran.
        step_inst({12'h003, 5'd0, 3'b101, 5'd0, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'd0);
        commit_inst();

        step_inst({12'h003, 5'd1, 3'b001, 5'd5, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'hffffffa3);
        check_int(32'h0, 5'b00011, "csrrw fcsr");
        repeat (3) @(negedge clk);
        check_int(32'h0, 5'b00011, "held CSR read");
        commit_inst();

        step_inst({12'h002, 5'd0, 3'b010, 5'd6, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h0);
        check_int(32'h5, 5'b00011, "read frm alias");
        commit_inst();

        step_inst({12'h001, 5'd1, 3'b010, 5'd5, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h10);
        check_int(32'h3, 5'b10011, "csrrs flags");
        commit_inst();

        step_inst({12'h001, 5'd1, 3'b011, 5'd5, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h2);
        check_int(32'h13, 5'b10001, "csrrc flags");
        commit_inst();

        step_inst({12'h002, 5'd1, 3'b101, 5'd5, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h0);
        check_int(32'h5, 5'b10001, "csrrwi frm");
        commit_inst();

        step_inst({12'h001, 5'd4, 3'b110, 5'd5, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h0);
        check_int(32'h11, 5'b10101, "csrrsi flags");
        commit_inst();

        step_inst({12'h001, 5'd16, 3'b111, 5'd5, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h0);
        check_int(32'h15, 5'b00101, "csrrci flags");
        commit_inst();

        step_inst({12'h003, 5'd0, 3'b010, 5'd6, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'hffffffff);
        check_int(32'h25, 5'b00101, "read whole fcsr");
        commit_inst();

        step_inst({12'h002, 5'd5, 3'b101, 5'd0, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h0);
        check_int(32'h1, 5'b00101, "reserved frm");
        commit_inst();

        // A reserved frm only traps a valid instruction that uses dynamic rm.
        step_inst(32'b0000000_00010_00001_111_00011_1010011, FP_ONE, FP_TWO, 32'd0, 32'd0);
        check_trap(1'b1, "reserved dynamic");
        commit_inst();

        step_inst(32'b0000000_00010_00001_000_00011_1010011, FP_ONE, FP_TWO, 32'd0, 32'd0);
        check_trap(1'b0, "static RNE");
        commit_inst();

        step_inst(32'b1010000_00010_00001_010_00101_1010011, FP_ONE, FP_ONE, 32'd0, 32'd0);
        check_trap(1'b0, "compare not rm");
        commit_inst();

        step_inst({12'h003, 5'd0, 3'b010, 5'd6, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h0);
        check_int(32'ha5, 5'b00101, "trap preserved CSR");
        commit_inst();

        step_inst({12'h002, 5'd4, 3'b101, 5'd0, 7'b1110011}, 32'd0, 32'd0, 32'd0, 32'h0);
        check_int(32'h5, 5'b00101, "valid frm");
        commit_inst();

        step_inst(32'b1100000_00000_00001_111_00101_1010011, FP_ONE, 32'd0, 32'd0, 32'd0);
        check_trap(1'b0, "valid dynamic rm");
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
