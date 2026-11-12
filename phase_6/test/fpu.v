`default_nettype none

// The floating-point unit implements the RISC-V F extension (single
// precision, IEEE 754-2008) and the Zicsr instructions for its control and
// status register fcsr. It sits in the execute stage beside the ALU and the
// multiply/divide unit, and decodes the details of its own instructions from
// the instruction word.
//
// It owns all F-extension state: fcsr, holding the dynamic rounding mode frm
// (fcsr[7:5]) and the accrued exception flags fflags (fcsr[4:0] = NV DZ OF
// UF NX). Floating-point exceptions never trap; they only set flags, which
// stay set until software clears them through a CSR instruction.
//
// fcsr only changes on a rising edge where the instruction in execute
// actually moves on to the memory stage (`i_advance`). An instruction can sit
// in execute for several cycles (for example while a cache miss freezes the
// pipeline); committing its flags or CSR write on every one of those cycles
// would make a CSR instruction read back its own write.
module fpu (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset. Clears fcsr (frm = round to nearest,
    // ties to even; no flags set).
    input  wire        i_rst,
    // The instruction word in the execute stage.
    input  wire [31:0] i_inst,
    // Asserted when `i_inst` is a valid, legal floating-point unit
    // instruction (decoder `o_fpu`): every OP-FP and fused multiply-add
    // instruction and the CSR instructions on fflags/frm/fcsr.
    input  wire        i_valid,
    // Asserted when the instruction in execute advances to the memory stage
    // on the next rising edge. Side effects (fcsr updates) happen only then.
    input  wire        i_advance,
    // Floating-point source operands (rs1, rs2, rs3), raw IEEE bit patterns.
    input  wire [31:0] i_frs1,
    input  wire [31:0] i_frs2,
    input  wire [31:0] i_frs3,
    // Integer source operand (forwarded rs1), used by fmv.w.x, fcvt.s.w[u]
    // and the register forms of the CSR instructions.
    input  wire [31:0] i_xrs1,
    // Result for instructions that write a floating-point register.
    output wire [31:0] o_fp_result,
    // Result for instructions that write an integer register: fmv.x.w,
    // fclass.s, feq/flt/fle.s, fcvt.w[u].s, and the old CSR value.
    output wire [31:0] o_int_result,
    // The accrued exception flags as they will be after this instruction
    // (reported in the retire trace).
    output wire [ 4:0] o_fflags,
    // Asserted while a multi-cycle operation (fdiv.s/fsqrt.s) is in
    // progress and the instruction must stay in execute.
    output wire        o_busy,
    // Asserted when a rounding instruction selects the dynamic rounding mode
    // (rm = 111) while frm holds a reserved value (101-111). The instruction
    // traps as illegal and changes no state.
    output wire        o_rm_trap
);

    // Decode only this slice. i_valid comes from the Phase 6 decoder.
    wire op_fp = i_valid && (i_inst[6:0] == 7'b1010011);
    wire unary_fp = op_fp &&
                 (i_inst[24:20] == 5'd0);
    wire move_to_int = unary_fp && (i_inst[31:25] == 7'b1110000) &&
                       (i_inst[14:12] == 3'b000);
    wire move_to_fp  = unary_fp && (i_inst[31:25] == 7'b1111000) &&
                       (i_inst[14:12] == 3'b000);
    wire classify    = unary_fp && (i_inst[31:25] == 7'b1110000) &&
                       (i_inst[14:12] == 3'b001);

    // Classify the raw bits. The exponent and fraction distinguish
    // zero, subnormal, normal, infinity, and NaN.
    wire sign = i_frs1[31];
    wire exp_zero = (i_frs1[30:23] == 8'h00);
    wire exp_ones = (i_frs1[30:23] == 8'hff);
    wire frac_zero = (i_frs1[22:0] == 23'd0);
    wire is_zero = exp_zero && frac_zero;
    wire is_subnormal = exp_zero && !frac_zero;
    wire is_normal = !exp_zero && !exp_ones;
    wire is_inf = exp_ones && frac_zero;
    wire is_nan = exp_ones && !frac_zero;
    wire is_qnan = is_nan && i_frs1[22];
    wire is_snan = is_nan && !i_frs1[22];

    // FCLASS.S sets one bit, from negative infinity (0) to quiet NaN (9).
    wire [9:0] class_mask = {
        is_qnan, is_snan,
        !sign && is_inf, !sign && is_normal, !sign && is_subnormal, !sign && is_zero,
         sign && is_zero, sign && is_subnormal, sign && is_normal, sign && is_inf
    };

    wire [2:0] funct3 = i_inst[14:12];
    wire sign_inject = op_fp && (i_inst[31:25] == 7'b0010000) && (funct3 <= 3'd2);
    wire min_max = op_fp && (i_inst[31:25] == 7'b0010100) && (funct3 <= 3'd1);
    wire compare = op_fp && (i_inst[31:25] == 7'b1010000) && (funct3 <= 3'd2);
    wire sign2 = i_frs2[31];
    wire inject_sign = (funct3 == 3'd0) ? sign2 :
                       (funct3 == 3'd1) ? !sign2 : (sign ^ sign2);
    wire [31:0] inject_result = {inject_sign, i_frs1[30:0]};

    wire nan2 = (i_frs2[30:23] == 8'hff) && (i_frs2[22:0] != 23'd0);
    wire snan2 = nan2 && !i_frs2[22];
    wire any_nan = is_nan || nan2;
    wire any_snan = is_snan || snan2;
    wire both_zero = is_zero && (i_frs2[30:0] == 31'd0);

    // Positive encodings grow with magnitude; negative encodings reverse
    // that order. Only min/max distinguish the two signs of zero.
    wire ordered_lt = (sign != sign2) ? sign :
                      sign ? (i_frs1[30:0] > i_frs2[30:0]) :
                             (i_frs1[30:0] < i_frs2[30:0]);
    wire cmp_eq = both_zero || (i_frs1 == i_frs2);
    wire cmp_lt = !both_zero && ordered_lt;
    wire cmp_result = !any_nan &&
                      ((funct3 == 3'd2) ? cmp_eq :
                       (funct3 == 3'd1) ? cmp_lt : (cmp_lt || cmp_eq));

    wire [31:0] min_max_result = (is_nan && nan2) ? 32'h7fc00000 :
                                 is_nan ? i_frs2 :
                                 nan2 ? i_frs1 :
                                 (ordered_lt ^ funct3[0]) ? i_frs1 : i_frs2;

    // CSR operands are either the forwarded integer register or a zero-extended uimm.
    wire [11:0] csr_addr = i_inst[31:20];
    wire [4:0] csr_src_field = i_inst[19:15];
    wire csr_is_flags = (csr_addr == 12'h001);
    wire csr_is_frm   = (csr_addr == 12'h002);
    wire csr_is_fcsr  = (csr_addr == 12'h003);
    wire csr_access = i_valid && (i_inst[6:0] == 7'b1110011) &&
                      (csr_is_flags || csr_is_frm || csr_is_fcsr) &&
                      (funct3[1:0] != 2'b00);
    wire csr_replace = (funct3[1:0] == 2'b01);
    wire csr_write = csr_access && (csr_replace || (csr_src_field != 5'd0));
    reg [7:0] fcsr;
    wire [7:0] csr_old = csr_is_flags ? {3'd0, fcsr[4:0]} :
                         csr_is_frm   ? {5'd0, fcsr[7:5]} : fcsr;
    wire [7:0] csr_src = funct3[2] ? {3'd0, csr_src_field} :
                         (csr_src_field == 5'd0) ? 8'd0 : i_xrs1[7:0];
    wire [7:0] csr_new = csr_replace ? csr_src :
                         (funct3[1:0] == 2'b10) ? (csr_old | csr_src) :
                                                   (csr_old & ~csr_src);

    // Only rounding instructions interpret funct3 as rm.
    wire [6:0] fp_funct7 = i_inst[31:25];
    wire round_op_fp = op_fp && ((fp_funct7 == 7'b0000000) || // add
                                 (fp_funct7 == 7'b0000100) || // subtract
                                 (fp_funct7 == 7'b0001000) || // multiply
                                 (fp_funct7 == 7'b0001100) || // divide
                                 (fp_funct7 == 7'b0101100) || // square root
                                 (fp_funct7 == 7'b1100000) || // to integer
                                 (fp_funct7 == 7'b1101000));  // from integer
    wire round_fused = i_valid && (i_inst[26:25] == 2'b00) &&
                       ((i_inst[6:0] == 7'b1000011) || (i_inst[6:0] == 7'b1000111) ||
                        (i_inst[6:0] == 7'b1001011) || (i_inst[6:0] == 7'b1001111));
    wire dynamic_rm = (funct3 == 3'b111);
    wire [2:0] effective_rm = dynamic_rm ? fcsr[7:5] : funct3;
    assign o_rm_trap = (round_op_fp || round_fused) && dynamic_rm && (effective_rm >= 3'd5);


    wire cvt_to_int = op_fp && (fp_funct7 == 7'b1100000) && (i_inst[24:20] <= 5'd1);
    wire cvt_to_fp  = op_fp && (fp_funct7 == 7'b1101000) && (i_inst[24:20] <= 5'd1);
    wire cvt_valid = (cvt_to_int || cvt_to_fp) && !o_rm_trap && (effective_rm <= 3'd4);
    wire [31:0] cvt_result;
    wire [4:0] cvt_flags;
    fpu_cvt convert (
        .i_to_int(cvt_to_int), .i_unsigned(i_inst[20]), .i_rm(effective_rm),
        .i_float(i_frs1), .i_int(i_xrs1), .o_result(cvt_result), .o_flags(cvt_flags)
    );


    // Add and subtract use a * 1.0 +/- b. Multiply skips the addend.
    wire fp_add = op_fp && (fp_funct7 == 7'b0000000);
    wire fp_sub = op_fp && (fp_funct7 == 7'b0000100);
    wire fp_mul = op_fp && (fp_funct7 == 7'b0001000);
    wire fma_valid = (fp_add || fp_sub || fp_mul || round_fused) &&
                     !o_rm_trap && (effective_rm <= 3'd4);
    wire [31:0] fma_result;
    wire [4:0] fma_flags;
    fpu_fma multiply_add (
        .i_a(i_frs1), .i_b((fp_add || fp_sub) ? 32'h3f800000 : i_frs2),
        .i_c(round_fused ? i_frs3 : i_frs2), .i_mul_only(fp_mul),
        .i_neg_product(round_fused && i_inst[3]),
        .i_neg_addend(fp_sub || (round_fused && i_inst[2])),
        .i_rm(effective_rm), .o_result(fma_result), .o_flags(fma_flags)
    );


    wire fp_div = op_fp && (fp_funct7 == 7'b0001100);
    wire fp_sqrt = op_fp && (fp_funct7 == 7'b0101100) && (i_inst[24:20] == 5'd0);
    wire div_sqrt_valid = (fp_div || fp_sqrt) && !o_rm_trap && (effective_rm <= 3'd4);
    wire [31:0] div_sqrt_result;
    wire [4:0] div_sqrt_flags;
    fpu_div_sqrt divide_root (
        .i_clk(i_clk), .i_rst(i_rst), .i_valid(div_sqrt_valid),
        .i_advance(i_advance), .i_sqrt(fp_sqrt), .i_a(i_frs1), .i_b(i_frs2),
        .i_rm(effective_rm), .o_busy(o_busy),
        .o_result(div_sqrt_result), .o_flags(div_sqrt_flags)
    );

    // Moves preserve every bit, including the sign of zero and NaN payloads.
    assign o_int_result = csr_access ? {24'd0, csr_old} :
                          move_to_int ? i_frs1 :
                          classify ? {22'd0, class_mask} :
                          compare ? {31'd0, cmp_result} :
                          (cvt_valid && cvt_to_int) ? cvt_result : 32'd0;
    assign o_fp_result = move_to_fp ? i_xrs1 :
                         sign_inject ? inject_result :
                         min_max ? min_max_result :
                         (cvt_valid && cvt_to_fp) ? cvt_result :
                         fma_valid ? fma_result :
                         div_sqrt_valid ? div_sqrt_result : 32'd0;

    // FEQ is quiet; FLT and FLE signal on any NaN. Min/max signal on sNaN.
    // Report the flags after the EX instruction, but commit only on advance.
    wire invalid = (compare && ((funct3 == 3'd2) ? any_snan : any_nan)) ||
                   (min_max && any_snan);
    wire [4:0] flags_next = fcsr[4:0] | {invalid, 4'd0} | (cvt_valid ? cvt_flags : 5'd0) |
                            (fma_valid ? fma_flags : 5'd0) |
                            (div_sqrt_valid ? div_sqrt_flags : 5'd0);
    wire [7:0] fcsr_next = o_rm_trap ? fcsr :
                           !csr_write ? {fcsr[7:5], flags_next} :
                           csr_is_flags ? {fcsr[7:5], csr_new[4:0]} :
                           csr_is_frm ? {csr_new[2:0], fcsr[4:0]} : csr_new;
    assign o_fflags = fcsr_next[4:0];

    always @(posedge i_clk) begin
        if (i_rst)
            fcsr <= 8'd0;
        else if (i_valid && i_advance && !o_busy)
            fcsr <= fcsr_next;
    end


endmodule

`default_nettype wire
