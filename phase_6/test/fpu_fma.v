`default_nettype none

// (a * b) + c, optionally negating the product and/or addend.
// i_mul_only skips c entirely, preserving the sign of a zero product.
module fpu_fma (
    input  wire [31:0] i_a, i_b, i_c,
    input  wire        i_mul_only, i_neg_product, i_neg_addend,
    input  wire [2:0]  i_rm,
    output wire [31:0] o_result,
    output wire [4:0]  o_flags
);
    wire [7:0] ea = i_a[30:23];
    wire [7:0] eb = i_b[30:23];
    wire [7:0] ec = i_c[30:23];
    wire [23:0] sa = {(ea != 8'd0), i_a[22:0]};
    wire [23:0] sb = {(eb != 8'd0), i_b[22:0]};
    wire [23:0] sc = {(ec != 8'd0), i_c[22:0]};
    wire zero_a = !(|sa);
    wire zero_b = !(|sb);
    wire zero_c = i_mul_only || !(|sc);
    wire nan_a = (ea == 8'hff) && (|i_a[22:0]);
    wire nan_b = (eb == 8'hff) && (|i_b[22:0]);
    wire nan_c = !i_mul_only && (ec == 8'hff) && (|i_c[22:0]);
    wire inf_a = (ea == 8'hff) && !nan_a;
    wire inf_b = (eb == 8'hff) && !nan_b;
    wire inf_c = !i_mul_only && (ec == 8'hff) && !nan_c;

    wire sign_p = i_a[31] ^ i_b[31] ^ i_neg_product;
    wire sign_c = i_c[31] ^ i_neg_addend;
    wire different_sign = sign_p ^ sign_c;
    wire bad_product = (inf_a && zero_b) || (inf_b && zero_a);
    wire inf_product = (inf_a || inf_b) && !nan_a && !nan_b && !bad_product;
    wire invalid = bad_product || (inf_product && inf_c && different_sign) ||
                   (nan_a && !i_a[22]) || (nan_b && !i_b[22]) || (nan_c && !i_c[22]);
    wire result_nan = invalid || nan_a || nan_b || nan_c;
    wire result_inf = inf_product || inf_c;
    wire inf_sign = inf_product ? sign_p : sign_c;

    // Multiply the two 24-bit significands with a tree of partial sums.
    // No partial product is discarded, including for subnormal operands.
    wire [47:0] tree [1:63];
    genvar g;
    generate
        for (g = 0; g < 32; g = g + 1) begin : leaf
            if (g < 24) begin : partial_product
                assign tree[32+g] = sb[g] ?
                                    {{(24-g){1'b0}}, sa, {g{1'b0}}} : 48'd0;
            end else begin : unused_leaf
                assign tree[32+g] = 48'd0;
            end
        end
        for (g = 1; g < 32; g = g + 1) begin : sum_node
            assign tree[g] = tree[2*g] + tree[2*g+1];
        end
    endgenerate

    wire [47:0] norm_p, norm_c;
    wire [5:0] shift_p, shift_c;
    fpu_normalize normalize_product (.i_mag(tree[1]), .o_mag(norm_p), .o_shift(shift_p));
    fpu_normalize normalize_addend (.i_mag({sc, 24'd0}), .o_mag(norm_c), .o_shift(shift_c));

    // Exponents here are unbiased and refer to bit 47 of each normalized value.
    wire [7:0] ea_eff = (ea == 8'd0) ? 8'd1 : ea;
    wire [7:0] eb_eff = (eb == 8'd0) ? 8'd1 : eb;
    wire [7:0] ec_eff = (ec == 8'd0) ? 8'd1 : ec;
    wire [9:0] ep = {2'd0, ea_eff} + {2'd0, eb_eff} - 10'd253 - {4'd0, shift_p};
    wire [9:0] ec_norm = {2'd0, ec_eff} - 10'd127 - {4'd0, shift_c};
    // Flip the sign bit to compare two's-complement exponents as unsigned.
    wire c_exp_greater = (ec_norm ^ 10'h200) > (ep ^ 10'h200);
    wire zero_p = zero_a || zero_b;
    wire c_is_large = !zero_c &&
                      (zero_p || c_exp_greater || ((ec_norm == ep) && (norm_c > norm_p)));
    wire [47:0] large_sig = c_is_large ? norm_c : norm_p;
    wire [47:0] small_sig = c_is_large ? norm_p : norm_c;
    wire [9:0] large_exp = c_is_large ? ec_norm : ep;
    wire [9:0] small_exp = c_is_large ? ep : ec_norm;
    wire small_zero = c_is_large ? zero_p : zero_c;
    wire [9:0] gap = large_exp - small_exp;
    wire [5:0] align_shift = (gap >= 10'd51) ? 6'd51 : gap[5:0];

    // The three extra low bits keep nearby operands exact during cancellation.
    // If a distant operand loses bits during alignment, keep a sticky bit.
    wire [50:0] small_aligned;
    fpu_shift_right_jam #(.WIDTH(51)) align_small (
        .i_value(small_zero ? 51'd0 : {small_sig, 3'd0}),
        .i_shift(align_shift), .o_value(small_aligned)
    );
    wire [51:0] large_mag = {1'b0, large_sig, 3'd0};
    wire [51:0] small_mag = {1'b0, small_aligned};
    wire [51:0] magnitude = different_sign ? large_mag - small_mag : large_mag + small_mag;
    wire exact_zero = !(|magnitude);
    wire zero_sign = i_mul_only || !different_sign ? sign_p : (i_rm == 3'd2);
    wire result_sign = exact_zero ? zero_sign : c_is_large ? sign_c : sign_p;

    wire [50:0] norm_result;
    wire [5:0] shift_result;
    fpu_normalize #(.WIDTH(51)) normalize_sum (
        .i_mag(magnitude[50:0]), .o_mag(norm_result), .o_shift(shift_result)
    );
    wire [50:0] norm_sum = magnitude[51] ?
                           {magnitude[51:2], (|magnitude[1:0])} : norm_result;
    wire [9:0] result_exp = exact_zero ? 10'd0 :
                                   magnitude[51] ? large_exp + 10'd1 :
                                   large_exp - {4'd0, shift_result};
    wire [26:0] sig_grs = {norm_sum[50:25], (|norm_sum[24:0])};
    wire [31:0] rounded;
    wire [4:0] rounded_flags;
    fpu_round rounder (
        .i_sign(result_sign), .i_rm(i_rm), .i_to_int(1'b0),
        .i_exp(result_exp), .i_mag_grs({8'd0, sig_grs}),
        .o_fp_result(rounded), .o_int_mag(), .o_flags(rounded_flags)
    );

    assign o_result = result_nan ? 32'h7fc00000 :
                      result_inf ? {inf_sign, 31'h7f800000} : rounded;
    assign o_flags = result_nan ? {invalid, 4'd0} :
                     result_inf ? 5'd0 : rounded_flags;
endmodule

`default_nettype wire
