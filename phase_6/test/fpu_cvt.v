`default_nettype none

module fpu_cvt (
    input  wire        i_to_int,
    input  wire        i_unsigned,
    input  wire [2:0]  i_rm,
    input  wire [31:0] i_float,
    input  wire [31:0] i_int,
    output wire [31:0] o_result,
    output wire [4:0]  o_flags
);
    // Integer to float: extract magnitude, then move the leading 1 to bit 31.
    wire int_sign = !i_unsigned && i_int[31];
    wire [31:0] int_mag = int_sign ? (32'd0 - i_int) : i_int;
    wire s16 = !(|int_mag[31:16]);
    wire [31:0] n16 = s16 ? (int_mag << 16) : int_mag;
    wire s8 = !(|n16[31:24]);
    wire [31:0] n8 = s8 ? (n16 << 8) : n16;
    wire s4 = !(|n8[31:28]);
    wire [31:0] n4 = s4 ? (n8 << 4) : n8;
    wire s2 = !(|n4[31:30]);
    wire [31:0] n2 = s2 ? (n4 << 2) : n4;
    wire s1 = !n2[31];
    wire [31:0] norm = s1 ? (n2 << 1) : n2;
    wire [9:0] int_exp = 10'd31 - {5'd0, s16, s8, s4, s2, s1};
    wire [26:0] int_sig_grs = {norm[31:6], (|norm[5:0])};

    // Float to integer: position the significand relative to the integer
    // binary point. Exponents above 31 always overflow a 32-bit result.
    wire [7:0] fp_exp = i_float[30:23];
    wire normal_exp = (fp_exp != 8'd0);
    wire [7:0] effective_exp = normal_exp ? fp_exp : 8'd1;
    wire [23:0] fp_sig = {normal_exp, i_float[22:0]};
    wire [7:0] int_distance = 8'd158 - effective_exp;
    wire [5:0] int_shift = (effective_exp > 8'd158) ? 6'd0 :
                           (int_distance >= 8'd35) ? 6'd35 : int_distance[5:0];
    wire [34:0] fp_mag_grs;
    fpu_shift_right_jam #(.WIDTH(35)) to_int_align (
        .i_value({fp_sig, 11'd0}), .i_shift(int_shift), .o_value(fp_mag_grs)
    );

    wire [31:0] fp_result;
    wire [32:0] rounded_int;
    wire [4:0] rounded_flags;
    fpu_round rounder (
        .i_sign(i_to_int ? i_float[31] : int_sign),
        .i_rm(i_rm), .i_to_int(i_to_int), .i_exp(int_exp),
        .i_mag_grs(i_to_int ? fp_mag_grs : {8'd0, int_sig_grs}),
        .o_fp_result(fp_result), .o_int_mag(rounded_int), .o_flags(rounded_flags)
    );

    wire fp_sign = i_float[31];
    wire fp_nan = (fp_exp == 8'hff) && (|i_float[22:0]);
    wire [32:0] signed_limit = fp_sign ? 33'h080000000 : 33'h07fffffff;
    wire out_of_range = (fp_exp > 8'd158) ||
                        (i_unsigned ? (rounded_int[32] || (fp_sign && (|rounded_int))) :
                                      (rounded_int > signed_limit));
    wire clamp_low = fp_sign && !fp_nan;
    wire [31:0] saturated = i_unsigned ? (clamp_low ? 32'd0 : 32'hffffffff) :
                                        (clamp_low ? 32'h80000000 : 32'h7fffffff);
    wire [31:0] int_result = fp_sign ? (32'd0 - rounded_int[31:0]) : rounded_int[31:0];
    assign o_result = !i_to_int ? fp_result : out_of_range ? saturated : int_result;
    assign o_flags = (i_to_int && out_of_range) ? 5'b10000 : rounded_flags;
endmodule

`default_nettype wire
