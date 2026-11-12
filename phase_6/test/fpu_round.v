`default_nettype none

module fpu_round (
    input  wire        i_sign,
    input  wire [2:0]  i_rm,
    input  wire        i_to_int,
    input  wire [9:0] i_exp,
    // Low three bits are guard, round and sticky. For FP, the next
    // 24 bits hold a normalized significand (unless the input is zero).
    // For integer rounding, the next 32 bits hold the truncated magnitude.
    input  wire [34:0] i_mag_grs,
    output wire [31:0] o_fp_result,
    output wire [32:0] o_int_mag,
    output wire [4:0]  o_flags
);
    // Exponents use 10-bit two's complement. Among negative values,
    // unsigned order agrees with signed order once the sign bits match.
    localparam [9:0] MIN_EXP = 10'd0 - 10'd126;
    wire tiny_exp = i_exp[9] && (i_exp < MIN_EXP);
    wire [9:0] denorm_distance = MIN_EXP - i_exp;
    wire [5:0] denorm_shift = !tiny_exp ? 6'd0 :
                              (denorm_distance >= 10'd27) ? 6'd27 :
                              denorm_distance[5:0];
    wire [26:0] denorm_sig;
    fpu_shift_right_jam #(.WIDTH(27)) denorm (
        .i_value(i_mag_grs[26:0]), .i_shift(denorm_shift), .o_value(denorm_sig)
    );

    wire [34:0] unrounded = i_to_int ? i_mag_grs : {8'd0, denorm_sig};
    wire guard_bit = unrounded[2];
    wire tail_bits = |unrounded[1:0];
    wire inexact = guard_bit || tail_bits;
    wire increment = (i_rm == 3'd0) ? (guard_bit && (tail_bits || unrounded[3])) :
                     (i_rm == 3'd2) ? (i_sign && inexact) :
                     (i_rm == 3'd3) ? (!i_sign && inexact) :
                     (i_rm == 3'd4) ? guard_bit : 1'b0;
    wire [32:0] rounded = {1'b0, unrounded[34:3]} + {32'd0, increment};
    assign o_int_mag = rounded;

    wire carry = rounded[24];
    wire [23:0] fp_sig = carry ? rounded[24:1] : rounded[23:0];
    wire [9:0] exp_base = tiny_exp ? MIN_EXP : i_exp;
    wire [9:0] exp_rounded = exp_base + (carry ? 10'd1 : 10'd0);
    wire [9:0] biased_exp = exp_rounded + 10'd127;
    wire [7:0] fp_exp = fp_sig[23] ? biased_exp[7:0] : 8'd0;
    wire overflow = (|i_mag_grs) && !exp_rounded[9] && (exp_rounded > 10'd127);
    wire underflow = !overflow && (fp_exp == 8'd0) && inexact;
    wire to_inf = (i_rm == 3'd0) || (i_rm == 3'd4) ||
                  ((i_rm == 3'd2) && i_sign) || ((i_rm == 3'd3) && !i_sign);
    assign o_fp_result = overflow ? {i_sign, (to_inf ? 31'h7f800000 : 31'h7f7fffff)} :
                                   {i_sign, fp_exp, fp_sig[22:0]};
    assign o_flags = i_to_int ? {4'd0, inexact} :
                                {2'd0, overflow, underflow, (inexact || overflow)};
endmodule

`default_nettype wire
