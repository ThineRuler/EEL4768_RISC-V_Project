`default_nettype none

module fpu_div_sqrt (
    input  wire        i_clk, i_rst,
    input  wire        i_valid, i_advance, i_sqrt,
    input  wire [31:0] i_a, i_b,
    input  wire [2:0]  i_rm,
    output wire        o_busy,
    output wire [31:0] o_result,
    output wire [4:0]  o_flags
);
    wire [7:0] ea = i_a[30:23];
    wire [7:0] eb = i_b[30:23];
    wire [23:0] sa = {(ea != 8'd0), i_a[22:0]};
    wire [23:0] sb = {(eb != 8'd0), i_b[22:0]};
    wire zero_a = !(|sa);
    wire zero_b = !(|sb);
    wire nan_a = (ea == 8'hff) && (|i_a[22:0]);
    wire nan_b = (eb == 8'hff) && (|i_b[22:0]);
    wire inf_a = (ea == 8'hff) && !nan_a;
    wire inf_b = (eb == 8'hff) && !nan_b;
    wire input_sign = i_sqrt ? i_a[31] : (i_a[31] ^ i_b[31]);

    // Special operands finish immediately; square root only examines a.
    wire invalid = (nan_a && !i_a[22]) ||
                   (i_sqrt ? (i_a[31] && !zero_a && !nan_a) :
                    ((nan_b && !i_b[22]) || (zero_a && zero_b) || (inf_a && inf_b)));
    wire result_nan = invalid || nan_a || (!i_sqrt && nan_b);
    wire result_inf = i_sqrt ? inf_a : (inf_a || zero_b);
    wire result_zero = zero_a || (!i_sqrt && inf_b);
    wire div_by_zero = !i_sqrt && zero_b && !zero_a && !inf_a && !result_nan;
    wire special = result_nan || result_inf || result_zero;
    wire [31:0] special_result = result_nan ? 32'h7fc00000 :
                                 {input_sign, (result_inf ? 31'h7f800000 : 31'd0)};
    wire [4:0] special_flags = {invalid, div_by_zero, 3'd0};

    wire [23:0] norm_a, norm_b;
    wire [5:0] shift_a, shift_b;
    fpu_normalize #(.WIDTH(24)) normalize_a (.i_mag(sa), .o_mag(norm_a), .o_shift(shift_a));
    fpu_normalize #(.WIDTH(24)) normalize_b (.i_mag(sb), .o_mag(norm_b), .o_shift(shift_b));
    wire [7:0] ea_eff = (ea == 8'd0) ? 8'd1 : ea;
    wire [7:0] eb_eff = (eb == 8'd0) ? 8'd1 : eb;
    wire [9:0] exp_a = {2'd0, ea_eff} - 10'd127 - {4'd0, shift_a};
    wire [9:0] exp_b = {2'd0, eb_eff} - 10'd127 - {4'd0, shift_b};

    // If a/b is below 1, double the numerator and lower the exponent.
    // The leading quotient bit is then known to be 1.
    wire double_a = (norm_a < norm_b);
    wire [24:0] div_numerator = double_a ? {norm_a, 1'b0} : {1'b0, norm_a};
    wire [9:0] div_exp = exp_a - exp_b - (double_a ? 10'd1 : 10'd0);

    // Make the exponent even and scale the significand so the integer
    // root contains 26 bits (24 significant bits plus guard and round).
    wire [51:0] sqrt_input = exp_a[0] ? {norm_a, 28'd0} : {1'b0, norm_a, 27'd0};
    // Sign-extend the two's-complement exponent while halving it.
    wire [9:0] sqrt_exp = {exp_a[9], exp_a[9:1]};

    localparam IDLE = 2'd0, RUN = 2'd1, DONE = 2'd2;
    reg [1:0] state;
    reg [4:0] remaining;
    reg run_sqrt, saved_sign;
    reg [2:0] saved_rm;
    reg [9:0] saved_exp;
    reg [25:0] digits;
    reg [23:0] divisor;
    reg [24:0] div_rem;
    reg [51:0] radicand;
    reg [28:0] sqrt_rem;

    // One restoring division bit per cycle.
    wire [24:0] div_shifted = {div_rem[23:0], 1'b0};
    wire div_bit = (div_shifted >= {1'b0, divisor});
    wire [24:0] div_next = div_bit ? div_shifted - {1'b0, divisor} : div_shifted;

    // Bring down two radicand bits. If the trial fits, subtract and
    // append a 1 to the partial root; otherwise append a 0.
    wire [28:0] sqrt_shifted = {sqrt_rem[26:0], radicand[51:50]};
    wire [28:0] sqrt_trial = {1'b0, digits, 2'b01};
    wire sqrt_bit = (sqrt_shifted >= sqrt_trial);
    wire [28:0] sqrt_next = sqrt_bit ? sqrt_shifted - sqrt_trial : sqrt_shifted;

    assign o_busy = i_valid && ((state == RUN) || ((state == IDLE) && !special));

    always @(posedge i_clk) begin
        if (i_rst) begin
            state <= IDLE;
            remaining <= 5'd0;
            run_sqrt <= 1'b0;
            saved_sign <= 1'b0;
            saved_rm <= 3'd0;
            saved_exp <= 10'd0;
            digits <= 26'd0;
            divisor <= 24'd0;
            div_rem <= 25'd0;
            radicand <= 52'd0;
            sqrt_rem <= 29'd0;
        end else if ((state == IDLE) && i_valid && !special) begin
            state <= RUN;
            remaining <= i_sqrt ? 5'd26 : 5'd25;
            run_sqrt <= i_sqrt;
            saved_sign <= input_sign;
            saved_rm <= i_rm;
            saved_exp <= i_sqrt ? sqrt_exp : div_exp;
            digits <= i_sqrt ? 26'd0 : 26'd1;
            divisor <= norm_b;
            div_rem <= div_numerator - {1'b0, norm_b};
            radicand <= sqrt_input;
            sqrt_rem <= 29'd0;
        end else if (state == RUN) begin
            digits <= {digits[24:0], (run_sqrt ? sqrt_bit : div_bit)};
            if (run_sqrt) begin
                radicand <= {radicand[49:0], 2'b00};
                sqrt_rem <= sqrt_next;
            end else begin
                div_rem <= div_next;
            end
            remaining <= remaining - 5'd1;
            if (remaining == 5'd1)
                state <= DONE;
        end else if ((state == DONE) && i_advance) begin
            state <= IDLE;
        end
    end

    wire sticky = run_sqrt ? (|sqrt_rem) : (|div_rem);
    wire [31:0] rounded;
    wire [4:0] rounded_flags;
    fpu_round rounder (
        .i_sign(saved_sign), .i_rm(saved_rm), .i_to_int(1'b0),
        .i_exp(saved_exp), .i_mag_grs({8'd0, digits, sticky}),
        .o_fp_result(rounded), .o_int_mag(), .o_flags(rounded_flags)
    );

    assign o_result = !i_valid ? 32'd0 :
                      (state == DONE) ? rounded :
                      (state == IDLE && special) ? special_result : 32'd0;
    assign o_flags = !i_valid ? 5'd0 :
                     (state == DONE) ? rounded_flags :
                     (state == IDLE && special) ? special_flags : 5'd0;
endmodule

`default_nettype wire
