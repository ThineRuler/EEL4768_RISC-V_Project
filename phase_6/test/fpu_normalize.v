`default_nettype none

// Move the leading 1 to the MSB. The caller handles an all-zero input.
module fpu_normalize #(
    parameter WIDTH = 48
) (
    input  wire [WIDTH-1:0] i_mag,
    output wire [WIDTH-1:0] o_mag,
    output wire [5:0]       o_shift
);
    wire [WIDTH-1:0] stage [0:6];
    assign stage[0] = i_mag;

    genvar g;
    generate
        for (g = 0; g < 6; g = g + 1) begin : normalize_stage
            localparam STEP = 2 ** (5-g);
            if (STEP < WIDTH) begin : shift_fits
                assign o_shift[5-g] = !(|stage[g][WIDTH-1:WIDTH-STEP]);
                assign stage[g+1] = o_shift[5-g] ?
                                    {stage[g][WIDTH-STEP-1:0], {STEP{1'b0}}} : stage[g];
            end else begin : shift_too_wide
                assign o_shift[5-g] = 1'b0;
                assign stage[g+1] = stage[g];
            end
        end
    endgenerate
    assign o_mag = stage[6];
endmodule

`default_nettype wire
