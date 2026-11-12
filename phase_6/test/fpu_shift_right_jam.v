`default_nettype none

// Right shift with every discarded bit folded into the output LSB.
module fpu_shift_right_jam #(
    parameter WIDTH = 27
) (
    input  wire [WIDTH-1:0] i_value,
    input  wire [5:0] i_shift,
    output wire [WIDTH-1:0] o_value
);
    wire [WIDTH-1:0] stage [0:6];
    assign stage[0] = i_value;
    genvar g;
    generate
        for (g = 0; g < 6; g = g + 1) begin : shift_stage
            localparam STEP = 2 ** g;
            wire [WIDTH-1:0] shifted;
            if (STEP < WIDTH-1) begin : partial
                assign shifted = {{STEP{1'b0}}, stage[g][WIDTH-1:STEP+1],
                                  (|stage[g][STEP:0])};
            end else begin : all_bits
                assign shifted = {{(WIDTH-1){1'b0}}, (|stage[g])};
            end
            assign stage[g+1] = i_shift[g] ? shifted : stage[g];
        end
    endgenerate
    assign o_value = stage[6];
endmodule

`default_nettype wire
