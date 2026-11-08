// The immediate generator is responsible for decoding the 32-bit
// sign-extended immediate from the incoming instruction word. It is a purely
// combinational block that is expected to be embedded in the instruction
// decoder.
module imm (
    // Input instruction word. This is used to extract the relevant immediate
    // bits and assemble them into the final immediate.
    input  wire [31:0] i_inst,
    // Instruction format, determined by the instruction decoder based on the
    // opcode. This is one-hot encoded according to the following format:
    // [0] R-type
    // [1] I-type
    // [2] S-type
    // [3] B-type
    // [4] U-type
    // [5] J-type
    // Because the R-type format does not have an immediate, the output
    // immediate can be treated as a don't-care under this case.
    input  wire [ 5:0] i_format,
    // Output 32-bit immediate, sign-extended from the immediate bitstring.
    output wire [31:0] o_immediate
);
    // Your implementation goes under here
    // ------------------------------------

    reg [31:0] immediate;

    always @(*) begin
        case (1'b1)
            i_format[1]: immediate = {{20{i_inst[31]}}, i_inst[31:20]}; // I-type
            i_format[2]: immediate = {{20{i_inst[31]}}, i_inst[31:25], i_inst[11:7]}; // S-type
            i_format[3]: immediate = {{20{i_inst[31]}}, i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0}; // B-type
            i_format[4]: immediate = {i_inst[31:12], 12'b0}; // U-type
            i_format[5]: immediate = {{12{i_inst[31]}}, i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0}; // J-type
            default:     immediate = 32'b0; // R-type / don't care
        endcase
    end

    assign o_immediate = immediate;

endmodule
