module imm_tb();
    reg clk;
    
    reg [31:0] inst;
    reg [5:0] format;
    // [0] R-type
    // [1] I-type
    // [2] S-type
    // [3] B-type
    // [4] U-type
    // [5] J-type
    wire [31:0] dut_immediate;

    imm dut (
        .i_inst(inst),
        .i_format(format),
        .o_immediate(dut_immediate)
    );

    always #5 clk = ~clk;

    initial begin
        // init
        clk = 0;
        inst = 32'b0;
        format = 6'b0;
        #10;

        // I-type 1 (addi, etc.)
        inst = 32'b111111111111_00000_000_00000_0010011;
        format = 6'b000010;
        #10;
        if (dut_immediate !== {{21{inst[31]}}, inst[30:20]}) $display("TEST FAILED: I-type 1 (addi)");

        // I-type 2 (load)
        inst = 32'b000000000101_00010_010_00001_0000011;
        format = 6'b000010;
        #10;
        if (dut_immediate !== {{21{inst[31]}}, inst[30:20]}) $display("TEST FAILED: I-type 2 (load)");

        // I-type 3 (jalr)
        inst = 32'b111111111000_00001_000_00010_1100111;
        format = 6'b000010;
        #10;
        if (dut_immediate !== {{21{inst[31]}}, inst[30:20]}) $display("TEST FAILED: I-type 3 (jalr)");

        // S-type (store)
        inst = 32'b1111111_00011_00100_010_00101_0100011; 
        format = 6'b000100;
        #10;
        if (dut_immediate !== {{21{inst[31]}}, inst[30:25], inst[11:7]}) $display("TEST FAILED: S-type (store)");

        // B-type (branch)
        inst = 32'b1_000000_00110_00101_000_00011_1100011;
        format = 6'b001000;
        #10;
        if (dut_immediate !== {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0}) $display("TEST FAILED: B-type (branch)");

        // U-type (lui, auipc)
        inst = 32'b000000000001_00000000_00000_0110111;
        format = 6'b010000;
        #10;
        if (dut_immediate !== {inst[31:12], 12'b0}) $display("TEST FAILED: U-type (lui)");
        
        // J-type (jal)
        inst = 32'b0_00000000_0001_00000_00000_1101111;
        format = 6'b100000;
        #10;
        if (dut_immediate !== {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}) $display("TEST FAILED: J-type (jal)");

        $finish;

    end

endmodule