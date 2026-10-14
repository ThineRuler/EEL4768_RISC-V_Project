`default_nettype none

// Remember to instantiate the imm in this module

module decoder (
    // Input instruction word.
    input  wire [31:0] i_inst,
    // Asserted if the instruction was decoded as a legal instruction. It is
    // important that the decoder not accept any illegal instruction
    // encodings as this could lead to undefined behavior in the processor
    // which is a safety hazard.
    output wire        o_legal,
    // Indicates that the instruction is an ebreak and should halt execution.
    output wire        o_halt,
    // First source register address.
    // For instructions that do not use a source register, this is effectively
    // a don't care because reading unused registers does not have any side
    // effects (and we don't care about power usage, really).
    output wire [ 4:0] o_rs1,
    // Second source register address.
    // Similarly to o_rs1, this is a don't care for instructions that do not
    // read a (second) source register.
    output wire [ 4:0] o_rs2,
    // Destination register address.
    // For instructions that do not write to a register, this must be set to
    // x0 so the value is discarded. This avoids the need for a separate write
    // enable since discard behavior must be present anyway.
    output wire [ 4:0] o_rd,
    // 32-bit immediate value, decoded from the instruction word. For R-type
    // instructions that do not use an immediate, this is a don't care.
    output wire [31:0] o_immediate,
    // Selects whether the first operand for the ALU is fed by the first
    // register source (rs1) or the current pc.
    // When asserted, the second operand is the immediate.
    output wire        o_op1_sel,
    // Selects whether the second operand for the ALU is fed by the second
    // register source (rs2) or the immediate.
    // When asserted, the second operand is the immediate.
    output wire        o_op2_sel,
    // Major opsel for the ALU. See ALU documentation for the encoding.
    output wire [ 2:0] o_alu_opsel,
    // Minor opsel flags for the ALU. See ALU documentation for the encoding.
    output wire        o_alu_sub,
    output wire        o_alu_unsigned,
    output wire        o_alu_arith,
    // If asserted, the instruction is a branch instruction and the PC should
    // be updated to the target address if the branch condition is met.
    output wire        o_branch,
    // If asserted, the instruction is a jump instruction and the PC should
    // be updated to the target address unconditionally.
    output wire        o_jump,
    // When asserted, the branch comparator checks for equality. When not
    // asserted, it checks for less than [unsigned].
    output wire        o_branch_equal,
    // When asserted, the branch comparator treats the less than comparison
    // operands as unsigned. This is only used when `!o_branch_equal`.
    output wire        o_branch_unsigned,
    // When asserted, the branch condition is inverted.
    // Equality -> inequality, less than -> greater than or equal.
    output wire        o_branch_invert,
    // When asserted, the instruction will load from memory.
    output wire        o_dmem_ren,
    // When asserted, the instruction will store to memory.
    output wire        o_dmem_wen,
    // This 2-bit mask selects which LSBs of the memory address should be
    // checked for alignment. This is because byte and half-word accesses need
    // only be 1-byte and 2-byte aligned, respectively.
    output wire [ 1:0] o_dmem_align,
    // These 3 bits select the size of the memory access.
    // They are effectively one-hot encoded.
    output wire        o_dmem_memb,
    output wire        o_dmem_memh,
    output wire        o_dmem_memw,
    // If asserted, the (byte or half-word) memory access is unsigned and the
    // load should be zero-extended to 32 bits instead of sign-extended.
    output wire        o_dmem_memu,
    // Selects the data to write to the destination register, one-hot.
    // [0] = ALU result
    // [1] = immediate
    // [2] = PC + 4
    // [3] = memory
    output wire [ 3:0] o_rd_sel,
    // If asserted, the PC jumps to the target address calculated by the ALU
    // rather than directly to the PC + immediate. This is used for JALR.
    output wire        o_pc_sel
);
    // Your implementation goes under here
    // ------------------------------------

    // Bitfield extraction
    wire [6:0] opcode = i_inst[6:0];
    wire [4:0] rd = i_inst[11:7];
    wire [2:0] funct3 = i_inst[14:12];
    wire [4:0] rs1 = i_inst[19:15];
    wire [4:0] rs2 = i_inst[24:20];
    wire [6:0] funct7 = i_inst[31:25];

    assign o_rs1 = rs1;
    assign o_rs2 = rs2;

    // Opcode constants for readability
    localparam OPCODE_R_TYPE = 7'b0110011;  // R-type instructions: add, sub, sll, slt, sltu, xor, srl, sra, or, and
    localparam OPCODE_I_TYPE = 7'b0010011;  // I-type instructions: addi, slli, slti, sltiu, xori, srli, srai, ori, andi
    localparam OPCODE_LOAD   = 7'b0000011;  // I-type load instructions: lb, lh, lw, lbu, lhu
    localparam OPCODE_STORE  = 7'b0100011;  // S-type instructions: sb, sh, sw
    localparam OPCODE_BRANCH = 7'b1100011;  // B-type instructions: beq, bne, blt, bge, bltu, bgeu
    localparam OPCODE_LUI    = 7'b0110111;  // U-type instruction: lui
    localparam OPCODE_AUIPC  = 7'b0010111;  // U-type instruction: auipc
    localparam OPCODE_JAL    = 7'b1101111;  // J-type instruction: jal
    localparam OPCODE_JALR   = 7'b1100111;  // I-type instruction: jalr
    localparam OPCODE_SYSTEM = 7'b1110011;  // System instruction: ebreak

    // Internal signals driven by combination logic (AI generated)
    reg        r_legal;
    reg        r_halt;
    reg        r_rd_wen;       // 1 if instruction writes to rd
    reg [ 5:0] r_imm_format;   // One-hot selector for imm module
    reg        r_op1_sel;
    reg        r_op2_sel;
    reg [ 2:0] r_alu_opsel;
    reg        r_alu_sub;
    reg        r_alu_unsigned;
    reg        r_alu_arith;
    reg        r_branch;
    reg        r_jump;
    reg        r_branch_equal;
    reg        r_branch_unsigned;
    reg        r_branch_invert;
    reg        r_dmem_ren;
    reg        r_dmem_wen;
    reg [ 1:0] r_dmem_align;
    reg        r_dmem_memb;
    reg        r_dmem_memh;
    reg        r_dmem_memw;
    reg        r_dmem_memu;
    reg [ 3:0] r_rd_sel;
    reg        r_pc_sel;

    // Combinational decoder logic
    always @(*) begin
        // Default values for all outputs
        r_legal = 1'b0;
        r_halt = 1'b0;
        r_rd_wen = 1'b0;
        r_imm_format = 6'b000001;
        r_op1_sel = 1'b0;
        r_op2_sel = 1'b0;
        r_alu_opsel = 3'b000;
        r_alu_sub = 1'b0;
        r_alu_unsigned = 1'b0;
        r_alu_arith = 1'b0;
        r_branch = 1'b0;
        r_jump = 1'b0;
        r_branch_equal = 1'b0;
        r_branch_unsigned = 1'b0;
        r_branch_invert = 1'b0;
        r_dmem_ren = 1'b0;
        r_dmem_wen = 1'b0;
        r_dmem_align = 2'b00;
        r_dmem_memb = 1'b0;
        r_dmem_memh = 1'b0;
        r_dmem_memw = 1'b0;
        r_dmem_memu = 1'b0;
        r_rd_sel = 4'b0000;
        r_pc_sel = 1'b0;

        case (opcode)
            OPCODE_R_TYPE: begin // Register-Register Arithmetic/Logic
                r_imm_format = 6'b000001; // R-type
                r_rd_wen     = 1'b1;
                r_rd_sel     = 4'b0001;   // ALU result
                r_alu_opsel  = funct3;
    
                // Check legal funct7 variations:
                // Normal ops require funct7 == 7'b0000000
                // SUB and SRA require funct7 == 7'b0100000
                case (funct7)
                    7'b0000000: begin
                        r_legal        = 1'b1;
                        r_alu_unsigned = (funct3 == 3'b011); // sltu
                    end
                    7'b0100000: begin
                        case (funct3)
                            3'b000: begin // sub
                                r_legal   = 1'b1;
                                r_alu_sub = 1'b1;
                            end
                            3'b101: begin // sra
                                r_legal     = 1'b1;
                                r_alu_arith = 1'b1;
                            end
                            default: r_legal = 1'b0;
                        endcase
                    end
                    default: r_legal = 1'b0;
                endcase
            end
    
            OPCODE_I_TYPE: begin // Register-Immediate Arithmetic/Logic
                r_imm_format = 6'b000010; // I-type
                r_rd_wen     = 1'b1;
                r_op2_sel    = 1'b1;      // Operand 2 is immediate
                r_rd_sel     = 4'b0001;   // ALU result
                r_alu_opsel  = funct3;
    
                case (funct3)
                    3'b001: begin // slli (shift amount is 5 bits, upper 7 must be 0)
                        case (funct7)
                            7'b0000000: r_legal = 1'b1;
                            default:    r_legal = 1'b0;
                        endcase
                    end
                    3'b101: begin // srli / srai
                        case (funct7)
                            7'b0000000: r_legal = 1'b1; // srli
                            7'b0100000: begin           // srai
                                r_legal     = 1'b1;
                                r_alu_arith = 1'b1;
                            end
                            default: r_legal = 1'b0;
                        endcase
                    end
                    default: begin // addi, slti, sltiu, xori, ori, andi
                        r_legal        = 1'b1;
                        r_alu_unsigned = (funct3 == 3'b011); // sltiu
                    end
                endcase
            end
    
            OPCODE_LOAD: begin // Load instructions
                r_imm_format = 6'b000010; // I-type
                r_rd_wen     = 1'b1;
                r_op2_sel    = 1'b1;      // ALU adds immediate to rs1
                r_alu_opsel  = 3'b000;    // ADD address
                r_dmem_ren   = 1'b1;      // Read memory
                r_rd_sel     = 4'b1000;   // Writeback from memory
    
                case (funct3)
                    3'b000: begin // lb
                        r_legal     = 1'b1;
                        r_dmem_memb = 1'b1;
                        r_dmem_align= 2'b00;
                    end
                    3'b001: begin // lh
                        r_legal     = 1'b1;
                        r_dmem_memh = 1'b1;
                        r_dmem_align= 2'b01;
                    end
                    3'b010: begin // lw
                        r_legal     = 1'b1;
                        r_dmem_memw = 1'b1;
                        r_dmem_align= 2'b11;
                    end
                    3'b100: begin // lbu
                        r_legal     = 1'b1;
                        r_dmem_memb = 1'b1;
                        r_dmem_memu = 1'b1;
                        r_dmem_align= 2'b00;
                    end
                    3'b101: begin // lhu
                        r_legal     = 1'b1;
                        r_dmem_memh = 1'b1;
                        r_dmem_memu = 1'b1;
                        r_dmem_align= 2'b01;
                    end
                    default: r_legal = 1'b0;
                endcase
            end
    
            OPCODE_STORE: begin // Store instructions
                r_imm_format = 6'b000100; // S-type
                r_op2_sel    = 1'b1;      // ALU adds immediate to rs1
                r_alu_opsel  = 3'b000;    // ADD address
                r_dmem_wen   = 1'b1;      // Write memory
                // r_rd_wen remains 0 (Stores don't write to rd!)
    
                case (funct3)
                    3'b000: begin // sb
                        r_legal      = 1'b1;
                        r_dmem_memb  = 1'b1;
                        r_dmem_align = 2'b00;
                    end
                    3'b001: begin // sh
                        r_legal      = 1'b1;
                        r_dmem_memh  = 1'b1;
                        r_dmem_align = 2'b01;
                    end
                    3'b010: begin // sw
                        r_legal      = 1'b1;
                        r_dmem_memw  = 1'b1;
                        r_dmem_align = 2'b11;
                    end
                    default: r_legal = 1'b0;
                endcase
            end
    
            OPCODE_BRANCH: begin // Conditional branches
                r_imm_format      = 6'b001000; // B-type
                r_branch          = 1'b1;
                r_branch_equal    = ~funct3[2];
                r_branch_unsigned = funct3[1];
                r_branch_invert   = funct3[0];
                r_alu_unsigned    = funct3[1]; // Per spec section 8
    
                // Legal funct3 for branches: 000, 001, 100, 101, 110, 111
                case (funct3)
                    3'b000, 3'b001, 3'b100, 3'b101, 3'b110, 3'b111: r_legal = 1'b1;
                    default:                                         r_legal = 1'b0;
                endcase
            end
    
            OPCODE_LUI: begin // Load Upper Immediate
                r_imm_format = 6'b010000; // U-type
                r_legal      = 1'b1;
                r_rd_wen     = 1'b1;
                r_rd_sel     = 4'b0010;   // Immediate directly into rd
            end
    
            OPCODE_AUIPC: begin // Add Upper Immediate to PC
                r_imm_format = 6'b010000; // U-type
                r_legal      = 1'b1;
                r_rd_wen     = 1'b1;
                r_op1_sel    = 1'b1;      // PC
                r_op2_sel    = 1'b1;      // Immediate
                r_alu_opsel  = 3'b000;    // ADD
                r_rd_sel     = 4'b0001;   // ALU result
            end
    
            OPCODE_JAL: begin // Jump and Link
                r_imm_format = 6'b100000; // J-type
                r_legal      = 1'b1;
                r_rd_wen     = 1'b1;
                r_jump       = 1'b1;
                r_pc_sel     = 1'b0;      // Target base is PC
                r_rd_sel     = 4'b0100;   // Write PC + 4 to rd
            end
    
            OPCODE_JALR: begin // Jump and Link Register
                r_imm_format = 6'b000010; // I-type
                case (funct3)
                    3'b000: begin
                        r_legal     = 1'b1;
                        r_rd_wen    = 1'b1;
                        r_op2_sel   = 1'b1;    // ALU adds rs1 + immediate
                        r_alu_opsel = 3'b000;  // ADD
                        r_jump      = 1'b1;
                        r_pc_sel    = 1'b1;    // Target base is calculated by ALU
                        r_rd_sel    = 4'b0100; // Write PC + 4 to rd
                    end
                    default: r_legal = 1'b0;
                endcase
            end
    
            OPCODE_SYSTEM: begin
                // Per spec: ONLY ebreak (32'h00100073) is legal and drives halt.
                // ecall (32'h00000073) is tested in the ILLEGAL group!
                case (i_inst)
                    32'h00100073: begin // ebreak
                        r_legal = 1'b1;
                        r_halt  = 1'b1;
                    end
                    default: begin
                        r_legal = 1'b0;
                    end
                endcase
            end
    
            default: begin
                r_legal = 1'b0;
            end
        endcase
    end

    // Output assignments
    assign o_legal = r_legal;
    assign o_halt = r_halt;
    assign o_rd = (r_legal && r_rd_wen) ? rd : 5'b00000; // If instruction doesn't write to rd or is illegal, set to x0
    assign o_op1_sel = r_op1_sel;
    assign o_op2_sel = r_op2_sel;
    assign o_alu_opsel = r_alu_opsel;
    assign o_alu_sub = r_alu_sub;
    assign o_alu_unsigned = r_alu_unsigned;
    assign o_alu_arith = r_alu_arith;
    assign o_branch = r_legal && r_branch;
    assign o_jump = r_legal && r_jump;
    assign o_branch_equal = r_branch_equal;
    assign o_branch_unsigned = r_branch_unsigned;
    assign o_branch_invert = r_branch_invert;
    assign o_dmem_ren = r_legal && r_dmem_ren;
    assign o_dmem_wen = r_legal && r_dmem_wen;
    assign o_dmem_align = r_dmem_align;
    assign o_dmem_memb = r_dmem_memb;
    assign o_dmem_memh = r_dmem_memh;
    assign o_dmem_memw = r_dmem_memw;
    assign o_dmem_memu = r_dmem_memu;
    assign o_rd_sel = r_rd_sel;
    assign o_pc_sel = r_pc_sel;

    // Instantiate the immediate generator
    imm imm_gen (
        .i_inst(i_inst),
        .i_format(r_imm_format),
        .o_immediate(o_immediate)
    );

endmodule

`default_nettype wire
