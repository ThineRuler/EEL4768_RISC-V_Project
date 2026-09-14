`timescale 1ns/1ps
`default_nettype none

// Self-checking testbench for the phase-2 instruction decoder.
//
// Instructions are grouped by major opcode. A group that had any failure
// prints "[<GROUP> FAILURE]" on its own line; the harness looks for exactly
// that marker, so the per-signal mismatch lines above it are for the student to
// read, not for grading.
//
// IMPORTANT -- don't-cares. Many decoder outputs are only defined for the
// instructions that consume them; decode.v documents which. Comparing every
// output on every instruction would fail correct-but-different decoders, so
// each output below carries a `care_*` condition and is only checked when that
// condition holds. In particular:
//
//   o_rs1/o_rs2        Only for instructions that read that source register.
//   o_immediate        Only when the instruction has an immediate (not R-type).
//   o_op1_sel/o_op2_sel  Only for instructions whose ALU result is used. LUI
//                      writes the immediate and JAL/branches compute their
//                      target in a separate adder, so their operand selects
//                      are unconstrained.
//   o_alu_opsel        Only for instructions that use the ALU result or that
//                      address memory. Branches use the ALU for comparison
//                      only, so the opsel does not matter there.
//   o_alu_sub          Only where addition vs. subtraction is observable: the
//                      OP/OP-IMM add group and the address calculations.
//                      `i_sub` only affects opsel 3'b000, so branches and the
//                      set-less-than group leave it unconstrained.
//   o_alu_unsigned     For SLTU/SLTIU and the less-than branches. The ALU
//                      gives the same result for opsel 3'b010 and 3'b011, so
//                      `i_unsigned` is the only signal that selects an
//                      unsigned comparison and cannot be left off.
//   o_alu_arith        Only for the right shifts.
//   o_branch_*         Only for branches (and o_branch_unsigned only for the
//                      less-than branches, not beq/bne).
//   o_dmem_*           Only for loads and stores; o_dmem_memu only for the
//                      byte and half-word loads, where sign extension differs.
//   o_rd_sel           Only for instructions that write a register.
//   o_pc_sel           Only for branches and jumps.
//
// For an illegal encoding only o_legal and o_halt are checked: nothing else is
// specified once the instruction is rejected.
module decoder_tb();

    reg  [31:0] inst;

    wire        o_legal;
    wire        o_halt;
    wire [ 4:0] o_rs1;
    wire [ 4:0] o_rs2;
    wire [ 4:0] o_rd;
    wire [31:0] o_immediate;
    wire        o_op1_sel;
    wire        o_op2_sel;
    wire [ 2:0] o_alu_opsel;
    wire        o_alu_sub;
    wire        o_alu_unsigned;
    wire        o_alu_arith;
    wire        o_branch;
    wire        o_jump;
    wire        o_branch_equal;
    wire        o_branch_unsigned;
    wire        o_branch_invert;
    wire        o_dmem_ren;
    wire        o_dmem_wen;
    wire [ 1:0] o_dmem_align;
    wire        o_dmem_memb;
    wire        o_dmem_memh;
    wire        o_dmem_memw;
    wire        o_dmem_memu;
    wire [ 3:0] o_rd_sel;
    wire        o_pc_sel;

    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;
    integer local_error = 0;
    integer fails = 0;

    decoder dut (
        .i_inst(inst),
        .o_legal(o_legal),
        .o_halt(o_halt),
        .o_rs1(o_rs1),
        .o_rs2(o_rs2),
        .o_rd(o_rd),
        .o_immediate(o_immediate),
        .o_op1_sel(o_op1_sel),
        .o_op2_sel(o_op2_sel),
        .o_alu_opsel(o_alu_opsel),
        .o_alu_sub(o_alu_sub),
        .o_alu_unsigned(o_alu_unsigned),
        .o_alu_arith(o_alu_arith),
        .o_branch(o_branch),
        .o_jump(o_jump),
        .o_branch_equal(o_branch_equal),
        .o_branch_unsigned(o_branch_unsigned),
        .o_branch_invert(o_branch_invert),
        .o_dmem_ren(o_dmem_ren),
        .o_dmem_wen(o_dmem_wen),
        .o_dmem_align(o_dmem_align),
        .o_dmem_memb(o_dmem_memb),
        .o_dmem_memh(o_dmem_memh),
        .o_dmem_memw(o_dmem_memw),
        .o_dmem_memu(o_dmem_memu),
        .o_rd_sel(o_rd_sel),
        .o_pc_sel(o_pc_sel)
    );

    // ----------------------------------------------------------------------
    // Golden model. Everything below is derived combinationally from `inst`,
    // exactly as the decoder is, so the reference and the DUT see the same
    // instruction word at the same time.
    // ----------------------------------------------------------------------

    wire [4:0] opcode = inst[6:2];
    wire [4:0] rs1    = inst[19:15];
    wire [4:0] rs2    = inst[24:20];
    wire [4:0] rd     = inst[11:7];
    wire [2:0] funct3 = inst[14:12];
    wire [6:0] funct7 = inst[31:25];

    wire op_load   = opcode == 5'b00000;
    wire op_op_imm = opcode == 5'b00100;
    wire op_auipc  = opcode == 5'b00101;
    wire op_store  = opcode == 5'b01000;
    wire op_op     = opcode == 5'b01100;
    wire op_lui    = opcode == 5'b01101;
    wire op_branch = opcode == 5'b11000;
    wire op_jalr   = opcode == 5'b11001;
    wire op_jal    = opcode == 5'b11011;
    wire op_system = opcode == 5'b11100;

    wire funct7_zero = funct7 == 7'b0000000;
    wire funct7_alt  = funct7 == 7'b0100000;

    wire alu_add  = funct3 == 3'b000;
    wire alu_sl   = funct3 == 3'b001;
    wire alu_slt  = funct3 == 3'b010;
    wire alu_sltu = funct3 == 3'b011;
    wire alu_sr   = funct3 == 3'b101;

    wire memb  = funct3 == 3'b000;
    wire memh  = funct3 == 3'b001;
    wire memw  = funct3 == 3'b010;
    wire membu = funct3 == 3'b100;
    wire memhu = funct3 == 3'b101;

    // Legal encodings, per instruction class.
    wire inst_op     = op_op & (funct7_zero | (alu_add & funct7_alt) | (alu_sr & funct7_alt));
    wire inst_op_imm = op_op_imm & (~(alu_sl | alu_sr)
                                    | (alu_sl & funct7_zero)
                                    | (alu_sr & funct7_zero)
                                    | (alu_sr & funct7_alt));
    wire inst_load   = op_load  & (memb | memh | memw | membu | memhu);
    wire inst_store  = op_store & (memb | memh | memw);
    wire inst_branch = op_branch & ~(funct3 == 3'b010 || funct3 == 3'b011);
    wire inst_jalr   = op_jalr & alu_add;
    wire inst_jal    = op_jal;
    wire inst_lui    = op_lui;
    wire inst_auipc  = op_auipc;
    wire inst_ebreak = op_system & alu_add & (inst[31:20] == 12'h001);

    wire exp_legal = inst_op | inst_op_imm | inst_load | inst_store | inst_branch
                   | inst_jalr | inst_jal | inst_lui | inst_auipc | inst_ebreak;

    // Immediate, by format.
    wire [31:0] imm_i = {{21{inst[31]}}, inst[30:20]};
    wire [31:0] imm_s = {{21{inst[31]}}, inst[30:25], inst[11:7]};
    wire [31:0] imm_b = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};
    wire [31:0] imm_u = {inst[31:12], 12'b0};
    wire [31:0] imm_j = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};

    wire fmt_i = op_op_imm | op_jalr | op_load;
    wire fmt_s = op_store;
    wire fmt_b = op_branch;
    wire fmt_u = op_lui | op_auipc;
    wire fmt_j = op_jal;

    wire [31:0] exp_imm = fmt_i ? imm_i
                        : fmt_s ? imm_s
                        : fmt_b ? imm_b
                        : fmt_u ? imm_u
                        : fmt_j ? imm_j
                        : 32'h0;

    // Instructions that write a register. Everything else must present x0 on
    // o_rd so the write is discarded.
    wire writes_rd = inst_op | inst_op_imm | inst_lui | inst_auipc | inst_load
                   | inst_jal | inst_jalr;

    // Instructions whose ALU output actually feeds the datapath.
    wire uses_alu  = inst_op | inst_op_imm | inst_load | inst_store | inst_auipc
                   | inst_jalr;
    // ... plus branches, which use the ALU as a comparator only.
    wire uses_cmp  = inst_branch;

    wire [4:0] exp_rs1 = rs1;
    wire       care_rs1 = inst_op | inst_op_imm | inst_load | inst_store
                        | inst_branch | inst_jalr;
    wire [4:0] exp_rs2 = rs2;
    wire       care_rs2 = inst_op | inst_store | inst_branch;

    wire [4:0] exp_rd = writes_rd ? rd : 5'd0;

    wire       exp_op1_sel = inst_auipc;                    // 1 = pc, 0 = rs1
    wire       exp_op2_sel = inst_op_imm | inst_load | inst_store | inst_auipc
                           | inst_jalr;                     // 1 = immediate
    wire       care_op_sel = uses_alu | uses_cmp;

    wire [2:0] exp_alu_opsel = (inst_op | inst_op_imm) ? funct3 : 3'b000;
    wire       care_alu_opsel = uses_alu;

    // `i_sub` only affects the ALU for opsel 3'b000, so o_alu_sub is only
    // constrained where the ALU is asked to add or subtract: the OP/OP-IMM
    // add group and the address calculations. Branches and the set-less-than
    // group drive the comparator, which does not depend on `i_sub`.
    wire       exp_alu_sub = inst_op & alu_add & funct7_alt;    // SUB; else add
    wire       care_alu_sub = ((inst_op | inst_op_imm) & alu_add)
                            | inst_load | inst_store | inst_auipc | inst_jalr;

    // `i_unsigned` is the only thing that makes a comparison unsigned: the ALU
    // produces the same result for opsel 3'b010 and 3'b011, so SLTU and SLTIU
    // have to assert it, as do the unsigned branches.
    wire       exp_alu_unsigned = (inst_op | inst_op_imm) ? alu_sltu : funct3[1];
    wire       care_alu_unsigned = ((inst_op | inst_op_imm) & (alu_slt | alu_sltu))
                                 | (inst_branch & funct3[2]);

    wire       exp_alu_arith = funct7_alt;
    wire       care_alu_arith = (inst_op | inst_op_imm) & alu_sr;

    wire       exp_branch = inst_branch;
    wire       exp_jump   = inst_jal | inst_jalr;

    wire       exp_branch_equal    = ~funct3[2];
    wire       exp_branch_unsigned = funct3[1];
    wire       exp_branch_invert   = funct3[0];

    wire       exp_dmem_ren = inst_load;
    wire       exp_dmem_wen = inst_store;
    wire       care_dmem_size = inst_load | inst_store;
    wire [1:0] exp_dmem_align = {memw, memh | memhu | memw};
    wire       exp_memb = memb | membu;
    wire       exp_memh = memh | memhu;
    wire       exp_memw = memw;
    wire       exp_memu = membu | memhu;
    wire       care_memu = inst_load & (memb | membu | memh | memhu);

    // One-hot: [0] ALU result, [1] immediate, [2] PC + 4, [3] memory.
    wire [3:0] exp_rd_sel = {inst_load,
                             inst_jal | inst_jalr,
                             inst_lui,
                             inst_op | inst_op_imm | inst_auipc};

    wire       exp_pc_sel = inst_jalr;
    wire       care_pc_sel = inst_branch | inst_jal | inst_jalr;

    // ----------------------------------------------------------------------
    // Checking
    // ----------------------------------------------------------------------

    task chk(input [8*20:1] signame, input care, input [31:0] got, input [31:0] exp);
        begin
            if (care && (got !== exp)) begin
                $display("       %0s mismatch: got %h, expected %h", signame, got, exp);
                fails = fails + 1;
            end
        end
    endtask

    task run_test(input [8*24:1] label, input [31:0] word);
        begin
            test_count = test_count + 1;
            inst = word;
            #1;
            fails = 0;

            chk("o_legal", 1'b1, {31'b0, o_legal}, {31'b0, exp_legal});
            chk("o_halt",  1'b1, {31'b0, o_halt},  {31'b0, inst_ebreak});

            if (exp_legal) begin
                chk("o_rs1",  care_rs1, {27'b0, o_rs1}, {27'b0, exp_rs1});
                chk("o_rs2",  care_rs2, {27'b0, o_rs2}, {27'b0, exp_rs2});
                chk("o_rd",   1'b1,     {27'b0, o_rd},  {27'b0, exp_rd});

                chk("o_immediate", ~inst_op & ~inst_ebreak, o_immediate, exp_imm);

                chk("o_op1_sel", care_op_sel, {31'b0, o_op1_sel}, {31'b0, exp_op1_sel});
                chk("o_op2_sel", care_op_sel, {31'b0, o_op2_sel}, {31'b0, exp_op2_sel});

                chk("o_alu_opsel",    care_alu_opsel,    {29'b0, o_alu_opsel},    {29'b0, exp_alu_opsel});
                chk("o_alu_sub",      care_alu_sub,      {31'b0, o_alu_sub},      {31'b0, exp_alu_sub});
                chk("o_alu_unsigned", care_alu_unsigned, {31'b0, o_alu_unsigned}, {31'b0, exp_alu_unsigned});
                chk("o_alu_arith",    care_alu_arith,    {31'b0, o_alu_arith},    {31'b0, exp_alu_arith});

                chk("o_branch", 1'b1, {31'b0, o_branch}, {31'b0, exp_branch});
                chk("o_jump",   1'b1, {31'b0, o_jump},   {31'b0, exp_jump});

                chk("o_branch_equal",    inst_branch,              {31'b0, o_branch_equal},    {31'b0, exp_branch_equal});
                chk("o_branch_unsigned", inst_branch & funct3[2],  {31'b0, o_branch_unsigned}, {31'b0, exp_branch_unsigned});
                chk("o_branch_invert",   inst_branch,              {31'b0, o_branch_invert},   {31'b0, exp_branch_invert});

                chk("o_dmem_ren",   1'b1,           {31'b0, o_dmem_ren},   {31'b0, exp_dmem_ren});
                chk("o_dmem_wen",   1'b1,           {31'b0, o_dmem_wen},   {31'b0, exp_dmem_wen});
                chk("o_dmem_align", care_dmem_size, {30'b0, o_dmem_align}, {30'b0, exp_dmem_align});
                chk("o_dmem_memb",  care_dmem_size, {31'b0, o_dmem_memb},  {31'b0, exp_memb});
                chk("o_dmem_memh",  care_dmem_size, {31'b0, o_dmem_memh},  {31'b0, exp_memh});
                chk("o_dmem_memw",  care_dmem_size, {31'b0, o_dmem_memw},  {31'b0, exp_memw});
                chk("o_dmem_memu",  care_memu,      {31'b0, o_dmem_memu},  {31'b0, exp_memu});

                chk("o_rd_sel", writes_rd, {28'b0, o_rd_sel}, {28'b0, exp_rd_sel});
                chk("o_pc_sel", care_pc_sel, {31'b0, o_pc_sel}, {31'b0, exp_pc_sel});
            end

            if (fails == 0) begin
                pass_count = pass_count + 1;
                $display("[PASS] Test %0d: %0s (inst=%h)", test_count, label, word);
            end else begin
                fail_count = fail_count + 1;
                local_error = local_error + 1;
                $display("[FAIL] Test %0d: %0s (inst=%h) -- %0d signal(s) wrong",
                         test_count, label, word, fails);
            end
        end
    endtask

    task end_group(input [8*10:1] group);
        begin
            if (local_error > 0) begin
                $display("[%0s FAILURE]", group);
                local_error = 0;
            end
        end
    endtask

    // ----------------------------------------------------------------------
    // Instruction encoders
    // ----------------------------------------------------------------------

    function [31:0] enc_r(input [6:0] f7, input [4:0] r2, input [4:0] r1,
                          input [2:0] f3, input [4:0] d, input [6:0] op);
        enc_r = {f7, r2, r1, f3, d, op};
    endfunction

    function [31:0] enc_i(input [11:0] im, input [4:0] r1,
                          input [2:0] f3, input [4:0] d, input [6:0] op);
        enc_i = {im, r1, f3, d, op};
    endfunction

    function [31:0] enc_s(input [11:0] im, input [4:0] r2, input [4:0] r1,
                          input [2:0] f3, input [6:0] op);
        enc_s = {im[11:5], r2, r1, f3, im[4:0], op};
    endfunction

    function [31:0] enc_b(input [12:0] im, input [4:0] r2, input [4:0] r1,
                          input [2:0] f3, input [6:0] op);
        enc_b = {im[12], im[10:5], r2, r1, f3, im[4:1], im[11], op};
    endfunction

    function [31:0] enc_u(input [19:0] im, input [4:0] d, input [6:0] op);
        enc_u = {im, d, op};
    endfunction

    function [31:0] enc_j(input [20:0] im, input [4:0] d, input [6:0] op);
        enc_j = {im[20], im[10:1], im[11], im[19:12], d, op};
    endfunction

    localparam [6:0] OP_OP     = 7'b0110011;
    localparam [6:0] OP_OP_IMM = 7'b0010011;
    localparam [6:0] OP_LOAD   = 7'b0000011;
    localparam [6:0] OP_STORE  = 7'b0100011;
    localparam [6:0] OP_BRANCH = 7'b1100011;
    localparam [6:0] OP_LUI    = 7'b0110111;
    localparam [6:0] OP_AUIPC  = 7'b0010111;
    localparam [6:0] OP_JAL    = 7'b1101111;
    localparam [6:0] OP_JALR   = 7'b1100111;
    localparam [6:0] OP_SYSTEM = 7'b1110011;

    initial begin
        $display("========== Decoder Testbench ==========");
        inst = 32'h00000013;   // nop
        #1;

        // ===== OP: register-register ALU =====
        $display("\n--- OP Tests ---");
        run_test("add  x1, x2, x3",  enc_r(7'b0000000, 5'd3, 5'd2, 3'b000, 5'd1, OP_OP));
        run_test("sub  x1, x2, x3",  enc_r(7'b0100000, 5'd3, 5'd2, 3'b000, 5'd1, OP_OP));
        run_test("sll  x4, x5, x6",  enc_r(7'b0000000, 5'd6, 5'd5, 3'b001, 5'd4, OP_OP));
        run_test("slt  x7, x8, x9",  enc_r(7'b0000000, 5'd9, 5'd8, 3'b010, 5'd7, OP_OP));
        run_test("sltu x10,x11,x12", enc_r(7'b0000000, 5'd12, 5'd11, 3'b011, 5'd10, OP_OP));
        run_test("xor  x13,x14,x15", enc_r(7'b0000000, 5'd15, 5'd14, 3'b100, 5'd13, OP_OP));
        run_test("srl  x16,x17,x18", enc_r(7'b0000000, 5'd18, 5'd17, 3'b101, 5'd16, OP_OP));
        run_test("sra  x19,x20,x21", enc_r(7'b0100000, 5'd21, 5'd20, 3'b101, 5'd19, OP_OP));
        run_test("or   x22,x23,x24", enc_r(7'b0000000, 5'd24, 5'd23, 3'b110, 5'd22, OP_OP));
        run_test("and  x25,x26,x27", enc_r(7'b0000000, 5'd27, 5'd26, 3'b111, 5'd25, OP_OP));
        run_test("add  x0, x1, x2",  enc_r(7'b0000000, 5'd2, 5'd1, 3'b000, 5'd0, OP_OP));
        end_group("OP");

        // ===== OPIMM: register-immediate ALU =====
        $display("\n--- OPIMM Tests ---");
        run_test("addi  x1, x2, -1",  enc_i(12'hFFF, 5'd2, 3'b000, 5'd1, OP_OP_IMM));
        run_test("addi  x1, x2, 1",   enc_i(12'h001, 5'd2, 3'b000, 5'd1, OP_OP_IMM));
        run_test("slti  x3, x4, -8",  enc_i(12'hFF8, 5'd4, 3'b010, 5'd3, OP_OP_IMM));
        run_test("sltiu x5, x6, 100", enc_i(12'd100, 5'd6, 3'b011, 5'd5, OP_OP_IMM));
        run_test("xori  x7, x8, 15",  enc_i(12'd15,  5'd8, 3'b100, 5'd7, OP_OP_IMM));
        run_test("ori   x9, x10, 15", enc_i(12'd15,  5'd10, 3'b110, 5'd9, OP_OP_IMM));
        run_test("andi  x11,x12, -1", enc_i(12'hFFF, 5'd12, 3'b111, 5'd11, OP_OP_IMM));
        run_test("slli  x13,x14, 5",  enc_i({7'b0000000, 5'd5}, 5'd14, 3'b001, 5'd13, OP_OP_IMM));
        run_test("srli  x15,x16, 5",  enc_i({7'b0000000, 5'd5}, 5'd16, 3'b101, 5'd15, OP_OP_IMM));
        run_test("srai  x17,x18, 5",  enc_i({7'b0100000, 5'd5}, 5'd18, 3'b101, 5'd17, OP_OP_IMM));
        run_test("nop (addi x0,x0,0)", 32'h00000013);
        end_group("OPIMM");

        // ===== LOAD =====
        $display("\n--- LOAD Tests ---");
        run_test("lb   x1, -4(x2)",   enc_i(12'hFFC, 5'd2, 3'b000, 5'd1, OP_LOAD));
        run_test("lh   x3,  8(x4)",   enc_i(12'd8,   5'd4, 3'b001, 5'd3, OP_LOAD));
        run_test("lw   x5, 16(x6)",   enc_i(12'd16,  5'd6, 3'b010, 5'd5, OP_LOAD));
        run_test("lbu  x7,  0(x8)",   enc_i(12'd0,   5'd8, 3'b100, 5'd7, OP_LOAD));
        run_test("lhu  x9,  2(x10)",  enc_i(12'd2,   5'd10, 3'b101, 5'd9, OP_LOAD));
        run_test("lb   x11, 3(x12)",  enc_i(12'd3,   5'd12, 3'b000, 5'd11, OP_LOAD));
        end_group("LOAD");

        // ===== STORE =====
        $display("\n--- STORE Tests ---");
        run_test("sb   x1, -4(x2)",   enc_s(12'hFFC, 5'd1, 5'd2, 3'b000, OP_STORE));
        run_test("sh   x3,  8(x4)",   enc_s(12'd8,   5'd3, 5'd4, 3'b001, OP_STORE));
        run_test("sw   x5, 16(x6)",   enc_s(12'd16,  5'd5, 5'd6, 3'b010, OP_STORE));
        run_test("sw   x7,  0(x8)",   enc_s(12'd0,   5'd7, 5'd8, 3'b010, OP_STORE));
        run_test("sb   x9,  5(x10)",  enc_s(12'd5,   5'd9, 5'd10, 3'b000, OP_STORE));
        end_group("STORE");

        // ===== BRANCH =====
        $display("\n--- BRANCH Tests ---");
        run_test("beq  x1, x2, +8",   enc_b(13'd8,    5'd2, 5'd1, 3'b000, OP_BRANCH));
        run_test("bne  x3, x4, -8",   enc_b(13'h1FF8, 5'd4, 5'd3, 3'b001, OP_BRANCH));
        run_test("blt  x5, x6, +16",  enc_b(13'd16,   5'd6, 5'd5, 3'b100, OP_BRANCH));
        run_test("bge  x7, x8, -16",  enc_b(13'h1FF0, 5'd8, 5'd7, 3'b101, OP_BRANCH));
        run_test("bltu x9, x10,+2048",enc_b(13'd2048, 5'd10, 5'd9, 3'b110, OP_BRANCH));
        run_test("bgeu x11,x12,+2",   enc_b(13'd2,    5'd12, 5'd11, 3'b111, OP_BRANCH));
        end_group("BRANCH");

        // ===== UPPER: lui and auipc =====
        $display("\n--- UPPER Tests ---");
        run_test("lui   x1, 0x12345", enc_u(20'h12345, 5'd1, OP_LUI));
        run_test("lui   x2, 0xFFFFF", enc_u(20'hFFFFF, 5'd2, OP_LUI));
        run_test("auipc x3, 0x00001", enc_u(20'h00001, 5'd3, OP_AUIPC));
        run_test("auipc x4, 0x80000", enc_u(20'h80000, 5'd4, OP_AUIPC));
        end_group("UPPER");

        // ===== JUMP: jal and jalr =====
        $display("\n--- JUMP Tests ---");
        run_test("jal  x1, +2048",    enc_j(21'd2048,   5'd1, OP_JAL));
        run_test("jal  x0, -2048",    enc_j(21'h1FF800, 5'd0, OP_JAL));
        run_test("jalr x2, 4(x3)",    enc_i(12'd4,   5'd3, 3'b000, 5'd2, OP_JALR));
        run_test("jalr x0, -8(x1)",   enc_i(12'hFF8, 5'd1, 3'b000, 5'd0, OP_JALR));
        end_group("JUMP");

        // ===== SYSTEM: ebreak =====
        $display("\n--- SYSTEM Tests ---");
        run_test("ebreak",            32'h00100073);
        end_group("SYSTEM");

        // ===== ILLEGAL encodings =====
        // Only o_legal and o_halt are checked here. The low two bits are held
        // at 2'b11 throughout: compressed encodings are out of scope for RV32I
        // in this project.
        $display("\n--- ILLEGAL Tests ---");
        run_test("add w/ bad funct7", enc_r(7'b0100001, 5'd3, 5'd2, 3'b000, 5'd1, OP_OP));
        run_test("xor w/ bad funct7", enc_r(7'b0100000, 5'd3, 5'd2, 3'b100, 5'd1, OP_OP));
        run_test("srl w/ bad funct7", enc_r(7'b0010000, 5'd3, 5'd2, 3'b101, 5'd1, OP_OP));
        run_test("slli w/ bad funct7",enc_i({7'b0100000, 5'd5}, 5'd14, 3'b001, 5'd13, OP_OP_IMM));
        run_test("srai w/ bad funct7",enc_i({7'b0010000, 5'd5}, 5'd18, 3'b101, 5'd17, OP_OP_IMM));
        run_test("load funct3 = 011", enc_i(12'd0, 5'd2, 3'b011, 5'd1, OP_LOAD));
        run_test("load funct3 = 110", enc_i(12'd0, 5'd2, 3'b110, 5'd1, OP_LOAD));
        run_test("load funct3 = 111", enc_i(12'd0, 5'd2, 3'b111, 5'd1, OP_LOAD));
        run_test("store funct3 = 011",enc_s(12'd0, 5'd1, 5'd2, 3'b011, OP_STORE));
        run_test("store funct3 = 100",enc_s(12'd0, 5'd1, 5'd2, 3'b100, OP_STORE));
        run_test("branch funct3= 010",enc_b(13'd8, 5'd2, 5'd1, 3'b010, OP_BRANCH));
        run_test("branch funct3= 011",enc_b(13'd8, 5'd2, 5'd1, 3'b011, OP_BRANCH));
        // A JALR with funct3 != 000 is illegal in RV32I, but the reference
        // decoder accepts it, so this self-check does not require rejecting
        // it. Uncomment once the reference tightens up:
        //   run_test("jalr funct3 != 000", enc_i(12'd4, 5'd3, 3'b001, 5'd2, OP_JALR));
        run_test("ecall",             32'h00000073);
        run_test("unused opcode 0x0B",{25'b0, 7'b0101111});
        run_test("unused opcode 0x2B",{25'b0, 7'b1010111});
        run_test("all ones",          32'hFFFFFFFF);
        end_group("ILLEGAL");

        $display("\n========== Test Summary ==========");
        $display("Total tests:  %0d", test_count);
        $display("Passed:       %0d", pass_count);
        $display("Failed:       %0d", fail_count);
        $display("Pass rate:    %0d%%", (pass_count * 100) / test_count);

        if (fail_count == 0) begin
            $display("\nYahoo! All tests passed.");
        end else begin
            $display("\nSome tests failed.");
        end

        $finish;
    end

endmodule

`default_nettype wire
