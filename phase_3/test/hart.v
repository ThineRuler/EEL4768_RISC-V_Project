`default_nettype none
`timescale 1ns / 1ps

// ============================================================================
// Module: hart
// Description: Single-Cycle RV32I Processor Core.
//              Fetches, decodes, executes, accesses memory, and writes back
//              in a single clock cycle. Exactly one instruction retires per
//              cycle (no bubbles, no pipeline stages).
// ============================================================================

module hart #(
    // Reset address: The program counter is initialized to this value on reset.
    // The first instruction fetched and retired after i_rst deasserts is at this address.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,

    // ---- Instruction memory interface --------------------------------------
    // External, combinational read-only memory.
    output wire [31:0] o_imem_raddr,
    input  wire [31:0] i_imem_rdata,

    // ---- Data memory interface ---------------------------------------------
    // External, combinational read, synchronous write.
    // Address is ALWAYS word-aligned (low 2 bits are 2'b00).
    output wire [31:0] o_dmem_addr,
    output wire        o_dmem_ren,
    output wire        o_dmem_wen,
    output wire [31:0] o_dmem_wdata,
    output wire [ 3:0] o_dmem_mask,
    input  wire [31:0] i_dmem_rdata,

    // ---- Retire interface (Testbench Verification) -------------------------
    output wire        o_retire_valid,
    output wire [31:0] o_retire_inst,
    output wire        o_retire_trap,
    output wire        o_retire_halt,
    output wire [ 4:0] o_retire_rs1_raddr,
    output wire [31:0] o_retire_rs1_rdata,
    output wire [ 4:0] o_retire_rs2_raddr,
    output wire [31:0] o_retire_rs2_rdata,
    output wire [ 4:0] o_retire_rd_waddr,
    output wire [31:0] o_retire_rd_wdata,
    output wire [31:0] o_retire_pc,
    output wire [31:0] o_retire_next_pc
);

    // ========================================================================
    // 1. Program Counter (PC)
    // ========================================================================
    reg  [31:0] pc_reg;
    wire [31:0] next_pc;
    wire [31:0] pc_plus_4;

    assign pc_plus_4    = pc_reg + 32'd4;
    assign o_imem_raddr = pc_reg;

    always @(posedge i_clk) begin
        if (i_rst) begin
            pc_reg <= RESET_ADDR;
        end else begin
            pc_reg <= next_pc;
        end
    end

    // ========================================================================
    // 2. Decoder & Immediate Generator Linking
    // ========================================================================
    // Note: imm.v is instantiated inside decoder.v (imm_gen), providing o_immediate.
    wire        r_legal;
    wire        r_halt;
    wire [ 4:0] r_rs1;
    wire [ 4:0] r_rs2;
    wire [ 4:0] r_rd;
    wire [31:0] r_immediate;
    wire        r_op1_sel;
    wire        r_op2_sel;
    wire [ 2:0] r_alu_opsel;
    wire        r_alu_sub;
    wire        r_alu_unsigned;
    wire        r_alu_arith;
    wire        r_branch;
    wire        r_jump;
    wire        r_branch_equal;
    wire        r_branch_unsigned;
    wire        r_branch_invert;
    wire        r_dmem_ren;
    wire        r_dmem_wen;
    wire [ 1:0] r_dmem_align;
    wire        r_dmem_memb;
    wire        r_dmem_memh;
    wire        r_dmem_memw;
    wire        r_dmem_memu;
    wire [ 3:0] r_rd_sel;
    wire        r_pc_sel;

    decoder decoder_i (
        .i_inst            (i_imem_rdata),
        .o_legal           (r_legal),
        .o_halt            (r_halt),
        .o_immediate       (r_immediate),
        .o_rs1             (r_rs1),
        .o_rs2             (r_rs2),
        .o_rd              (r_rd),
        .o_op1_sel         (r_op1_sel),
        .o_op2_sel         (r_op2_sel),
        .o_alu_opsel       (r_alu_opsel),
        .o_alu_sub         (r_alu_sub),
        .o_alu_unsigned    (r_alu_unsigned),
        .o_alu_arith       (r_alu_arith),
        .o_branch          (r_branch),
        .o_jump            (r_jump),
        .o_branch_equal    (r_branch_equal),
        .o_branch_unsigned (r_branch_unsigned),
        .o_branch_invert   (r_branch_invert),
        .o_dmem_ren        (r_dmem_ren),
        .o_dmem_wen        (r_dmem_wen),
        .o_dmem_align      (r_dmem_align),
        .o_dmem_memb       (r_dmem_memb),
        .o_dmem_memh       (r_dmem_memh),
        .o_dmem_memw       (r_dmem_memw),
        .o_dmem_memu       (r_dmem_memu),
        .o_rd_sel          (r_rd_sel),
        .o_pc_sel          (r_pc_sel)
    );

    // ========================================================================
    // 3. Register File (RF) Linking
    // ========================================================================
    // BYPASS_EN = 0 per single-cycle specification.
    // Writes to x0 or when trapped are discarded by gating i_rd_waddr to 5'd0.
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;
    wire [31:0] rd_data;
    wire [ 4:0] actual_rd_waddr;
    wire        trap_condition;

    assign actual_rd_waddr = trap_condition ? 5'd0 : r_rd;

    rf #(
        .BYPASS_EN(0)
    ) rf_connect (
        .i_clk       (i_clk),
        .i_rst       (i_rst),
        .i_rs1_raddr (r_rs1),
        .o_rs1_rdata (rs1_data),
        .i_rs2_raddr (r_rs2),
        .o_rs2_rdata (rs2_data),
        .i_rd_waddr  (actual_rd_waddr),
        .i_rd_wdata  (rd_data)
    );

    // ========================================================================
    // 4. ALU Linking
    // ========================================================================
    wire [31:0] alu_op1;
    wire [31:0] alu_op2;
    wire [31:0] alu_result;
    wire        alu_eq;
    wire        alu_slt;

    assign alu_op1 = r_op1_sel ? pc_reg : rs1_data;
    assign alu_op2 = r_op2_sel ? r_immediate : rs2_data;

    alu alu_i (
        .i_opsel    (r_alu_opsel),
        .i_sub      (r_alu_sub),
        .i_unsigned (r_alu_unsigned),
        .i_arith    (r_alu_arith),
        .i_op1      (alu_op1),
        .i_op2      (alu_op2),
        .o_result   (alu_result),
        .o_eq       (alu_eq),
        .o_slt      (alu_slt)
    );

    // ========================================================================
    // [TEAMMATE HOOK 1]: Branch and Jump Logic
    // ========================================================================
    // Computes target addresses and whether a branch/jump is taken.
    wire [31:0] branch_target;
    wire [31:0] jump_target;
    wire        branch_comp;
    wire        branch_taken;

    assign branch_target = pc_reg + r_immediate;
    assign jump_target   = r_pc_sel ? (alu_result & ~32'd1) : (pc_reg + r_immediate);

    assign branch_comp   = r_branch_equal ? alu_eq : alu_slt;
    assign branch_taken  = r_branch & (branch_comp ^ r_branch_invert);

    assign next_pc = r_jump       ? jump_target :
                     branch_taken ? branch_target :
                                    pc_plus_4;

    // ========================================================================
    // [TEAMMATE HOOK 2]: Data Memory Interface & Sub-word Alignment
    // ========================================================================
    // Checks alignment, forms byte mask, shifts store data, and sign/zero extends loads.
    wire [1:0] mem_byte_offset;
    assign mem_byte_offset = alu_result[1:0];

    // Misalignment check:
    // Half-word access requires 2-byte alignment (LSB must be 0).
    // Word access requires 4-byte alignment (LSBs must be 00).
    wire misaligned_access;
    assign misaligned_access = (r_dmem_ren | r_dmem_wen) & (
        (r_dmem_memh & mem_byte_offset[0]) |
        (r_dmem_memw & (mem_byte_offset != 2'b00))
    );

    assign trap_condition = (!r_legal) | misaligned_access;

    // Word-aligned address output to memory
    assign o_dmem_addr = {alu_result[31:2], 2'b00};

    // Suppress memory access on trap or halt
    assign o_dmem_ren  = r_dmem_ren & (!trap_condition) & (!r_halt);
    assign o_dmem_wen  = r_dmem_wen & (!trap_condition) & (!r_halt);

    // Byte lane mask generation
    assign o_dmem_mask = r_dmem_memw ? 4'b1111 :
                         r_dmem_memh ? (mem_byte_offset[1] ? 4'b1100 : 4'b0011) :
                         r_dmem_memb ? (
                             (mem_byte_offset == 2'b00) ? 4'b0001 :
                             (mem_byte_offset == 2'b01) ? 4'b0010 :
                             (mem_byte_offset == 2'b10) ? 4'b0100 : 4'b1000
                         ) : 4'b0000;

    // Store data positioning into active byte lanes
    assign o_dmem_wdata = r_dmem_memw ? rs2_data :
                          r_dmem_memh ? (mem_byte_offset[1] ? {rs2_data[15:0], 16'b0} : {16'b0, rs2_data[15:0]}) :
                          r_dmem_memb ? (
                              (mem_byte_offset == 2'b00) ? {24'b0, rs2_data[7:0]} :
                              (mem_byte_offset == 2'b01) ? {16'b0, rs2_data[7:0], 8'b0} :
                              (mem_byte_offset == 2'b10) ? {8'b0, rs2_data[7:0], 16'b0} :
                                                           {rs2_data[7:0], 24'b0}
                          ) : rs2_data;

    // Load data byte lane extraction and sign/zero extension
    wire [ 7:0] raw_byte;
    wire [15:0] raw_half;
    wire [31:0] mem_rdata_ext;

    assign raw_byte = (mem_byte_offset == 2'b00) ? i_dmem_rdata[ 7: 0] :
                      (mem_byte_offset == 2'b01) ? i_dmem_rdata[15: 8] :
                      (mem_byte_offset == 2'b10) ? i_dmem_rdata[23:16] :
                                                   i_dmem_rdata[31:24];

    assign raw_half = mem_byte_offset[1] ? i_dmem_rdata[31:16] : i_dmem_rdata[15:0];

    assign mem_rdata_ext = r_dmem_memw ? i_dmem_rdata :
                           r_dmem_memh ? (r_dmem_memu ? {16'b0, raw_half} : {{16{raw_half[15]}}, raw_half}) :
                           r_dmem_memb ? (r_dmem_memu ? {24'b0, raw_byte} : {{24{raw_byte[7]}}, raw_byte}) :
                                         32'd0;

    // ========================================================================
    // [TEAMMATE HOOK 3]: Writeback Multiplexer
    // ========================================================================
    // Selects destination data based on one-hot r_rd_sel:
    // [0] = ALU result
    // [1] = immediate (LUI)
    // [2] = PC + 4 (JAL / JALR)
    // [3] = memory load data
    assign rd_data = r_rd_sel[0] ? alu_result :
                     r_rd_sel[1] ? r_immediate :
                     r_rd_sel[2] ? pc_plus_4 :
                     r_rd_sel[3] ? mem_rdata_ext :
                                   32'd0;

    // ========================================================================
    // [TEAMMATE HOOK 4]: Retire Interface
    // ========================================================================
    // Drives cycle-by-cycle retire information for the testbench.
    wire [6:0] inst_opcode;
    assign inst_opcode = i_imem_rdata[6:0];

    // rs1 is read by OP, OP-IMM, LOAD, STORE, BRANCH, JALR (not LUI, AUIPC, JAL, or illegal)
    wire rs1_is_read;
    assign rs1_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0010011) | // OP-IMM
        (inst_opcode == 7'b0000011) | // LOAD
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011) | // BRANCH
        (inst_opcode == 7'b1100111)   // JALR
    );

    // rs2 is read only by OP, STORE, and BRANCH
    wire rs2_is_read;
    assign rs2_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011)   // BRANCH
    );

    assign o_retire_valid     = ~i_rst;
    assign o_retire_inst      = i_imem_rdata;
    assign o_retire_trap      = trap_condition;
    assign o_retire_halt      = r_halt;
    assign o_retire_rs1_raddr = rs1_is_read ? r_rs1 : 5'd0;
    assign o_retire_rs1_rdata = rs1_data;
    assign o_retire_rs2_raddr = rs2_is_read ? r_rs2 : 5'd0;
    assign o_retire_rs2_rdata = rs2_data;
    assign o_retire_rd_waddr  = actual_rd_waddr;
    assign o_retire_rd_wdata  = rd_data;
    assign o_retire_pc        = pc_reg;
    assign o_retire_next_pc   = next_pc;

endmodule

`default_nettype wire