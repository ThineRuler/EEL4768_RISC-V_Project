// Super Awesome Group
// Ian Hunter, Arianna Balsamo Uzcategui, David Escobar, Leah Dwyer

`default_nettype none

module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00400000,
    // When set, pipeline forwarding optimizations are enabled.
    parameter FWD_EN = 1,
    // When set, register file bypassing is enabled.
    parameter BYPASS_EN = 1
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available on the same cycle.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored).
    output wire [31:0] o_dmem_addr,
    output wire        o_dmem_ren,
    output wire        o_dmem_wen,
    output wire [31:0] o_dmem_wdata,
    output wire [ 3:0] o_dmem_mask,
    input  wire [31:0] i_dmem_rdata,
    // Retire interface
    output wire        o_retire_valid,
    output wire [31:0] o_retire_inst,
    output wire        o_retire_trap,
    output wire        o_retire_halt,
    output wire [ 4:0] o_retire_rs1_raddr,
    output wire [ 4:0] o_retire_rs2_raddr,
    output wire [31:0] o_retire_rs1_rdata,
    output wire [31:0] o_retire_rs2_rdata,
    output wire [ 4:0] o_retire_rd_waddr,
    output wire [31:0] o_retire_rd_wdata,
    output wire [31:0] o_retire_dmem_addr,
    output wire [ 3:0] o_retire_dmem_mask,
    output wire        o_retire_dmem_ren,
    output wire        o_retire_dmem_wen,
    output wire [31:0] o_retire_dmem_rdata,
    output wire [31:0] o_retire_dmem_wdata,
    output wire [31:0] o_retire_pc,
    output wire [31:0] o_retire_next_pc

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);

    // ========================================================================
    // Pipeline Registers and Internal Signal Declarations
    // (Declared up front to satisfy `default_nettype none` in Verilog-2005)
    // ========================================================================

    // Pipeline control
    wire stall;
    wire flush;

    // ------------------------------------------------------------------------
    // IF Stage Signals
    // ------------------------------------------------------------------------
    reg  [31:0] PC;
    wire [31:0] nxt_instruct;
    wire [31:0] next_pc;

    // ------------------------------------------------------------------------
    // IF/ID Pipeline Register
    // ------------------------------------------------------------------------
    reg  [31:0] IF_ID_PC;
    reg  [31:0] IF_ID_instruct;
    reg  [31:0] IF_ID_next_pc;
    reg         IF_ID_valid;

    // ------------------------------------------------------------------------
    // ID Stage Signals
    // ------------------------------------------------------------------------
    wire        r_legal;
    wire        r_halt;
    wire        r_op1_sel;
    wire        r_op2_sel;
    wire [2:0]  r_alu_opsel;
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
    wire [1:0]  r_dmem_align;
    wire        r_dmem_memb;
    wire        r_dmem_memh;
    wire        r_dmem_memw;
    wire        r_dmem_memu;
    wire [3:0]  r_rd_sel;
    wire        r_pc_sel;
    wire [31:0] r_immediate;
    wire [4:0]  r_rs1;
    wire [4:0]  r_rs2;
    wire [4:0]  r_rd;
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;
    wire [6:0]  inst_opcode;
    wire        rs1_is_read;
    wire        rs2_is_read;
    wire        ID_trap;

    // ------------------------------------------------------------------------
    // ID/EX Pipeline Register
    // ------------------------------------------------------------------------
    reg  [31:0] ID_EX_PC;
    reg  [31:0] ID_EX_next_pc;
    reg  [31:0] ID_EX_link_pc;
    reg  [31:0] ID_EX_instruct;
    reg         ID_EX_valid;

    reg  [4:0]  ID_EX_rs1;
    reg  [4:0]  ID_EX_rs2;
    reg  [4:0]  ID_EX_rd;
    reg  [31:0] ID_EX_immediate;
    reg  [31:0] ID_EX_rs1_data;
    reg  [31:0] ID_EX_rs2_data;

    reg         ID_EX_op1_sel;
    reg         ID_EX_op2_sel;
    reg  [2:0]  ID_EX_alu_opsel;
    reg         ID_EX_alu_sub;
    reg         ID_EX_alu_unsigned;
    reg         ID_EX_alu_arith;

    reg         ID_EX_branch;
    reg         ID_EX_jump;
    reg         ID_EX_branch_equal;
    reg         ID_EX_branch_unsigned;
    reg         ID_EX_branch_invert;

    reg         ID_EX_dmem_ren;
    reg         ID_EX_dmem_wen;
    reg  [1:0]  ID_EX_dmem_align;
    reg         ID_EX_dmem_memb;
    reg         ID_EX_dmem_memh;
    reg         ID_EX_dmem_memw;
    reg         ID_EX_dmem_memu;

    reg  [3:0]  ID_EX_rd_sel;
    reg         ID_EX_pc_sel;
    reg         ID_EX_legal;
    reg         ID_EX_halt;
    reg         ID_EX_rs1_is_read;
    reg         ID_EX_rs2_is_read;
    reg         ID_EX_trap;

    // ------------------------------------------------------------------------
    // EX Stage Signals
    // ------------------------------------------------------------------------
    wire        r_alu_slt;
    wire        r_alu_eq;
    wire [31:0] r_result;
    wire [31:0] r_alu_op1;
    wire [31:0] r_alu_op2;
    wire [31:0] r_forward_rs1;
    wire [31:0] r_forward_rs2;
    wire [31:0] r_exmem_fwd_data;
    wire [31:0] branch_target;
    wire [31:0] jump_target;
    wire        branch_comp;
    wire        branch_taken;
    wire        is_jump;
    wire [31:0] EX_next_pc;

    // ------------------------------------------------------------------------
    // EX/MEM Pipeline Register
    // ------------------------------------------------------------------------
    reg  [31:0] EX_MEM_PC;
    reg  [31:0] EX_MEM_next_pc;
    reg  [31:0] EX_MEM_link_pc;
    reg  [31:0] EX_MEM_instruct;
    reg         EX_MEM_valid;

    reg  [31:0] EX_MEM_result;
    reg  [31:0] EX_MEM_rs2_data;
    reg  [4:0]  EX_MEM_rd;
    reg  [31:0] EX_MEM_immediate;

    reg         EX_MEM_dmem_ren;
    reg         EX_MEM_dmem_wen;
    reg  [1:0]  EX_MEM_dmem_align;
    reg         EX_MEM_dmem_memb;
    reg         EX_MEM_dmem_memh;
    reg         EX_MEM_dmem_memw;
    reg         EX_MEM_dmem_memu;

    reg  [3:0]  EX_MEM_rd_sel;
    reg         EX_MEM_pc_sel;
    reg         EX_MEM_legal;
    reg         EX_MEM_halt;
    reg  [4:0]  EX_MEM_rs1;
    reg  [4:0]  EX_MEM_rs2;
    reg  [31:0] EX_MEM_rs1_data;
    reg         EX_MEM_rs1_is_read;
    reg         EX_MEM_rs2_is_read;
    reg         EX_MEM_trap;

    // ------------------------------------------------------------------------
    // MEM Stage Signals
    // ------------------------------------------------------------------------
    wire [1:0]  mem_byte_offset;
    wire        misaligned_access;
    wire        trap_in_mem;
    wire [31:0] r_store_data;
    wire [ 7:0] raw_byte;
    wire [15:0] raw_half;
    wire [31:0] mem_rdata_ext;

    // ------------------------------------------------------------------------
    // MEM/WB Pipeline Register
    // ------------------------------------------------------------------------
    reg  [31:0] MEM_WB_PC;
    reg  [31:0] MEM_WB_next_pc;
    reg  [31:0] MEM_WB_link_pc;
    reg  [31:0] MEM_WB_instruct;
    reg         MEM_WB_valid;

    reg  [31:0] MEM_WB_result;
    reg  [31:0] MEM_WB_mem_rdata;
    reg  [4:0]  MEM_WB_rd;
    reg  [31:0] MEM_WB_immediate;
    reg  [3:0]  MEM_WB_rd_sel;

    reg         MEM_WB_trap;
    reg         MEM_WB_legal;
    reg         MEM_WB_halt;

    reg  [4:0]  MEM_WB_rs1;
    reg  [4:0]  MEM_WB_rs2;
    reg  [31:0] MEM_WB_rs1_data;
    reg  [31:0] MEM_WB_rs2_data;
    reg         MEM_WB_rs1_is_read;
    reg         MEM_WB_rs2_is_read;

    reg  [31:0] MEM_WB_dmem_addr;
    reg         MEM_WB_dmem_ren;
    reg         MEM_WB_dmem_wen;
    reg  [3:0]  MEM_WB_dmem_mask;
    reg  [31:0] MEM_WB_dmem_rdata;
    reg  [31:0] MEM_WB_dmem_wdata;

    // ------------------------------------------------------------------------
    // WB Stage Signals
    // ------------------------------------------------------------------------
    wire [31:0] rd_data;
    wire [31:0] r_memwb_fwd_data;
    wire [4:0]  actual_rd_waddr;

    // ========================================================================
    // 1. Program Counter (PC) & IF Stage
    // ========================================================================

    assign o_imem_raddr = PC;
    assign nxt_instruct = PC + 32'd4;
    assign next_pc      = flush ? EX_next_pc : nxt_instruct;

    always @(posedge i_clk) begin
        if (i_rst)
            PC <= RESET_ADDR;
        else if (flush)
            PC <= EX_next_pc;
        else if (stall)
            PC <= PC;
        else
            PC <= nxt_instruct;
    end

    // ========================================================================
    // IF/ID Pipeline Register
    // ========================================================================

    always @(posedge i_clk) begin
        if (i_rst) begin
            IF_ID_PC       <= 32'd0;
            IF_ID_instruct <= 32'h00000013; // NOP instruction (addi x0, x0, 0)
            IF_ID_next_pc  <= 32'd0;
            IF_ID_valid    <= 1'b0;
        end else if (flush) begin
            IF_ID_PC       <= 32'd0;
            IF_ID_instruct <= 32'h00000013; // Flushed bubble
            IF_ID_next_pc  <= 32'd0;
            IF_ID_valid    <= 1'b0;
        end else if (stall) begin
            IF_ID_PC       <= IF_ID_PC;
            IF_ID_instruct <= IF_ID_instruct;
            IF_ID_next_pc  <= IF_ID_next_pc;
            IF_ID_valid    <= IF_ID_valid;
        end else begin
            IF_ID_PC       <= PC;
            IF_ID_instruct <= i_imem_rdata;
            IF_ID_next_pc  <= nxt_instruct;
            IF_ID_valid    <= 1'b1;
        end
    end

    // ========================================================================
    // 2. Decoder & Immediate Generator Linking
    // ========================================================================

    decoder decoder_i (
        .i_inst             (IF_ID_instruct),
        .o_legal            (r_legal),
        .o_halt             (r_halt),
        .o_immediate        (r_immediate),
        .o_rs1              (r_rs1),
        .o_rs2              (r_rs2),
        .o_rd               (r_rd),
        .o_op1_sel          (r_op1_sel),
        .o_op2_sel          (r_op2_sel),
        .o_alu_opsel        (r_alu_opsel),
        .o_alu_sub          (r_alu_sub),
        .o_alu_unsigned     (r_alu_unsigned),
        .o_alu_arith        (r_alu_arith),
        .o_branch           (r_branch),
        .o_jump             (r_jump),
        .o_branch_equal     (r_branch_equal),
        .o_branch_unsigned  (r_branch_unsigned),
        .o_branch_invert    (r_branch_invert),
        .o_dmem_ren         (r_dmem_ren),
        .o_dmem_wen         (r_dmem_wen),
        .o_dmem_align       (r_dmem_align),
        .o_dmem_memb        (r_dmem_memb),
        .o_dmem_memh        (r_dmem_memh),
        .o_dmem_memw        (r_dmem_memw),
        .o_dmem_memu        (r_dmem_memu),
        .o_rd_sel           (r_rd_sel),
        .o_pc_sel           (r_pc_sel)
    );

    // ========================================================================
    // 3. Register File (RF) Linking
    // ========================================================================

    // Writes to x0 or when trapped are discarded by gating actual_rd_waddr to 5'd0.
    assign actual_rd_waddr = (!MEM_WB_valid || MEM_WB_trap) ? 5'd0 : MEM_WB_rd;

    rf #(
        .BYPASS_EN(BYPASS_EN)
    ) rf_connect (
        .i_clk          (i_clk),
        .i_rst          (i_rst),
        .o_rs1_rdata    (rs1_data),
        .o_rs2_rdata    (rs2_data),
        .i_rs1_raddr    (r_rs1),
        .i_rs2_raddr    (r_rs2),
        .i_rd_waddr     (actual_rd_waddr),
        .i_rd_wdata     (rd_data)
    );

    // Drives cycle-by-cycle retire information for the testbench.
    assign inst_opcode = IF_ID_instruct[6:0];

    // rs1 is read by OP, OP-IMM, LOAD, STORE, BRANCH, JALR (not LUI, AUIPC, JAL, or illegal)
    assign rs1_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0010011) | // OP-IMM
        (inst_opcode == 7'b0000011) | // LOAD
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011) | // BRANCH
        (inst_opcode == 7'b1100111)   // JALR
    );

    // rs2 is read only by OP, STORE, and BRANCH
    assign rs2_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011)   // BRANCH
    );

    // Illegal instruction trap detected in ID
    assign ID_trap = IF_ID_valid && (!r_legal);

    // ========================================================================
    // STALL / HAZARD DETECTION SIGNAL
    // ========================================================================

    // Load-use hazard: checks if the instruction in ID depends on the load in EX
    wire load_use_hazard;
    assign load_use_hazard =
        ID_EX_valid && ID_EX_dmem_ren && (ID_EX_rd != 5'd0) &&
        (
            (rs1_is_read && (r_rs1 == ID_EX_rd)) ||
            (rs2_is_read && (r_rs2 == ID_EX_rd))
        );

    // When forwarding is disabled (FWD_EN == 0), stall on any RAW dependency in EX, MEM, or WB
    wire raw_stall_no_fwd;
    assign raw_stall_no_fwd =
        (!FWD_EN) && (
            (ID_EX_valid && (ID_EX_rd != 5'd0) && (ID_EX_rd_sel != 4'd0) &&
                ((rs1_is_read && (r_rs1 == ID_EX_rd)) || (rs2_is_read && (r_rs2 == ID_EX_rd)))) ||
            (EX_MEM_valid && (EX_MEM_rd != 5'd0) && (EX_MEM_rd_sel != 4'd0) &&
                ((rs1_is_read && (r_rs1 == EX_MEM_rd)) || (rs2_is_read && (r_rs2 == EX_MEM_rd)))) ||
            (MEM_WB_valid && (MEM_WB_rd != 5'd0) && (MEM_WB_rd_sel != 4'd0) &&
                ((rs1_is_read && (r_rs1 == MEM_WB_rd)) || (rs2_is_read && (r_rs2 == MEM_WB_rd))))
        );

    assign stall = load_use_hazard || raw_stall_no_fwd;

    // ========================================================================
    // ID/EX Pipeline Registers
    // ========================================================================

    always @(posedge i_clk) begin
        if (i_rst) begin
            ID_EX_PC              <= 32'd0;
            ID_EX_next_pc         <= 32'd0;
            ID_EX_link_pc         <= 32'd0;
            ID_EX_instruct        <= 32'h00000013; // NOP instruction
            ID_EX_valid           <= 1'b0;

            ID_EX_rs1             <= 5'd0;
            ID_EX_rs2             <= 5'd0;
            ID_EX_rd              <= 5'd0;
            ID_EX_immediate       <= 32'd0;

            ID_EX_rs1_data        <= 32'd0;
            ID_EX_rs2_data        <= 32'd0;

            ID_EX_op1_sel         <= 1'b0;
            ID_EX_op2_sel         <= 1'b0;
            ID_EX_alu_opsel       <= 3'd0;
            ID_EX_alu_sub         <= 1'b0;
            ID_EX_alu_unsigned    <= 1'b0;
            ID_EX_alu_arith       <= 1'b0;

            ID_EX_branch          <= 1'b0;
            ID_EX_jump            <= 1'b0;
            ID_EX_branch_equal    <= 1'b0;
            ID_EX_branch_unsigned <= 1'b0;
            ID_EX_branch_invert   <= 1'b0;

            ID_EX_dmem_ren        <= 1'b0;
            ID_EX_dmem_wen        <= 1'b0;
            ID_EX_dmem_align      <= 2'd0;
            ID_EX_dmem_memb       <= 1'b0;
            ID_EX_dmem_memh       <= 1'b0;
            ID_EX_dmem_memw       <= 1'b0;
            ID_EX_dmem_memu       <= 1'b0;

            ID_EX_rd_sel          <= 4'd0;
            ID_EX_pc_sel          <= 1'b0;
            ID_EX_legal           <= 1'b0;
            ID_EX_halt            <= 1'b0;
            ID_EX_rs1_is_read     <= 1'b0;
            ID_EX_rs2_is_read     <= 1'b0;
            ID_EX_trap            <= 1'b0;
        end else if (flush || stall) begin
            // Insert bubble into EX on branch/jump flush or load-use stall
            ID_EX_PC              <= 32'd0;
            ID_EX_next_pc         <= 32'd0;
            ID_EX_link_pc         <= 32'd0;
            ID_EX_instruct        <= 32'h00000013; // NOP
            ID_EX_valid           <= 1'b0;

            ID_EX_rs1             <= 5'd0;
            ID_EX_rs2             <= 5'd0;
            ID_EX_rd              <= 5'd0;
            ID_EX_immediate       <= 32'd0;

            ID_EX_rs1_data        <= 32'd0;
            ID_EX_rs2_data        <= 32'd0;

            ID_EX_op1_sel         <= 1'b0;
            ID_EX_op2_sel         <= 1'b0;
            ID_EX_alu_opsel       <= 3'd0;
            ID_EX_alu_sub         <= 1'b0;
            ID_EX_alu_unsigned    <= 1'b0;
            ID_EX_alu_arith       <= 1'b0;

            ID_EX_branch          <= 1'b0;
            ID_EX_jump            <= 1'b0;
            ID_EX_branch_equal    <= 1'b0;
            ID_EX_branch_unsigned <= 1'b0;
            ID_EX_branch_invert   <= 1'b0;

            ID_EX_dmem_ren        <= 1'b0;
            ID_EX_dmem_wen        <= 1'b0;
            ID_EX_dmem_align      <= 2'd0;
            ID_EX_dmem_memb       <= 1'b0;
            ID_EX_dmem_memh       <= 1'b0;
            ID_EX_dmem_memw       <= 1'b0;
            ID_EX_dmem_memu       <= 1'b0;

            ID_EX_rd_sel          <= 4'd0;
            ID_EX_pc_sel          <= 1'b0;
            ID_EX_legal           <= 1'b0;
            ID_EX_halt            <= 1'b0;
            ID_EX_rs1_is_read     <= 1'b0;
            ID_EX_rs2_is_read     <= 1'b0;
            ID_EX_trap            <= 1'b0;
        end else begin
            // Normal latching of the Decoder & RF outputs
            ID_EX_PC              <= IF_ID_PC;
            ID_EX_next_pc         <= IF_ID_next_pc;
            ID_EX_link_pc         <= IF_ID_next_pc; // PC + 4 for JAL/JALR link address
            ID_EX_instruct        <= IF_ID_instruct;
            ID_EX_valid           <= IF_ID_valid;

            ID_EX_rs1             <= r_rs1;
            ID_EX_rs2             <= r_rs2;
            ID_EX_rd              <= r_rd;
            ID_EX_immediate       <= r_immediate;

            ID_EX_rs1_data        <= rs1_data;
            ID_EX_rs2_data        <= rs2_data;

            ID_EX_op1_sel         <= r_op1_sel;
            ID_EX_op2_sel         <= r_op2_sel;
            ID_EX_alu_opsel       <= r_alu_opsel;
            ID_EX_alu_sub         <= r_alu_sub;
            ID_EX_alu_unsigned    <= r_alu_unsigned;
            ID_EX_alu_arith       <= r_alu_arith;

            ID_EX_branch          <= r_branch;
            ID_EX_jump            <= r_jump;
            ID_EX_branch_equal    <= r_branch_equal;
            ID_EX_branch_unsigned <= r_branch_unsigned;
            ID_EX_branch_invert   <= r_branch_invert;

            ID_EX_dmem_ren        <= r_dmem_ren;
            ID_EX_dmem_wen        <= r_dmem_wen;
            ID_EX_dmem_align      <= r_dmem_align;
            ID_EX_dmem_memb       <= r_dmem_memb;
            ID_EX_dmem_memh       <= r_dmem_memh;
            ID_EX_dmem_memw       <= r_dmem_memw;
            ID_EX_dmem_memu       <= r_dmem_memu;

            ID_EX_rd_sel          <= r_rd_sel;
            ID_EX_pc_sel          <= r_pc_sel;
            ID_EX_legal           <= r_legal;
            ID_EX_halt            <= r_halt;
            ID_EX_rs1_is_read     <= rs1_is_read;
            ID_EX_rs2_is_read     <= rs2_is_read;
            ID_EX_trap            <= ID_trap;
        end
    end

    // ========================================================================
    // 4. ALU & Forwarding Linking
    // ========================================================================

    // Select the value forwarded from EX/MEM stage.
    // Loads are intentionally excluded here because load data is not ready
    // until the MEM/WB stage. Link PC is forwarded for JAL/JALR.
    assign r_exmem_fwd_data =
        EX_MEM_rd_sel[0] ? EX_MEM_result :
        EX_MEM_rd_sel[1] ? EX_MEM_immediate :
        EX_MEM_rd_sel[2] ? EX_MEM_link_pc :
                           32'd0;

    // Forwarding to ALU operand 1 (rs1)
    // Priority: 1. EX/MEM (newer)  2. MEM/WB (older)  3. Register-file value
    assign r_forward_rs1 =
        (FWD_EN &&
         (ID_EX_rs1 != 5'd0) &&
         EX_MEM_valid &&
         EX_MEM_legal &&
         (!EX_MEM_halt) &&
         (EX_MEM_rd != 5'd0) &&
         (EX_MEM_rd_sel[0] || EX_MEM_rd_sel[1] || EX_MEM_rd_sel[2]) &&
         (EX_MEM_rd == ID_EX_rs1))
        ? r_exmem_fwd_data :

        (FWD_EN &&
         (ID_EX_rs1 != 5'd0) &&
         MEM_WB_valid &&
         (!MEM_WB_trap) &&
         (!MEM_WB_halt) &&
         (MEM_WB_rd != 5'd0) &&
         (MEM_WB_rd_sel != 4'd0) &&
         (MEM_WB_rd == ID_EX_rs1))
        ? r_memwb_fwd_data :

        ID_EX_rs1_data;

    // Forwarding to ALU operand 2 (rs2)
    assign r_forward_rs2 =
        (FWD_EN &&
         (ID_EX_rs2 != 5'd0) &&
         EX_MEM_valid &&
         EX_MEM_legal &&
         (!EX_MEM_halt) &&
         (EX_MEM_rd != 5'd0) &&
         (EX_MEM_rd_sel[0] || EX_MEM_rd_sel[1] || EX_MEM_rd_sel[2]) &&
         (EX_MEM_rd == ID_EX_rs2))
        ? r_exmem_fwd_data :

        (FWD_EN &&
         (ID_EX_rs2 != 5'd0) &&
         MEM_WB_valid &&
         (!MEM_WB_trap) &&
         (!MEM_WB_halt) &&
         (MEM_WB_rd != 5'd0) &&
         (MEM_WB_rd_sel != 4'd0) &&
         (MEM_WB_rd == ID_EX_rs2))
        ? r_memwb_fwd_data :

        ID_EX_rs2_data;

    // ALU operands
    assign r_alu_op1 = ID_EX_op1_sel ? ID_EX_PC : r_forward_rs1;
    assign r_alu_op2 = ID_EX_op2_sel ? ID_EX_immediate : r_forward_rs2;

    alu alu_i (
        .i_opsel    (ID_EX_alu_opsel),
        .i_sub      (ID_EX_alu_sub),
        .i_unsigned (ID_EX_alu_unsigned),
        .o_slt      (r_alu_slt),
        .o_eq       (r_alu_eq),
        .o_result   (r_result),
        .i_op1      (r_alu_op1),
        .i_op2      (r_alu_op2),
        .i_arith    (ID_EX_alu_arith)
    );

    // ========================================================================
    // Branch and Jump Logic (Section 4.1.3 & 4.2.2)
    // ========================================================================
    assign branch_target = ID_EX_PC + ID_EX_immediate;
    assign jump_target   = ID_EX_pc_sel ? (r_result & ~32'd1) : (ID_EX_PC + ID_EX_immediate);

    assign branch_comp   = ID_EX_branch_equal ? r_alu_eq : r_alu_slt;
    assign branch_taken  = ID_EX_valid && (!ID_EX_trap) && ID_EX_branch && (branch_comp ^ ID_EX_branch_invert);
    assign is_jump       = ID_EX_valid && (!ID_EX_trap) && ID_EX_jump;

    // Flushing signal: flushes IF/ID and ID/EX on taken branch or jump resolved in EX
    assign flush = is_jump || branch_taken;

    assign EX_next_pc = is_jump      ? jump_target :
                        branch_taken ? branch_target :
                                       ID_EX_next_pc;

    // ========================================================================
    // EX/MEM Pipeline Registers
    // ========================================================================

    always @(posedge i_clk) begin
        if (i_rst) begin
            EX_MEM_PC          <= 32'd0;
            EX_MEM_next_pc     <= 32'd0;
            EX_MEM_link_pc     <= 32'd0;
            EX_MEM_instruct    <= 32'h00000013; // NOP instruction
            EX_MEM_valid       <= 1'b0;

            EX_MEM_result      <= 32'd0;
            EX_MEM_rs2_data    <= 32'd0;
            EX_MEM_rd          <= 5'd0;
            EX_MEM_immediate   <= 32'd0;

            EX_MEM_dmem_ren    <= 1'b0;
            EX_MEM_dmem_wen    <= 1'b0;
            EX_MEM_dmem_align  <= 2'd0;
            EX_MEM_dmem_memb   <= 1'b0;
            EX_MEM_dmem_memh   <= 1'b0;
            EX_MEM_dmem_memw   <= 1'b0;
            EX_MEM_dmem_memu   <= 1'b0;

            EX_MEM_rd_sel      <= 4'd0;
            EX_MEM_pc_sel      <= 1'b0;
            EX_MEM_legal       <= 1'b0;
            EX_MEM_halt        <= 1'b0;
            EX_MEM_rs1         <= 5'd0;
            EX_MEM_rs2         <= 5'd0;
            EX_MEM_rs1_data    <= 32'd0;
            EX_MEM_rs1_is_read <= 1'b0;
            EX_MEM_rs2_is_read <= 1'b0;
            EX_MEM_trap        <= 1'b0;
        end else begin
            EX_MEM_PC          <= ID_EX_PC;
            EX_MEM_next_pc     <= EX_next_pc;
            EX_MEM_link_pc     <= ID_EX_link_pc;
            EX_MEM_instruct    <= ID_EX_instruct;
            EX_MEM_valid       <= ID_EX_valid;

            EX_MEM_result      <= r_result;
            EX_MEM_rs2_data    <= r_forward_rs2;
            EX_MEM_rd          <= ID_EX_rd;
            EX_MEM_immediate   <= ID_EX_immediate;

            EX_MEM_dmem_ren    <= ID_EX_dmem_ren;
            EX_MEM_dmem_wen    <= ID_EX_dmem_wen;
            EX_MEM_dmem_align  <= ID_EX_dmem_align;
            EX_MEM_dmem_memb   <= ID_EX_dmem_memb;
            EX_MEM_dmem_memh   <= ID_EX_dmem_memh;
            EX_MEM_dmem_memw   <= ID_EX_dmem_memw;
            EX_MEM_dmem_memu   <= ID_EX_dmem_memu;

            EX_MEM_rd_sel      <= ID_EX_rd_sel;
            EX_MEM_pc_sel      <= ID_EX_pc_sel;
            EX_MEM_legal       <= ID_EX_legal;
            EX_MEM_halt        <= ID_EX_halt;
            EX_MEM_rs1         <= ID_EX_rs1;
            EX_MEM_rs2         <= ID_EX_rs2;
            EX_MEM_rs1_data    <= r_forward_rs1;
            EX_MEM_rs1_is_read <= ID_EX_rs1_is_read;
            EX_MEM_rs2_is_read <= ID_EX_rs2_is_read;
            EX_MEM_trap        <= ID_EX_trap;
        end
    end

    // ========================================================================
    // MEM Stage: Data Memory Interface & Sub-word Alignment
    // ========================================================================

    // MEM -> MEM Store Data Forwarding (Section 4.2.5)
    assign r_store_data =
        (FWD_EN &&
         (EX_MEM_rs2 != 5'd0) &&
         MEM_WB_valid &&
         (!MEM_WB_trap) &&
         (!MEM_WB_halt) &&
         (MEM_WB_rd != 5'd0) &&
         (MEM_WB_rd_sel != 4'd0) &&
         (MEM_WB_rd == EX_MEM_rs2))
        ? r_memwb_fwd_data :
        EX_MEM_rs2_data;

    assign mem_byte_offset = EX_MEM_result[1:0];

    // Misalignment check (Section 4.5.1):
    // Half-word access requires 2-byte alignment (LSB must be 0).
    // Word access requires 4-byte alignment (LSBs must be 00).
    // Byte access is never misaligned.
    assign misaligned_access = EX_MEM_valid && (!EX_MEM_trap) &&
        (EX_MEM_dmem_ren | EX_MEM_dmem_wen) && (
            (EX_MEM_dmem_memh & mem_byte_offset[0]) |
            (EX_MEM_dmem_memw & (mem_byte_offset != 2'b00))
        );

    assign trap_in_mem = EX_MEM_trap | misaligned_access;

    // Word-aligned address output to memory
    assign o_dmem_addr = {EX_MEM_result[31:2], 2'b00};

    // Suppress memory access on trap or halt
    assign o_dmem_ren  = EX_MEM_valid & EX_MEM_dmem_ren & (!trap_in_mem) & (!EX_MEM_halt);
    assign o_dmem_wen  = EX_MEM_valid & EX_MEM_dmem_wen & (!trap_in_mem) & (!EX_MEM_halt);

    // Byte lane mask generation
    assign o_dmem_mask = EX_MEM_dmem_memw ? 4'b1111 :
                         EX_MEM_dmem_memh ? (mem_byte_offset[1] ? 4'b1100 : 4'b0011) :
                         EX_MEM_dmem_memb ? (
                             (mem_byte_offset == 2'b00) ? 4'b0001 :
                             (mem_byte_offset == 2'b01) ? 4'b0010 :
                             (mem_byte_offset == 2'b10) ? 4'b0100 : 4'b1000
                         ) : 4'b0000;

    // Store data positioning into active byte lanes
    assign o_dmem_wdata = EX_MEM_dmem_memw ? r_store_data :
                          EX_MEM_dmem_memh ? (mem_byte_offset[1] ? {r_store_data[15:0], 16'b0} : {16'b0, r_store_data[15:0]}) :
                          EX_MEM_dmem_memb ? (
                              (mem_byte_offset == 2'b00) ? {24'b0, r_store_data[7:0]} :
                              (mem_byte_offset == 2'b01) ? {16'b0, r_store_data[7:0], 8'b0} :
                              (mem_byte_offset == 2'b10) ? {8'b0, r_store_data[7:0], 16'b0} :
                                                           {r_store_data[7:0], 24'b0}
                          ) : r_store_data;

    // Load data byte lane extraction and sign/zero extension
    assign raw_byte = (mem_byte_offset == 2'b00) ? i_dmem_rdata[ 7: 0] :
                      (mem_byte_offset == 2'b01) ? i_dmem_rdata[15: 8] :
                      (mem_byte_offset == 2'b10) ? i_dmem_rdata[23:16] :
                                                   i_dmem_rdata[31:24];

    assign raw_half = mem_byte_offset[1] ? i_dmem_rdata[31:16] : i_dmem_rdata[15:0];

    assign mem_rdata_ext = EX_MEM_dmem_memw ? i_dmem_rdata :
                           EX_MEM_dmem_memh ? (EX_MEM_dmem_memu ? {16'b0, raw_half} : {{16{raw_half[15]}}, raw_half}) :
                           EX_MEM_dmem_memb ? (EX_MEM_dmem_memu ? {24'b0, raw_byte} : {{24{raw_byte[7]}}, raw_byte}) :
                                              32'd0;

    // ========================================================================
    // MEM/WB Pipeline Registers
    // ========================================================================

    always @(posedge i_clk) begin
        if (i_rst) begin
            MEM_WB_PC          <= 32'd0;
            MEM_WB_next_pc     <= 32'd0;
            MEM_WB_link_pc     <= 32'd0;
            MEM_WB_instruct    <= 32'h00000013; // NOP instruction
            MEM_WB_valid       <= 1'b0;

            MEM_WB_result      <= 32'd0;
            MEM_WB_mem_rdata   <= 32'd0;
            MEM_WB_rd          <= 5'd0;
            MEM_WB_immediate   <= 32'd0;
            MEM_WB_rd_sel      <= 4'd0;

            MEM_WB_trap        <= 1'b0;
            MEM_WB_legal       <= 1'b0;
            MEM_WB_halt        <= 1'b0;

            MEM_WB_rs1         <= 5'd0;
            MEM_WB_rs2         <= 5'd0;
            MEM_WB_rs1_data    <= 32'd0;
            MEM_WB_rs2_data    <= 32'd0;
            MEM_WB_rs1_is_read <= 1'b0;
            MEM_WB_rs2_is_read <= 1'b0;

            MEM_WB_dmem_addr   <= 32'd0;
            MEM_WB_dmem_ren    <= 1'b0;
            MEM_WB_dmem_wen    <= 1'b0;
            MEM_WB_dmem_mask   <= 4'd0;
            MEM_WB_dmem_rdata  <= 32'd0;
            MEM_WB_dmem_wdata  <= 32'd0;
        end else begin
            // Latch values from EX/MEM stage to MEM/WB stage
            MEM_WB_PC          <= EX_MEM_PC;
            MEM_WB_next_pc     <= EX_MEM_next_pc;
            MEM_WB_link_pc     <= EX_MEM_link_pc;
            MEM_WB_instruct    <= EX_MEM_instruct;
            MEM_WB_valid       <= EX_MEM_valid;

            MEM_WB_result      <= EX_MEM_result;
            MEM_WB_mem_rdata   <= mem_rdata_ext; // Load data from memory
            MEM_WB_rd          <= EX_MEM_rd;
            MEM_WB_immediate   <= EX_MEM_immediate;
            MEM_WB_rd_sel      <= EX_MEM_rd_sel;

            // Propagate trap and halt conditions
            MEM_WB_trap        <= trap_in_mem;
            MEM_WB_legal       <= EX_MEM_legal;
            MEM_WB_halt        <= EX_MEM_halt;

            // Propagate source register information for retirement
            MEM_WB_rs1         <= EX_MEM_rs1;
            MEM_WB_rs2         <= EX_MEM_rs2;
            MEM_WB_rs1_data    <= EX_MEM_rs1_data;
            MEM_WB_rs2_data    <= r_store_data; // Reflects MEM->MEM forwarding!
            MEM_WB_rs1_is_read <= EX_MEM_rs1_is_read;
            MEM_WB_rs2_is_read <= EX_MEM_rs2_is_read;

            // Latch memory operation control signals for retirement
            MEM_WB_dmem_addr   <= o_dmem_addr;
            MEM_WB_dmem_ren    <= o_dmem_ren;
            MEM_WB_dmem_wen    <= o_dmem_wen;
            MEM_WB_dmem_mask   <= o_dmem_mask;
            MEM_WB_dmem_rdata  <= i_dmem_rdata;
            MEM_WB_dmem_wdata  <= o_dmem_wdata;
        end
    end

    // ========================================================================
    // 5. Writeback Multiplexer
    // ========================================================================

    // Selects destination data based on one-hot rd_sel:
    // [0] = ALU result
    // [1] = immediate (LUI)
    // [2] = PC + 4 (JAL / JALR link address)
    // [3] = memory load data
    assign rd_data =
        MEM_WB_rd_sel[0] ? MEM_WB_result :
        MEM_WB_rd_sel[1] ? MEM_WB_immediate :
        MEM_WB_rd_sel[2] ? MEM_WB_link_pc :
        MEM_WB_rd_sel[3] ? MEM_WB_mem_rdata :
                           32'd0;

    // MEM/WB forwarding uses the same final value that is written back.
    assign r_memwb_fwd_data = rd_data;

    // ========================================================================
    // 6. Retire Interface (Section 4.5 & 4.5.1)
    // ========================================================================

    assign o_retire_valid     = MEM_WB_valid;
    assign o_retire_inst      = MEM_WB_instruct;
    assign o_retire_trap      = MEM_WB_trap;
    assign o_retire_halt      = MEM_WB_halt;

    // For illegal instruction, rs1/rs2 raddr must report 0 (Section 4.5.1)
    assign o_retire_rs1_raddr = (MEM_WB_valid && MEM_WB_rs1_is_read && (!MEM_WB_trap || MEM_WB_legal)) ? MEM_WB_rs1 : 5'd0;
    assign o_retire_rs1_rdata = (MEM_WB_valid && MEM_WB_rs1_is_read) ? MEM_WB_rs1_data : 32'd0;

    assign o_retire_rs2_raddr = (MEM_WB_valid && MEM_WB_rs2_is_read && (!MEM_WB_trap || MEM_WB_legal)) ? MEM_WB_rs2 : 5'd0;
    assign o_retire_rs2_rdata = (MEM_WB_valid && MEM_WB_rs2_is_read) ? MEM_WB_rs2_data : 32'd0;

    // On trap, no register write: rd_waddr is 5'd0 (Section 4.5.1)
    assign o_retire_rd_waddr  = (!MEM_WB_valid || MEM_WB_trap) ? 5'd0 : MEM_WB_rd;
    assign o_retire_rd_wdata  = rd_data;

    assign o_retire_pc        = MEM_WB_PC;
    // Trapped instructions continue at PC + 4 (Section 4.5.1)
    assign o_retire_next_pc   = MEM_WB_trap ? (MEM_WB_PC + 32'd4) : MEM_WB_next_pc;

    // Retire data memory interface signals (timed to writeback cycle)
    assign o_retire_dmem_addr  = MEM_WB_dmem_addr;
    assign o_retire_dmem_mask  = MEM_WB_dmem_mask;
    assign o_retire_dmem_ren   = MEM_WB_dmem_ren;
    assign o_retire_dmem_wen   = MEM_WB_dmem_wen;
    assign o_retire_dmem_rdata = MEM_WB_dmem_rdata;
    assign o_retire_dmem_wdata = MEM_WB_dmem_wdata;

endmodule

`default_nettype wire
