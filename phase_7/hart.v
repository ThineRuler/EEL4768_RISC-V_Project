`default_nettype none

// Tomasulo-style Out-of-Order RV32IMF CPU (Phase 7)
// - Combinational instruction memory fetch (o_imem_raddr / i_imem_rdata)
// - Non-speculative control flow (ctrl_inflight stalls issue until branch/jump resolves)
// - 8-entry ROB, 4-entry INT ALU RS, 4-entry FP RS, 8-entry in-order LSU
// - Separate INT & FP RATs (rat.v), single CDB with age arbitration (cdb_arb.v)
// - Dual FPU instances (fpu.v) + Phase 5 data cache (cache/cache.v) on LSU
module hart #(
    parameter RESET_ADDR     = 32'h00000000,
    parameter BYPASS_EN      = 1,
    parameter EVICT_POLICY   = 0,
    parameter NUM_WAYS       = 1,
    parameter CACHE_SIZE     = 16,
    parameter BLOCK_SIZE     = 128,
    parameter PREFETCH_EN    = 0,
    parameter WRITE_ALLOCATE = 0
) (
    input  wire                        i_clk,
    input  wire                        i_rst,

    // Combinational instruction memory port
    output wire [31:0]                 o_imem_raddr,
    input  wire [31:0]                 i_imem_rdata,

    // Data cache main memory port
    input  wire                        i_mem_ready,
    input  wire                        i_mem_valid,
    input  wire [BLOCK_SIZE*8-1:0]     i_mem_rd_data,
    output wire                        o_mem_rd,
    output wire                        o_mem_wr,
    output wire [31:0]                 o_mem_rd_addr,
    output wire [31:0]                 o_mem_wr_addr,
    output wire [BLOCK_SIZE*8-1:0]     o_mem_wr_data,
    output wire [BLOCK_SIZE-1:0]       o_mem_wr_be,

    // Retire interface (in-order from ROB head)
    output wire                        o_retire_valid,
    output wire [31:0]                 o_retire_inst,
    output wire                        o_retire_trap,
    output wire                        o_retire_halt,
    output wire [ 4:0]                 o_retire_rs1_raddr,
    output wire [ 4:0]                 o_retire_rs2_raddr,
    output wire [31:0]                 o_retire_rs1_rdata,
    output wire [31:0]                 o_retire_rs2_rdata,
    output wire [ 4:0]                 o_retire_rd_waddr,
    output wire [31:0]                 o_retire_rd_wdata,
    output wire [31:0]                 o_retire_dmem_addr,
    output wire [ 3:0]                 o_retire_dmem_mask,
    output wire                        o_retire_dmem_ren,
    output wire                        o_retire_dmem_wen,
    output wire [31:0]                 o_retire_dmem_rdata,
    output wire [31:0]                 o_retire_dmem_wdata,
    output wire [31:0]                 o_retire_pc,
    output wire [31:0]                 o_retire_next_pc,

    // Complete interface (CDB broadcast)
    output wire                        o_complete_valid,
    output wire [ 2:0]                 o_complete_tag,
    output wire [ 4:0]                 o_complete_rd,
    output wire [31:0]                 o_complete_value,
    output wire [31:0]                 o_complete_pc,

    // F extension retire fields
    output wire [ 4:0]                 o_retire_frs1_raddr,
    output wire [31:0]                 o_retire_frs1_rdata,
    output wire [ 4:0]                 o_retire_frs2_raddr,
    output wire [31:0]                 o_retire_frs2_rdata,
    output wire [ 4:0]                 o_retire_frs3_raddr,
    output wire [31:0]                 o_retire_frs3_rdata,
    output wire                        o_retire_frd_wen,
    output wire [ 4:0]                 o_retire_frd_waddr,
    output wire [31:0]                 o_retire_frd_wdata,
    output wire [ 4:0]                 o_retire_fflags

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);

    localparam ROB_SIZE    = 8;
    localparam ALU_RS_SIZE = 4;
    localparam FP_RS_SIZE  = 4;
    localparam LSU_SIZE    = 8;
    localparam TAG_W       = 3;

    // ========================================================================
    // 1. Frontend: PC, Combinational IMEM, IF/ID Register
    // ========================================================================
    reg  [31:0] PC;
    reg  [31:0] IF_ID_PC;
    reg  [31:0] IF_ID_instruct;
    reg         IF_ID_valid;
    reg         ctrl_inflight;
    reg         halted_issued;

    wire [31:0] pc_plus_4 = PC + 32'd4;
    assign o_imem_raddr = PC;

    // ========================================================================
    // 2. Decoder & Instruction Classification
    // ========================================================================
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
    wire [4:0]  r_rd_sel;
    wire        r_pc_sel;
    wire [31:0] r_immediate;
    wire [4:0]  r_rs1;
    wire [4:0]  r_rs2;
    wire [4:0]  r_rd;
    wire        r_mext;
    wire        r_fpu;
    wire        r_frs1_en;
    wire        r_frs2_en;
    wire        r_frs3_en;
    wire [4:0]  r_rs3;
    wire        r_frd_en;

    decoder decoder_i (
        .i_inst            (IF_ID_instruct),
        .o_legal           (r_legal),
        .o_halt            (r_halt),
        .o_immediate       (r_immediate),
        .o_rs1             (r_rs1),
        .o_rs2             (r_rs2),
        .o_rd              (r_rd),
        .o_op1_sel         (r_op1_sel),
        .o_op2_sel         (r_op2_sel),
        .o_alu_opsel       (r_alu_opsel),
        .o_mext            (r_mext),
        .o_fpu             (r_fpu),
        .o_alu_sub         (r_alu_sub),
        .o_alu_unsigned    (r_alu_unsigned),
        .o_alu_arith       (r_alu_arith),
        .o_frs1_en         (r_frs1_en),
        .o_frs2_en         (r_frs2_en),
        .o_frs3_en         (r_frs3_en),
        .o_rs3             (r_rs3),
        .o_frd_en          (r_frd_en),
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

    wire [6:0] inst_opcode = IF_ID_instruct[6:0];

    wire rs1_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0010011) | // OP-IMM
        (inst_opcode == 7'b0000011) | // LOAD
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011) | // BRANCH
        (inst_opcode == 7'b1100111) | // JALR
        (inst_opcode == 7'b0000111) | // LOAD_FP
        (inst_opcode == 7'b0100111) | // STORE_FP
        (r_fpu && !r_frs1_en && !((inst_opcode == 7'b1110011) && IF_ID_instruct[14]))
    );

    wire rs2_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011)   // BRANCH
    );

    wire cls_illegal_or_halt = !r_legal || r_halt;
    wire cls_mem             = r_legal && (r_dmem_ren || r_dmem_wen);
    wire cls_fpu             = r_legal && r_fpu;
    wire cls_brjmp           = r_legal && (r_branch || r_jump);
    wire cls_alu             = r_legal && !r_halt && !r_dmem_ren && !r_dmem_wen && !r_fpu && !r_branch && !r_jump;

    // ========================================================================
    // 3. ROB Storage & Commit Signals
    // ========================================================================
    reg  [2:0]  rob_head;
    reg  [2:0]  rob_tail;
    reg  [3:0]  rob_count;

    reg         rob_valid       [0:ROB_SIZE-1];
    reg         rob_done        [0:ROB_SIZE-1];
    reg         rob_trap        [0:ROB_SIZE-1];
    reg         rob_halt        [0:ROB_SIZE-1];
    reg         rob_legal       [0:ROB_SIZE-1];
    reg  [31:0] rob_pc          [0:ROB_SIZE-1];
    reg  [31:0] rob_next_pc     [0:ROB_SIZE-1];
    reg  [31:0] rob_inst        [0:ROB_SIZE-1];
    reg  [ 4:0] rob_rd          [0:ROB_SIZE-1];
    reg         rob_frd_en      [0:ROB_SIZE-1];
    reg         rob_int_wen     [0:ROB_SIZE-1];
    reg  [31:0] rob_value       [0:ROB_SIZE-1];
    reg  [ 4:0] rob_rs1         [0:ROB_SIZE-1];
    reg  [ 4:0] rob_rs2         [0:ROB_SIZE-1];
    reg  [ 4:0] rob_rs3         [0:ROB_SIZE-1];
    reg         rob_rs1_is_read [0:ROB_SIZE-1];
    reg         rob_rs2_is_read [0:ROB_SIZE-1];
    reg         rob_frs1_en     [0:ROB_SIZE-1];
    reg         rob_frs2_en     [0:ROB_SIZE-1];
    reg         rob_frs3_en     [0:ROB_SIZE-1];
    reg  [31:0] rob_op1_val     [0:ROB_SIZE-1];
    reg         rob_op1_qv      [0:ROB_SIZE-1];
    reg  [ 2:0] rob_op1_qt      [0:ROB_SIZE-1];
    reg  [31:0] rob_op2_val     [0:ROB_SIZE-1];
    reg         rob_op2_qv      [0:ROB_SIZE-1];
    reg  [ 2:0] rob_op2_qt      [0:ROB_SIZE-1];
    reg  [31:0] rob_op3_val     [0:ROB_SIZE-1];
    reg         rob_op3_qv      [0:ROB_SIZE-1];
    reg  [ 2:0] rob_op3_qt      [0:ROB_SIZE-1];
    reg         rob_is_load     [0:ROB_SIZE-1];
    reg         rob_is_store    [0:ROB_SIZE-1];
    reg         rob_is_fpu      [0:ROB_SIZE-1];
    reg         rob_is_fpu_csr  [0:ROB_SIZE-1];
    reg  [ 4:0] rob_fflags      [0:ROB_SIZE-1];
    reg  [ 2:0] rob_frm_new     [0:ROB_SIZE-1];
    reg         rob_frm_wen     [0:ROB_SIZE-1];
    reg  [31:0] rob_dmem_addr   [0:ROB_SIZE-1];
    reg  [ 3:0] rob_dmem_mask   [0:ROB_SIZE-1];
    reg         rob_dmem_ren    [0:ROB_SIZE-1];
    reg         rob_dmem_wen    [0:ROB_SIZE-1];
    reg  [31:0] rob_dmem_rdata  [0:ROB_SIZE-1];
    reg  [31:0] rob_dmem_wdata  [0:ROB_SIZE-1];

    wire rob_full  = (rob_count == 4'd8);
    wire rob_empty = (rob_count == 4'd0);

    // Architectural FCSR (authoritative at ROB commit)
    reg  [ 7:0] arch_fcsr;

    // Forward declarations for CDB, RS, LSU, Cache
    wire        cdb_valid;
    wire [ 2:0] cdb_tag;
    wire [ 4:0] cdb_rd;
    wire [31:0] cdb_val;
    wire [31:0] cdb_pc;
    wire        o_alu_gnt, o_fp0_gnt, o_fp1_gnt, o_ld_gnt;

    wire        alu_rs_full;
    wire        fp_rs_full;
    wire        lsu_full;
    wire        dcache_ready;

    // Store commit needs D-cache ready (if non-trapping)
    wire rob_commit_store_req = rob_valid[rob_head] && rob_done[rob_head] &&
                                rob_is_store[rob_head] && !rob_trap[rob_head];
    wire rob_commit = rob_valid[rob_head] && rob_done[rob_head] &&
                      (!rob_is_store[rob_head] || rob_trap[rob_head] || dcache_ready);

    // On trapped instructions that had renamed a destination (e.g. misaligned lw/flw or fpu_rm_trap),
    // restore architectural RF[rd] to any younger dependent instructions during commit cycle.
    wire trap_wb_needed = rob_valid[rob_head] && rob_done[rob_head] &&
                          rob_trap[rob_head] && (rob_int_wen[rob_head] || rob_frd_en[rob_head]);
    wire trap_wb_stall  = trap_wb_needed;
    wire trap_wb_fire   = rob_commit && trap_wb_needed;

    wire issue_stall = IF_ID_valid && (
        halted_issued ||
        trap_wb_stall ||
        ctrl_inflight ||
        rob_full ||
        ((cls_alu || cls_brjmp) && alu_rs_full) ||
        (cls_fpu && fp_rs_full) ||
        (cls_mem && lsu_full)
    );
    wire issue_fire = IF_ID_valid && !issue_stall;

    // ========================================================================
    // 4. Register File & Register Alias Tables (INT & FP)
    // ========================================================================
    wire [31:0] rs1_data, rs2_data, frs1_data, frs2_data, frs3_data;
    wire [ 4:0] rf_rs1_raddr  = trap_wb_stall ? rob_rd[rob_head] : r_rs1;
    wire [ 4:0] rf_frs1_raddr = trap_wb_stall ? rob_rd[rob_head] : r_rs1;

    wire [ 4:0] actual_rd_waddr  = (rob_commit && !rob_trap[rob_head] && !rob_frd_en[rob_head]) ? rob_rd[rob_head] : 5'd0;
    wire [31:0] actual_rd_wdata  = rob_value[rob_head];
    wire        actual_frd_wen   = rob_commit && !rob_trap[rob_head] && rob_frd_en[rob_head];
    wire [ 4:0] actual_frd_waddr = rob_rd[rob_head];
    wire [31:0] actual_frd_wdata = rob_value[rob_head];

    rf #(.BYPASS_EN(BYPASS_EN)) rf_connect (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .o_rs1_rdata  (rs1_data),
        .o_rs2_rdata  (rs2_data),
        .i_rs1_raddr  (rf_rs1_raddr),
        .i_rs2_raddr  (r_rs2),
        .i_rd_waddr   (actual_rd_waddr),
        .i_rd_wdata   (actual_rd_wdata),
        .i_frs1_raddr (rf_frs1_raddr),
        .o_frs1_rdata (frs1_data),
        .i_frs2_raddr (r_rs2),
        .o_frs2_rdata (frs2_data),
        .i_frs3_raddr (r_rs3),
        .o_frs3_rdata (frs3_data),
        .i_frd_wen    (actual_frd_wen),
        .i_frd_waddr  (actual_frd_waddr),
        .i_frd_wdata  (actual_frd_wdata)
    );

    wire [31:0] trap_wb_val = rob_frd_en[rob_head] ? frs1_data : rs1_data;

    wire        rat_int_busy1, rat_int_busy2;
    wire [ 2:0] rat_int_tag1,  rat_int_tag2;
    wire        rat_fp_busy1,  rat_fp_busy2, rat_fp_busy3;
    wire [ 2:0] rat_fp_tag1,   rat_fp_tag2,  rat_fp_tag3;

    rat #(.TAG_W(3), .ZERO_REG(1)) rat_int (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .i_raddr1     (r_rs1),
        .o_busy1      (rat_int_busy1),
        .o_tag1       (rat_int_tag1),
        .i_raddr2     (r_rs2),
        .o_busy2      (rat_int_busy2),
        .o_tag2       (rat_int_tag2),
        .i_raddr3     (5'd0),
        .o_busy3      (),
        .o_tag3       (),
        .i_issue_en   (issue_fire && !r_frd_en && (r_rd != 5'd0)),
        .i_issue_rd   (r_rd),
        .i_issue_tag  (rob_tail),
        .i_commit_en  (rob_commit && !rob_frd_en[rob_head] && (rob_rd[rob_head] != 5'd0)),
        .i_commit_rd  (rob_rd[rob_head]),
        .i_commit_tag (rob_head)
    );

    rat #(.TAG_W(3), .ZERO_REG(0)) rat_fp (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .i_raddr1     (r_rs1),
        .o_busy1      (rat_fp_busy1),
        .o_tag1       (rat_fp_tag1),
        .i_raddr2     (r_rs2),
        .o_busy2      (rat_fp_busy2),
        .o_tag2       (rat_fp_tag2),
        .i_raddr3     (r_rs3),
        .o_busy3      (rat_fp_busy3),
        .o_tag3       (rat_fp_tag3),
        .i_issue_en   (issue_fire && r_frd_en),
        .i_issue_rd   (r_rd),
        .i_issue_tag  (rob_tail),
        .i_commit_en  (rob_commit && rob_frd_en[rob_head]),
        .i_commit_rd  (rob_rd[rob_head]),
        .i_commit_tag (rob_head)
    );

    // ========================================================================
    // 5. Issue Operand Resolution (RF / ROB / Same-Cycle CDB Bypass / Q-tag)
    // ========================================================================
    wire        op1_need   = rs1_is_read || r_frs1_en;
    wire        op1_busy   = op1_need && (r_frs1_en ? rat_fp_busy1 : rat_int_busy1);
    wire [ 2:0] op1_tag    = r_frs1_en ? rat_fp_tag1 : rat_int_tag1;
    wire [31:0] op1_rf_val = r_frs1_en ? frs1_data : ((r_rs1 == 5'd0) ? 32'd0 : rs1_data);
    wire        op1_in_rob = op1_busy && rob_done[op1_tag] && !rob_trap[op1_tag];
    wire        op1_on_cdb = op1_busy && !op1_in_rob && cdb_valid && (cdb_tag == op1_tag);
    wire        iss_op1_qv = op1_busy && !op1_in_rob && !op1_on_cdb;
    wire [ 2:0] iss_op1_qt = op1_tag;
    wire [31:0] iss_op1_val = !op1_need  ? 32'd0 :
                              !op1_busy  ? op1_rf_val :
                              op1_in_rob ? rob_value[op1_tag] :
                              op1_on_cdb ? cdb_val :
                                           32'd0;

    wire        op2_need   = rs2_is_read || r_frs2_en;
    wire        op2_busy   = op2_need && (r_frs2_en ? rat_fp_busy2 : rat_int_busy2);
    wire [ 2:0] op2_tag    = r_frs2_en ? rat_fp_tag2 : rat_int_tag2;
    wire [31:0] op2_rf_val = r_frs2_en ? frs2_data : ((r_rs2 == 5'd0) ? 32'd0 : rs2_data);
    wire        op2_in_rob = op2_busy && rob_done[op2_tag] && !rob_trap[op2_tag];
    wire        op2_on_cdb = op2_busy && !op2_in_rob && cdb_valid && (cdb_tag == op2_tag);
    wire        iss_op2_qv = op2_busy && !op2_in_rob && !op2_on_cdb;
    wire [ 2:0] iss_op2_qt = op2_tag;
    wire [31:0] iss_op2_val = !op2_need  ? 32'd0 :
                              !op2_busy  ? op2_rf_val :
                              op2_in_rob ? rob_value[op2_tag] :
                              op2_on_cdb ? cdb_val :
                                           32'd0;

    wire        op3_need   = r_frs3_en;
    wire        op3_busy   = op3_need && rat_fp_busy3;
    wire [ 2:0] op3_tag    = rat_fp_tag3;
    wire        op3_in_rob = op3_busy && rob_done[op3_tag] && !rob_trap[op3_tag];
    wire        op3_on_cdb = op3_busy && !op3_in_rob && cdb_valid && (cdb_tag == op3_tag);
    wire        iss_op3_qv = op3_busy && !op3_in_rob && !op3_on_cdb;
    wire [ 2:0] iss_op3_qt = op3_tag;
    wire [31:0] iss_op3_val = !op3_need  ? 32'd0 :
                              !op3_busy  ? frs3_data :
                              op3_in_rob ? rob_value[op3_tag] :
                              op3_on_cdb ? cdb_val :
                                           32'd0;

    // ========================================================================
    // 6. INT ALU Reservation Station (4 entries) + Shared ALU + Registered CDB
    // ========================================================================
    reg         alu_rs_valid       [0:ALU_RS_SIZE-1];
    reg  [ 2:0] alu_rs_tag         [0:ALU_RS_SIZE-1];
    reg  [31:0] alu_rs_pc          [0:ALU_RS_SIZE-1];
    reg  [ 4:0] alu_rs_rd          [0:ALU_RS_SIZE-1];
    reg  [31:0] alu_rs_imm         [0:ALU_RS_SIZE-1];
    reg         alu_rs_op1_sel     [0:ALU_RS_SIZE-1];
    reg         alu_rs_op2_sel     [0:ALU_RS_SIZE-1];
    reg  [ 2:0] alu_rs_opsel       [0:ALU_RS_SIZE-1];
    reg         alu_rs_sub         [0:ALU_RS_SIZE-1];
    reg         alu_rs_unsigned    [0:ALU_RS_SIZE-1];
    reg         alu_rs_arith       [0:ALU_RS_SIZE-1];
    reg         alu_rs_mext        [0:ALU_RS_SIZE-1];
    reg         alu_rs_branch      [0:ALU_RS_SIZE-1];
    reg         alu_rs_jump        [0:ALU_RS_SIZE-1];
    reg         alu_rs_br_eq       [0:ALU_RS_SIZE-1];
    reg         alu_rs_br_inv      [0:ALU_RS_SIZE-1];
    reg  [ 4:0] alu_rs_rd_sel      [0:ALU_RS_SIZE-1];
    reg         alu_rs_pc_sel      [0:ALU_RS_SIZE-1];
    reg  [31:0] alu_rs_op1_val     [0:ALU_RS_SIZE-1];
    reg         alu_rs_q1_v        [0:ALU_RS_SIZE-1];
    reg  [ 2:0] alu_rs_q1_t        [0:ALU_RS_SIZE-1];
    reg  [31:0] alu_rs_op2_val     [0:ALU_RS_SIZE-1];
    reg         alu_rs_q2_v        [0:ALU_RS_SIZE-1];
    reg  [ 2:0] alu_rs_q2_t        [0:ALU_RS_SIZE-1];

    assign alu_rs_full = alu_rs_valid[0] && alu_rs_valid[1] && alu_rs_valid[2] && alu_rs_valid[3];

    wire [1:0] alu_rs_free_idx =
        !alu_rs_valid[0] ? 2'd0 :
        !alu_rs_valid[1] ? 2'd1 :
        !alu_rs_valid[2] ? 2'd2 :
                           2'd3;

    // Ready status with CDB -> ALU RS operand bypass
    wire [3:0] alu_rs_ready;
    wire [2:0] alu_rs_age [0:3];
    genvar ak;
    generate
        for (ak = 0; ak < 4; ak = ak + 1) begin : gen_alu_rs_rdy
            wire q1_rdy = !alu_rs_q1_v[ak] || (cdb_valid && (cdb_tag == alu_rs_q1_t[ak]));
            wire q2_rdy = !alu_rs_q2_v[ak] || (cdb_valid && (cdb_tag == alu_rs_q2_t[ak]));
            assign alu_rs_ready[ak] = alu_rs_valid[ak] && q1_rdy && q2_rdy;
            assign alu_rs_age[ak]   = alu_rs_tag[ak] - rob_head;
        end
    endgenerate

    // Oldest-ready selection
    wire alu_pick0 = alu_rs_ready[0] &&
                     (!alu_rs_ready[1] || (alu_rs_age[0] <= alu_rs_age[1])) &&
                     (!alu_rs_ready[2] || (alu_rs_age[0] <= alu_rs_age[2])) &&
                     (!alu_rs_ready[3] || (alu_rs_age[0] <= alu_rs_age[3]));
    wire alu_pick1 = !alu_pick0 && alu_rs_ready[1] &&
                     (!alu_rs_ready[2] || (alu_rs_age[1] <= alu_rs_age[2])) &&
                     (!alu_rs_ready[3] || (alu_rs_age[1] <= alu_rs_age[3]));
    wire alu_pick2 = !alu_pick0 && !alu_pick1 && alu_rs_ready[2] &&
                     (!alu_rs_ready[3] || (alu_rs_age[2] <= alu_rs_age[3]));
    wire alu_pick3 = !alu_pick0 && !alu_pick1 && !alu_pick2 && alu_rs_ready[3];

    reg        alu_div_active;
    reg  [1:0] alu_div_slot;

    wire       alu_sel_valid = alu_div_active || (|alu_rs_ready);
    wire [1:0] alu_sel_slot  = alu_div_active ? alu_div_slot :
                               alu_pick0 ? 2'd0 :
                               alu_pick1 ? 2'd1 :
                               alu_pick2 ? 2'd2 : 2'd3;

    // Registered ALU CDB output buffer
    reg        alu_cdb_v;
    reg  [2:0] alu_cdb_tag;
    reg  [4:0] alu_cdb_rd;
    reg  [31:0] alu_cdb_val;
    reg  [31:0] alu_cdb_pc;

    wire alu_cdb_can_accept = !alu_cdb_v || o_alu_gnt;

    wire [31:0] alu_sel_op1_eff =
        (alu_rs_q1_v[alu_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q1_t[alu_sel_slot])) ?
        cdb_val : alu_rs_op1_val[alu_sel_slot];
    wire [31:0] alu_sel_op2_eff =
        (alu_rs_q2_v[alu_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q2_t[alu_sel_slot])) ?
        cdb_val : alu_rs_op2_val[alu_sel_slot];

    wire [31:0] alu_op1 = alu_rs_op1_sel[alu_sel_slot] ? alu_rs_pc[alu_sel_slot]  : alu_sel_op1_eff;
    wire [31:0] alu_op2 = alu_rs_op2_sel[alu_sel_slot] ? alu_rs_imm[alu_sel_slot] : alu_sel_op2_eff;

    wire        alu_sel_is_div = alu_sel_valid && alu_rs_mext[alu_sel_slot] && alu_rs_opsel[alu_sel_slot][2];
    wire [31:0] alu_out_result;
    wire        alu_out_eq, alu_out_slt, alu_out_busy;

    wire        alu_disp_fire = alu_sel_valid && !alu_out_busy && alu_cdb_can_accept;

    alu alu_i (
        .i_clk      (i_clk),
        .i_rst      (i_rst),
        .i_opsel    (alu_sel_valid ? alu_rs_opsel[alu_sel_slot]    : 3'b000),
        .i_sub      (alu_sel_valid ? alu_rs_sub[alu_sel_slot]      : 1'b0),
        .i_unsigned (alu_sel_valid ? alu_rs_unsigned[alu_sel_slot] : 1'b0),
        .i_arith    (alu_sel_valid ? alu_rs_arith[alu_sel_slot]    : 1'b0),
        .i_mext     (alu_sel_valid ? alu_rs_mext[alu_sel_slot]     : 1'b0),
        .i_advance  (alu_disp_fire),
        .i_op1      (alu_op1),
        .i_op2      (alu_op2),
        .o_result   (alu_out_result),
        .o_eq       (alu_out_eq),
        .o_slt      (alu_out_slt),
        .o_busy     (alu_out_busy)
    );

    // Branch / Jump resolution on ALU dispatch
    wire [31:0] alu_sel_pc       = alu_rs_pc[alu_sel_slot];
    wire [31:0] alu_sel_imm      = alu_rs_imm[alu_sel_slot];
    wire [ 2:0] alu_sel_tag      = alu_rs_tag[alu_sel_slot];
    wire [ 4:0] alu_sel_rd       = alu_rs_rd[alu_sel_slot];
    wire        alu_sel_branch   = alu_rs_branch[alu_sel_slot];
    wire        alu_sel_jump     = alu_rs_jump[alu_sel_slot];
    wire [31:0] alu_br_target    = alu_sel_pc + alu_sel_imm;
    wire [31:0] alu_jmp_target   = alu_rs_pc_sel[alu_sel_slot] ? (alu_out_result & ~32'd1) : alu_br_target;
    wire        alu_br_comp      = alu_rs_br_eq[alu_sel_slot] ? alu_out_eq : alu_out_slt;
    wire        alu_br_taken     = alu_sel_branch && (alu_br_comp ^ alu_rs_br_inv[alu_sel_slot]);
    wire        alu_redirect_raw = alu_sel_jump || alu_br_taken;
    wire [31:0] alu_next_pc      = alu_sel_jump ? alu_jmp_target :
                                   alu_br_taken ? alu_br_target :
                                                  (alu_sel_pc + 32'd4);
    wire        alu_br_trap      = alu_redirect_raw && (alu_next_pc[1:0] != 2'b00);
    wire        alu_resolve_fire = alu_disp_fire && (alu_sel_branch || alu_sel_jump);
    wire        redirect_taken   = alu_resolve_fire && alu_redirect_raw && !alu_br_trap;
    wire [31:0] redirect_pc      = alu_next_pc;

    wire [31:0] alu_cdb_calc_val =
        alu_sel_branch               ? 32'd0 :
        alu_rs_rd_sel[alu_sel_slot][1] ? alu_sel_imm :
        alu_rs_rd_sel[alu_sel_slot][2] ? (alu_sel_pc + 32'd4) :
                                         alu_out_result;

    // ========================================================================
    // 7. FP Reservation Station (4 entries) + Dual FPUs (fpu_0, fpu_1)
    // ========================================================================
    reg         fp_rs_valid      [0:FP_RS_SIZE-1];
    reg  [ 2:0] fp_rs_tag        [0:FP_RS_SIZE-1];
    reg  [31:0] fp_rs_pc         [0:FP_RS_SIZE-1];
    reg  [31:0] fp_rs_inst       [0:FP_RS_SIZE-1];
    reg  [ 4:0] fp_rs_rd         [0:FP_RS_SIZE-1];
    reg         fp_rs_frd_en     [0:FP_RS_SIZE-1];
    reg         fp_rs_is_csr     [0:FP_RS_SIZE-1];
    reg         fp_rs_dyn_rm     [0:FP_RS_SIZE-1];
    reg  [31:0] fp_rs_op1_val    [0:FP_RS_SIZE-1];
    reg         fp_rs_q1_v       [0:FP_RS_SIZE-1];
    reg  [ 2:0] fp_rs_q1_t       [0:FP_RS_SIZE-1];
    reg  [31:0] fp_rs_op2_val    [0:FP_RS_SIZE-1];
    reg         fp_rs_q2_v       [0:FP_RS_SIZE-1];
    reg  [ 2:0] fp_rs_q2_t       [0:FP_RS_SIZE-1];
    reg  [31:0] fp_rs_op3_val    [0:FP_RS_SIZE-1];
    reg         fp_rs_q3_v       [0:FP_RS_SIZE-1];
    reg  [ 2:0] fp_rs_q3_t       [0:FP_RS_SIZE-1];

    assign fp_rs_full = fp_rs_valid[0] && fp_rs_valid[1] && fp_rs_valid[2] && fp_rs_valid[3];

    wire [1:0] fp_rs_free_idx =
        !fp_rs_valid[0] ? 2'd0 :
        !fp_rs_valid[1] ? 2'd1 :
        !fp_rs_valid[2] ? 2'd2 :
                          2'd3;

    reg        fp0_busy_slot_v, fp1_busy_slot_v;
    reg  [1:0] fp0_busy_slot,   fp1_busy_slot;

    reg        fp0_cdb_v, fp1_cdb_v;
    reg  [2:0] fp0_cdb_tag, fp1_cdb_tag;
    reg  [4:0] fp0_cdb_rd,  fp1_cdb_rd;
    reg  [31:0] fp0_cdb_val, fp1_cdb_val;
    reg  [31:0] fp0_cdb_pc,  fp1_cdb_pc;
    reg  [4:0] fp0_cdb_fflags, fp1_cdb_fflags;

    reg        fp0_needs_sync, fp1_needs_sync;

    wire fp0_cdb_can_accept = !fp0_cdb_v || o_fp0_gnt;
    wire fp1_cdb_can_accept = !fp1_cdb_v || o_fp1_gnt;

    wire [3:0] fp_rs_ready;
    wire [3:0] fp_rs_older_csr;
    wire [2:0] fp_rs_age [0:3];
    genvar fk, rj;
    generate
        for (fk = 0; fk < 4; fk = fk + 1) begin : gen_fp_rs_rdy
            wire q1_rdy = !fp_rs_q1_v[fk] || (cdb_valid && (cdb_tag == fp_rs_q1_t[fk]));
            wire q2_rdy = !fp_rs_q2_v[fk] || (cdb_valid && (cdb_tag == fp_rs_q2_t[fk]));
            wire q3_rdy = !fp_rs_q3_v[fk] || (cdb_valid && (cdb_tag == fp_rs_q3_t[fk]));
            wire [7:0] older_csr_vec;
            for (rj = 0; rj < 8; rj = rj + 1) begin : gen_older_csr
                localparam [2:0] RJ_TAG = rj;
                wire [2:0] rj_age = RJ_TAG - rob_head;
                assign older_csr_vec[rj] = rob_valid[rj] && rob_is_fpu_csr[rj] && (rj_age < fp_rs_age[fk]);
            end
            assign fp_rs_age[fk]       = fp_rs_tag[fk] - rob_head;
            assign fp_rs_older_csr[fk] = |older_csr_vec;
            assign fp_rs_ready[fk]     = fp_rs_valid[fk] && q1_rdy && q2_rdy && q3_rdy && !fp_rs_older_csr[fk];
        end
    endgenerate

    // CSR instructions serialize at ROB head when both FPUs and their CDB buffers are idle
    wire csr_can_run = !fp0_busy_slot_v && !fp1_busy_slot_v &&
                       !fp0_cdb_v && !fp1_cdb_v &&
                       !fp0_needs_sync && !fp1_needs_sync;

    wire [3:0] fp0_elig;
    wire [3:0] fp1_elig;
    generate
        for (fk = 0; fk < 4; fk = fk + 1) begin : gen_fp_elig
            localparam [1:0] FK_IDX = fk;
            wire not_on_fp1 = !(fp1_busy_slot_v && (fp1_busy_slot == FK_IDX));
            wire not_on_fp0 = !(fp0_busy_slot_v && (fp0_busy_slot == FK_IDX));
            wire csr_ok0    = !fp_rs_is_csr[fk] || ((fp_rs_tag[fk] == rob_head) && csr_can_run);
            wire rm_ok0     = !fp_rs_dyn_rm[fk] || !fp0_needs_sync;
            wire rm_ok1     = !fp_rs_dyn_rm[fk] || !fp1_needs_sync;
            assign fp0_elig[fk] = fp_rs_ready[fk] && not_on_fp1 && csr_ok0 && rm_ok0;
            assign fp1_elig[fk] = fp_rs_ready[fk] && not_on_fp0 && !fp_rs_is_csr[fk] && rm_ok1;
        end
    endgenerate

    wire fp0_pick0 = fp0_elig[0] &&
                     (!fp0_elig[1] || (fp_rs_age[0] <= fp_rs_age[1])) &&
                     (!fp0_elig[2] || (fp_rs_age[0] <= fp_rs_age[2])) &&
                     (!fp0_elig[3] || (fp_rs_age[0] <= fp_rs_age[3]));
    wire fp0_pick1 = !fp0_pick0 && fp0_elig[1] &&
                     (!fp0_elig[2] || (fp_rs_age[1] <= fp_rs_age[2])) &&
                     (!fp0_elig[3] || (fp_rs_age[1] <= fp_rs_age[3]));
    wire fp0_pick2 = !fp0_pick0 && !fp0_pick1 && fp0_elig[2] &&
                     (!fp0_elig[3] || (fp_rs_age[2] <= fp_rs_age[3]));

    wire       fp0_sync_now  = fp0_needs_sync && !fp0_busy_slot_v && !fp0_cdb_v;
    wire       fp0_sel_valid = !fp0_sync_now && (fp0_busy_slot_v || (|fp0_elig));
    wire [1:0] fp0_sel_slot  = fp0_busy_slot_v ? fp0_busy_slot :
                               fp0_pick0 ? 2'd0 :
                               fp0_pick1 ? 2'd1 :
                               fp0_pick2 ? 2'd2 : 2'd3;

    // FP1 excludes fp0_sel_slot when fp0_sel_valid is asserted
    wire [3:0] fp1_elig_excl;
    generate
        for (fk = 0; fk < 4; fk = fk + 1) begin : gen_fp1_excl
            localparam [1:0] FK_IDX = fk;
            assign fp1_elig_excl[fk] = fp1_elig[fk] && !(fp0_sel_valid && (fp0_sel_slot == FK_IDX));
        end
    endgenerate

    wire fp1_pick0 = fp1_elig_excl[0] &&
                     (!fp1_elig_excl[1] || (fp_rs_age[0] <= fp_rs_age[1])) &&
                     (!fp1_elig_excl[2] || (fp_rs_age[0] <= fp_rs_age[2])) &&
                     (!fp1_elig_excl[3] || (fp_rs_age[0] <= fp_rs_age[3]));
    wire fp1_pick1 = !fp1_pick0 && fp1_elig_excl[1] &&
                     (!fp1_elig_excl[2] || (fp_rs_age[1] <= fp_rs_age[2])) &&
                     (!fp1_elig_excl[3] || (fp_rs_age[1] <= fp_rs_age[3]));
    wire fp1_pick2 = !fp1_pick0 && !fp1_pick1 && fp1_elig_excl[2] &&
                     (!fp1_elig_excl[3] || (fp_rs_age[2] <= fp_rs_age[3]));

    wire       fp1_sync_now  = fp1_needs_sync && !fp1_busy_slot_v && !fp1_cdb_v;
    wire       fp1_sel_valid = !fp1_sync_now && (fp1_busy_slot_v || (|fp1_elig_excl));
    wire [1:0] fp1_sel_slot  = fp1_busy_slot_v ? fp1_busy_slot :
                               fp1_pick0 ? 2'd0 :
                               fp1_pick1 ? 2'd1 :
                               fp1_pick2 ? 2'd2 : 2'd3;

    // Effective operands for FP0 and FP1 (with CDB bypass)
    wire [31:0] fp0_op1_eff = (fp_rs_q1_v[fp0_sel_slot] && cdb_valid && (cdb_tag == fp_rs_q1_t[fp0_sel_slot])) ? cdb_val : fp_rs_op1_val[fp0_sel_slot];
    wire [31:0] fp0_op2_eff = (fp_rs_q2_v[fp0_sel_slot] && cdb_valid && (cdb_tag == fp_rs_q2_t[fp0_sel_slot])) ? cdb_val : fp_rs_op2_val[fp0_sel_slot];
    wire [31:0] fp0_op3_eff = (fp_rs_q3_v[fp0_sel_slot] && cdb_valid && (cdb_tag == fp_rs_q3_t[fp0_sel_slot])) ? cdb_val : fp_rs_op3_val[fp0_sel_slot];

    wire [31:0] fp1_op1_eff = (fp_rs_q1_v[fp1_sel_slot] && cdb_valid && (cdb_tag == fp_rs_q1_t[fp1_sel_slot])) ? cdb_val : fp_rs_op1_val[fp1_sel_slot];
    wire [31:0] fp1_op2_eff = (fp_rs_q2_v[fp1_sel_slot] && cdb_valid && (cdb_tag == fp_rs_q2_t[fp1_sel_slot])) ? cdb_val : fp_rs_op2_val[fp1_sel_slot];
    wire [31:0] fp1_op3_eff = (fp_rs_q3_v[fp1_sel_slot] && cdb_valid && (cdb_tag == fp_rs_q3_t[fp1_sel_slot])) ? cdb_val : fp_rs_op3_val[fp1_sel_slot];

    wire [31:0] fp0_fp_res, fp0_int_res, fp1_fp_res, fp1_int_res;
    wire [ 4:0] fp0_fflags, fp1_fflags;
    wire        fp0_busy,   fp1_busy;
    wire        fp0_rm_trap, fp1_rm_trap;

    wire fp0_disp_fire = fp0_sel_valid && !fp0_busy && fp0_cdb_can_accept;
    wire fp1_disp_fire = fp1_sel_valid && !fp1_busy && fp1_cdb_can_accept;

    // 32'h00309073 = csrrw x0, fcsr, x1 (syncs internal fpu fcsr to arch_fcsr in 1 cycle)
    fpu fpu_0 (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .i_inst       (fp0_sync_now ? 32'h00309073 : fp_rs_inst[fp0_sel_slot]),
        .i_valid      (fp0_sync_now || fp0_sel_valid),
        .i_advance    (fp0_sync_now || fp0_disp_fire),
        .i_frs1       (fp0_op1_eff),
        .i_frs2       (fp0_op2_eff),
        .i_frs3       (fp0_op3_eff),
        .i_xrs1       (fp0_sync_now ? {24'd0, arch_fcsr} : fp0_op1_eff),
        .o_fp_result  (fp0_fp_res),
        .o_int_result (fp0_int_res),
        .o_fflags     (fp0_fflags),
        .o_busy       (fp0_busy),
        .o_rm_trap    (fp0_rm_trap)
    );

    fpu fpu_1 (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .i_inst       (fp1_sync_now ? 32'h00309073 : fp_rs_inst[fp1_sel_slot]),
        .i_valid      (fp1_sync_now || fp1_sel_valid),
        .i_advance    (fp1_sync_now || fp1_disp_fire),
        .i_frs1       (fp1_op1_eff),
        .i_frs2       (fp1_op2_eff),
        .i_frs3       (fp1_op3_eff),
        .i_xrs1       (fp1_sync_now ? {24'd0, arch_fcsr} : fp1_op1_eff),
        .o_fp_result  (fp1_fp_res),
        .o_int_result (fp1_int_res),
        .o_fflags     (fp1_fflags),
        .o_busy       (fp1_busy),
        .o_rm_trap    (fp1_rm_trap)
    );

    wire [31:0] fp0_res_val = fp_rs_frd_en[fp0_sel_slot] ? fp0_fp_res : fp0_int_res;
    wire [31:0] fp1_res_val = fp_rs_frd_en[fp1_sel_slot] ? fp1_fp_res : fp1_int_res;

    // ========================================================================
    // 8. In-Order Load-Store Unit (8 entries) + Phase 5 Data Cache
    // ========================================================================
    reg  [ 2:0] lsu_head, lsu_tail;
    reg  [ 3:0] lsu_count;

    reg         lsu_valid   [0:LSU_SIZE-1];
    reg  [ 2:0] lsu_tag     [0:LSU_SIZE-1];
    reg  [31:0] lsu_pc      [0:LSU_SIZE-1];
    reg  [ 4:0] lsu_rd      [0:LSU_SIZE-1];
    reg         lsu_is_load [0:LSU_SIZE-1];
    reg  [31:0] lsu_imm     [0:LSU_SIZE-1];
    reg         lsu_memb    [0:LSU_SIZE-1];
    reg         lsu_memh    [0:LSU_SIZE-1];
    reg         lsu_memw    [0:LSU_SIZE-1];
    reg         lsu_memu    [0:LSU_SIZE-1];
    reg  [31:0] lsu_op1_val [0:LSU_SIZE-1];
    reg         lsu_q1_v    [0:LSU_SIZE-1];
    reg  [ 2:0] lsu_q1_t    [0:LSU_SIZE-1];
    reg  [31:0] lsu_op2_val [0:LSU_SIZE-1];
    reg         lsu_q2_v    [0:LSU_SIZE-1];
    reg  [ 2:0] lsu_q2_t    [0:LSU_SIZE-1];
    reg         lsu_st_prep [0:LSU_SIZE-1];

    // Store commit latch (captured when store becomes ready in LSU)
    reg  [31:0] lsu_st_byte_addr;
    reg  [ 3:0] lsu_st_mask;
    reg  [31:0] lsu_st_wdata;

    // Load CDB buffer (if dcache_ready asserted while load lost CDB arbitration)
    reg         ld_buf_valid;
    reg  [31:0] ld_buf_val;
    reg  [31:0] ld_buf_raw_rdata;
    reg  [31:0] ld_buf_word_addr;
    reg  [ 3:0] ld_buf_mask;

    assign lsu_full = (lsu_count == 4'd8);

    wire [31:0] lsu_eff_addr   = lsu_op1_val[lsu_head] + lsu_imm[lsu_head];
    wire [ 1:0] lsu_byte_off   = lsu_eff_addr[1:0];
    wire        lsu_misaligned = (lsu_memh[lsu_head] && lsu_byte_off[0]) ||
                                 (lsu_memw[lsu_head] && (lsu_byte_off != 2'b00));

    wire [ 3:0] lsu_mask =
        lsu_memw[lsu_head] ? 4'b1111 :
        lsu_memh[lsu_head] ? (lsu_byte_off[1] ? 4'b1100 : 4'b0011) :
        lsu_memb[lsu_head] ? (
            (lsu_byte_off == 2'b00) ? 4'b0001 :
            (lsu_byte_off == 2'b01) ? 4'b0010 :
            (lsu_byte_off == 2'b10) ? 4'b0100 : 4'b1000
        ) : 4'b0000;

    wire [31:0] lsu_st_data_raw = lsu_op2_val[lsu_head];
    wire [31:0] lsu_wdata_shifted =
        lsu_memw[lsu_head] ? lsu_st_data_raw :
        lsu_memh[lsu_head] ? (lsu_byte_off[1] ? {lsu_st_data_raw[15:0], 16'b0} : {16'b0, lsu_st_data_raw[15:0]}) :
        lsu_memb[lsu_head] ? (
            (lsu_byte_off == 2'b00) ? {24'b0, lsu_st_data_raw[7:0]} :
            (lsu_byte_off == 2'b01) ? {16'b0, lsu_st_data_raw[7:0], 8'b0} :
            (lsu_byte_off == 2'b10) ? {8'b0, lsu_st_data_raw[7:0], 16'b0} :
                                      {lsu_st_data_raw[7:0], 24'b0}
        ) : lsu_st_data_raw;

    wire lsu_ld_ready = lsu_valid[lsu_head] && lsu_is_load[lsu_head] && !lsu_q1_v[lsu_head];
    wire lsu_ld_trap_fire = lsu_ld_ready && lsu_misaligned && !ld_buf_valid;

    wire lsu_st_prep_fire = lsu_valid[lsu_head] && !lsu_is_load[lsu_head] &&
                            !lsu_q1_v[lsu_head] && !lsu_q2_v[lsu_head] && !lsu_st_prep[lsu_head];

    // Cache arbitration: committing store at ROB head always has priority over load request
    wire        dcache_wen   = rob_commit_store_req;
    wire        dcache_ren   = !rob_commit_store_req && !ld_buf_valid && lsu_ld_ready && !lsu_misaligned;
    wire [31:0] dcache_addr  = rob_commit_store_req ? lsu_st_byte_addr : lsu_eff_addr;
    wire [ 3:0] dcache_mask  = rob_commit_store_req ? lsu_st_mask      : lsu_mask;
    wire [31:0] dcache_wdata = lsu_st_wdata;
    wire [31:0] dcache_rdata;

    cache #(
        .EVICT_POLICY (EVICT_POLICY),
        .NUM_WAYS     (NUM_WAYS),
        .CACHE_SIZE   (CACHE_SIZE),
        .BLOCK_SIZE   (BLOCK_SIZE),
        .PREFETCH_EN  (PREFETCH_EN)
    ) dcache_i (
        .i_clk         (i_clk),
        .i_rst         (i_rst),
        .i_ren         (dcache_ren),
        .i_wen         (dcache_wen),
        .i_addr        (dcache_addr),
        .i_mask        (dcache_mask),
        .i_wdata       (dcache_wdata),
        .o_ready       (dcache_ready),
        .o_rdata       (dcache_rdata),
        .o_mem_rd      (o_mem_rd),
        .o_mem_rd_addr (o_mem_rd_addr),
        .i_mem_valid   (i_mem_valid),
        .i_mem_rd_data (i_mem_rd_data),
        .o_mem_wr      (o_mem_wr),
        .o_mem_wr_addr (o_mem_wr_addr),
        .o_mem_wr_data (o_mem_wr_data),
        .o_mem_wr_be   (o_mem_wr_be)
    );

    wire [ 7:0] raw_byte = (lsu_byte_off == 2'b00) ? dcache_rdata[ 7: 0] :
                           (lsu_byte_off == 2'b01) ? dcache_rdata[15: 8] :
                           (lsu_byte_off == 2'b10) ? dcache_rdata[23:16] :
                                                     dcache_rdata[31:24];
    wire [15:0] raw_half = lsu_byte_off[1] ? dcache_rdata[31:16] : dcache_rdata[15:0];
    wire [31:0] lsu_load_ext =
        lsu_memw[lsu_head] ? dcache_rdata :
        lsu_memh[lsu_head] ? (lsu_memu[lsu_head] ? {16'b0, raw_half} : {{16{raw_half[15]}}, raw_half}) :
        lsu_memb[lsu_head] ? (lsu_memu[lsu_head] ? {24'b0, raw_byte} : {{24{raw_byte[7]}}, raw_byte}) :
                             32'd0;

    wire        ld_cdb_req       = ld_buf_valid || (dcache_ren && dcache_ready);
    wire [31:0] ld_cdb_req_val   = ld_buf_valid ? ld_buf_val       : lsu_load_ext;
    wire [31:0] ld_cdb_req_raw   = ld_buf_valid ? ld_buf_raw_rdata : dcache_rdata;
    wire [31:0] ld_cdb_req_waddr = ld_buf_valid ? ld_buf_word_addr : {lsu_eff_addr[31:2], 2'b00};
    wire [ 3:0] ld_cdb_req_mask  = ld_buf_valid ? ld_buf_mask      : lsu_mask;

    wire lsu_st_pop = rob_commit && rob_is_store[rob_head];
    wire lsu_pop    = lsu_ld_trap_fire || o_ld_gnt || lsu_st_pop;
    wire lsu_push   = issue_fire && cls_mem;

    // ========================================================================
    // 9. CDB Arbiter (cdb_arb.v) & Complete Interface
    // ========================================================================
    cdb_arb #(.TAG_W(3)) cdb_arb_i (
        .i_rob_head  (rob_head),
        .i_alu_v     (alu_cdb_v),
        .i_alu_tag   (alu_cdb_tag),
        .i_alu_rd    (alu_cdb_rd),
        .i_alu_val   (alu_cdb_val),
        .i_alu_pc    (alu_cdb_pc),
        .i_fp0_v     (fp0_cdb_v),
        .i_fp0_tag   (fp0_cdb_tag),
        .i_fp0_rd    (fp0_cdb_rd),
        .i_fp0_val   (fp0_cdb_val),
        .i_fp0_pc    (fp0_cdb_pc),
        .i_fp1_v     (fp1_cdb_v),
        .i_fp1_tag   (fp1_cdb_tag),
        .i_fp1_rd    (fp1_cdb_rd),
        .i_fp1_val   (fp1_cdb_val),
        .i_fp1_pc    (fp1_cdb_pc),
        .i_ld_v      (ld_cdb_req),
        .i_ld_tag    (lsu_tag[lsu_head]),
        .i_ld_rd     (lsu_rd[lsu_head]),
        .i_ld_val    (ld_cdb_req_val),
        .i_ld_pc     (lsu_pc[lsu_head]),
        .o_alu_gnt   (o_alu_gnt),
        .o_fp0_gnt   (o_fp0_gnt),
        .o_fp1_gnt   (o_fp1_gnt),
        .o_ld_gnt    (o_ld_gnt),
        .o_cdb_valid (cdb_valid),
        .o_cdb_tag   (cdb_tag),
        .o_cdb_rd    (cdb_rd),
        .o_cdb_val   (cdb_val),
        .o_cdb_pc    (cdb_pc)
    );

    assign o_complete_valid = cdb_valid;
    assign o_complete_tag   = cdb_tag;
    assign o_complete_rd    = cdb_rd;
    assign o_complete_value = cdb_val;
    assign o_complete_pc    = cdb_pc;

    // ========================================================================
    // 10. Sequential State Updates (Frontend, ROB, ALU RS, FP RS, LSU, CDB Buffers)
    // ========================================================================

    // 10.1 Frontend (PC, IF/ID, ctrl_inflight, halted_issued)
    always @(posedge i_clk) begin
        if (i_rst) begin
            PC             <= RESET_ADDR;
            IF_ID_PC       <= 32'd0;
            IF_ID_instruct <= 32'h00000013;
            IF_ID_valid    <= 1'b0;
            ctrl_inflight  <= 1'b0;
            halted_issued  <= 1'b0;
        end else begin
            if (redirect_taken) begin
                PC             <= redirect_pc;
                IF_ID_PC       <= 32'd0;
                IF_ID_instruct <= 32'h00000013;
                IF_ID_valid    <= 1'b0;
            end else if (!(issue_stall || halted_issued || trap_wb_stall)) begin
                PC             <= pc_plus_4;
                IF_ID_PC       <= PC;
                IF_ID_instruct <= i_imem_rdata;
                IF_ID_valid    <= 1'b1;
            end

            if (alu_resolve_fire)
                ctrl_inflight <= 1'b0;
            else if (issue_fire && cls_brjmp)
                ctrl_inflight <= 1'b1;

            if (issue_fire && r_halt)
                halted_issued <= 1'b1;
        end
    end

    // 10.2 ROB Head / Tail / Count & Architectural FCSR
    wire [11:0] fp0_csr_addr   = fp_rs_inst[fp0_sel_slot][31:20];
    wire [ 2:0] fp0_csr_funct3 = fp_rs_inst[fp0_sel_slot][14:12];
    wire [ 4:0] fp0_csr_rs1f   = fp_rs_inst[fp0_sel_slot][19:15];
    wire [ 7:0] fp0_csr_src    = fp0_csr_funct3[2] ? {3'd0, fp0_csr_rs1f} :
                                 (fp0_csr_rs1f == 5'd0) ? 8'd0 : fp0_op1_eff[7:0];
    wire [ 7:0] fp0_csr_old    = (fp0_csr_addr == 12'h001) ? {3'd0, arch_fcsr[4:0]} :
                                 (fp0_csr_addr == 12'h002) ? {5'd0, arch_fcsr[7:5]} :
                                                             arch_fcsr;
    wire [ 7:0] fp0_csr_new    = (fp0_csr_funct3[1:0] == 2'b01) ? fp0_csr_src :
                                 (fp0_csr_funct3[1:0] == 2'b10) ? (fp0_csr_old | fp0_csr_src) :
                                                                  (fp0_csr_old & ~fp0_csr_src);
    wire        fp0_csr_frm_we = fp_rs_is_csr[fp0_sel_slot] &&
                                 ((fp0_csr_funct3[1:0] == 2'b01) || (fp0_csr_rs1f != 5'd0)) &&
                                 ((fp0_csr_addr == 12'h002) || (fp0_csr_addr == 12'h003));
    wire [ 2:0] fp0_csr_new_frm = (fp0_csr_addr == 12'h002) ? fp0_csr_new[2:0] : fp0_csr_new[7:5];

    wire [4:0] commit_fflags_next =
        (!rob_valid[rob_head] || !rob_done[rob_head] || rob_trap[rob_head] || !rob_is_fpu[rob_head]) ? arch_fcsr[4:0] :
        rob_is_fpu_csr[rob_head] ? rob_fflags[rob_head] :
                                   (arch_fcsr[4:0] | rob_fflags[rob_head]);
    wire [2:0] commit_frm_next =
        (rob_valid[rob_head] && rob_done[rob_head] && !rob_trap[rob_head] && rob_frm_wen[rob_head]) ?
        rob_frm_new[rob_head] : arch_fcsr[7:5];

    always @(posedge i_clk) begin
        if (i_rst) begin
            rob_head  <= 3'd0;
            rob_tail  <= 3'd0;
            rob_count <= 4'd0;
            arch_fcsr <= 8'd0;
        end else begin
            if (rob_commit)
                rob_head <= rob_head + 3'd1;
            if (issue_fire)
                rob_tail <= rob_tail + 3'd1;
            rob_count <= rob_count + (issue_fire ? 4'd1 : 4'd0) - (rob_commit ? 4'd1 : 4'd0);

            if (rob_commit && !rob_trap[rob_head] && rob_is_fpu[rob_head])
                arch_fcsr <= {commit_frm_next, commit_fflags_next};
        end
    end

    // 10.3 ROB Entries (generated per entry to obey no-procedural-loop rule)
    genvar rg;
    generate
        for (rg = 0; rg < ROB_SIZE; rg = rg + 1) begin : gen_rob_entry
            localparam [2:0] RG_TAG = rg;
            wire alloc_this  = issue_fire && (rob_tail == RG_TAG);
            wire commit_this = rob_commit && (rob_head == RG_TAG);
            wire cdb_this    = cdb_valid  && (cdb_tag  == RG_TAG);
            wire alu_res_this = alu_resolve_fire && (alu_sel_tag == RG_TAG);
            wire fp0_rm_this  = fp0_disp_fire && fp0_rm_trap && (fp_rs_tag[fp0_sel_slot] == RG_TAG);
            wire fp1_rm_this  = fp1_disp_fire && fp1_rm_trap && (fp_rs_tag[fp1_sel_slot] == RG_TAG);
            wire ld_trap_this = lsu_ld_trap_fire && (lsu_tag[lsu_head] == RG_TAG);
            wire st_prep_this = lsu_st_prep_fire && (lsu_tag[lsu_head] == RG_TAG);

            always @(posedge i_clk) begin
                if (i_rst) begin
                    rob_valid[rg]       <= 1'b0;
                    rob_done[rg]        <= 1'b0;
                    rob_trap[rg]        <= 1'b0;
                    rob_halt[rg]        <= 1'b0;
                    rob_legal[rg]       <= 1'b0;
                    rob_pc[rg]          <= 32'd0;
                    rob_next_pc[rg]     <= 32'd0;
                    rob_inst[rg]        <= 32'h00000013;
                    rob_rd[rg]          <= 5'd0;
                    rob_frd_en[rg]      <= 1'b0;
                    rob_int_wen[rg]     <= 1'b0;
                    rob_value[rg]       <= 32'd0;
                    rob_rs1[rg]         <= 5'd0;
                    rob_rs2[rg]         <= 5'd0;
                    rob_rs3[rg]         <= 5'd0;
                    rob_rs1_is_read[rg] <= 1'b0;
                    rob_rs2_is_read[rg] <= 1'b0;
                    rob_frs1_en[rg]     <= 1'b0;
                    rob_frs2_en[rg]     <= 1'b0;
                    rob_frs3_en[rg]     <= 1'b0;
                    rob_op1_val[rg]     <= 32'd0;
                    rob_op1_qv[rg]      <= 1'b0;
                    rob_op1_qt[rg]      <= 3'd0;
                    rob_op2_val[rg]     <= 32'd0;
                    rob_op2_qv[rg]      <= 1'b0;
                    rob_op2_qt[rg]      <= 3'd0;
                    rob_op3_val[rg]     <= 32'd0;
                    rob_op3_qv[rg]      <= 1'b0;
                    rob_op3_qt[rg]      <= 3'd0;
                    rob_is_load[rg]     <= 1'b0;
                    rob_is_store[rg]    <= 1'b0;
                    rob_is_fpu[rg]      <= 1'b0;
                    rob_is_fpu_csr[rg]  <= 1'b0;
                    rob_fflags[rg]      <= 5'd0;
                    rob_frm_new[rg]     <= 3'd0;
                    rob_frm_wen[rg]     <= 1'b0;
                    rob_dmem_addr[rg]   <= 32'd0;
                    rob_dmem_mask[rg]   <= 4'd0;
                    rob_dmem_ren[rg]    <= 1'b0;
                    rob_dmem_wen[rg]    <= 1'b0;
                    rob_dmem_rdata[rg]  <= 32'd0;
                    rob_dmem_wdata[rg]  <= 32'd0;
                end else if (alloc_this) begin
                    rob_valid[rg]       <= 1'b1;
                    rob_done[rg]        <= cls_illegal_or_halt;
                    rob_trap[rg]        <= !r_legal;
                    rob_halt[rg]        <= r_halt;
                    rob_legal[rg]       <= r_legal;
                    rob_pc[rg]          <= IF_ID_PC;
                    rob_next_pc[rg]     <= IF_ID_PC + 32'd4;
                    rob_inst[rg]        <= IF_ID_instruct;
                    rob_rd[rg]          <= r_rd;
                    rob_frd_en[rg]      <= r_frd_en;
                    rob_int_wen[rg]     <= r_legal && !r_frd_en && (r_rd != 5'd0);
                    rob_value[rg]       <= 32'd0;
                    rob_rs1[rg]         <= r_rs1;
                    rob_rs2[rg]         <= r_rs2;
                    rob_rs3[rg]         <= r_rs3;
                    rob_rs1_is_read[rg] <= rs1_is_read;
                    rob_rs2_is_read[rg] <= rs2_is_read;
                    rob_frs1_en[rg]     <= r_frs1_en;
                    rob_frs2_en[rg]     <= r_frs2_en;
                    rob_frs3_en[rg]     <= r_frs3_en;
                    rob_op1_val[rg]     <= iss_op1_val;
                    rob_op1_qv[rg]      <= iss_op1_qv;
                    rob_op1_qt[rg]      <= iss_op1_qt;
                    rob_op2_val[rg]     <= iss_op2_val;
                    rob_op2_qv[rg]      <= iss_op2_qv;
                    rob_op2_qt[rg]      <= iss_op2_qt;
                    rob_op3_val[rg]     <= iss_op3_val;
                    rob_op3_qv[rg]      <= iss_op3_qv;
                    rob_op3_qt[rg]      <= iss_op3_qt;
                    rob_is_load[rg]     <= r_legal && r_dmem_ren;
                    rob_is_store[rg]    <= r_legal && r_dmem_wen;
                    rob_is_fpu[rg]      <= r_legal && r_fpu;
                    rob_is_fpu_csr[rg]  <= r_legal && r_fpu && (inst_opcode == 7'b1110011);
                    rob_fflags[rg]      <= 5'd0;
                    rob_frm_new[rg]     <= 3'd0;
                    rob_frm_wen[rg]     <= 1'b0;
                    rob_dmem_addr[rg]   <= 32'd0;
                    rob_dmem_mask[rg]   <= 4'd0;
                    rob_dmem_ren[rg]    <= 1'b0;
                    rob_dmem_wen[rg]    <= 1'b0;
                    rob_dmem_rdata[rg]  <= 32'd0;
                    rob_dmem_wdata[rg]  <= 32'd0;
                end else if (commit_this) begin
                    rob_valid[rg]      <= 1'b0;
                    rob_done[rg]       <= 1'b0;
                    rob_is_fpu_csr[rg] <= 1'b0;
                end else if (rob_valid[rg]) begin
                    // Operand snooping on CDB & trap-restore wakeup
                    if (cdb_valid && rob_op1_qv[rg] && (rob_op1_qt[rg] == cdb_tag)) begin
                        rob_op1_val[rg] <= cdb_val;
                        rob_op1_qv[rg]  <= 1'b0;
                    end else if (trap_wb_fire && rob_op1_qv[rg] && (rob_op1_qt[rg] == rob_head)) begin
                        rob_op1_val[rg] <= trap_wb_val;
                        rob_op1_qv[rg]  <= 1'b0;
                    end

                    if (cdb_valid && rob_op2_qv[rg] && (rob_op2_qt[rg] == cdb_tag)) begin
                        rob_op2_val[rg] <= cdb_val;
                        rob_op2_qv[rg]  <= 1'b0;
                    end else if (trap_wb_fire && rob_op2_qv[rg] && (rob_op2_qt[rg] == rob_head)) begin
                        rob_op2_val[rg] <= trap_wb_val;
                        rob_op2_qv[rg]  <= 1'b0;
                    end

                    if (cdb_valid && rob_op3_qv[rg] && (rob_op3_qt[rg] == cdb_tag)) begin
                        rob_op3_val[rg] <= cdb_val;
                        rob_op3_qv[rg]  <= 1'b0;
                    end else if (trap_wb_fire && rob_op3_qv[rg] && (rob_op3_qt[rg] == rob_head)) begin
                        rob_op3_val[rg] <= trap_wb_val;
                        rob_op3_qv[rg]  <= 1'b0;
                    end

                    // CDB completion
                    if (cdb_this) begin
                        rob_done[rg]  <= 1'b1;
                        rob_value[rg] <= cdb_val;
                        if (o_fp0_gnt)
                            rob_fflags[rg] <= fp0_cdb_fflags;
                        else if (o_fp1_gnt)
                            rob_fflags[rg] <= fp1_cdb_fflags;
                        if (o_ld_gnt) begin
                            rob_dmem_addr[rg]  <= ld_cdb_req_waddr;
                            rob_dmem_mask[rg]  <= ld_cdb_req_mask;
                            rob_dmem_ren[rg]   <= 1'b1;
                            rob_dmem_rdata[rg] <= ld_cdb_req_raw;
                        end
                    end

                    // Branch/jump resolve (sets next_pc; traps if target misaligned)
                    if (alu_res_this) begin
                        rob_next_pc[rg] <= alu_next_pc;
                        if (alu_br_trap) begin
                            rob_trap[rg] <= 1'b1;
                            rob_done[rg] <= 1'b1;
                        end
                    end

                    // FPU CSR metadata capture at dispatch
                    if (fp0_disp_fire && !fp0_rm_trap && (fp_rs_tag[fp0_sel_slot] == RG_TAG) &&
                        fp_rs_is_csr[fp0_sel_slot]) begin
                        rob_frm_new[rg] <= fp0_csr_new_frm;
                        rob_frm_wen[rg] <= fp0_csr_frm_we;
                    end

                    // Non-CDB traps and store preparation
                    if (fp0_rm_this || fp1_rm_this || ld_trap_this) begin
                        rob_trap[rg] <= 1'b1;
                        rob_done[rg] <= 1'b1;
                    end

                    if (st_prep_this) begin
                        rob_done[rg]       <= 1'b1;
                        rob_trap[rg]       <= rob_trap[rg] | lsu_misaligned;
                        rob_dmem_addr[rg]  <= {lsu_eff_addr[31:2], 2'b00};
                        rob_dmem_mask[rg]  <= lsu_misaligned ? 4'd0 : lsu_mask;
                        rob_dmem_wen[rg]   <= !lsu_misaligned;
                        rob_dmem_wdata[rg] <= lsu_wdata_shifted;
                    end
                end
            end
        end
    endgenerate

    // 10.4 ALU RS Entries & ALU CDB Buffer
    always @(posedge i_clk) begin
        if (i_rst) begin
            alu_div_active <= 1'b0;
            alu_div_slot   <= 2'd0;
            alu_cdb_v      <= 1'b0;
            alu_cdb_tag    <= 3'd0;
            alu_cdb_rd     <= 5'd0;
            alu_cdb_val    <= 32'd0;
            alu_cdb_pc     <= 32'd0;
        end else begin
            if (alu_sel_valid && alu_sel_is_div && alu_out_busy) begin
                alu_div_active <= 1'b1;
                alu_div_slot   <= alu_sel_slot;
            end else if (alu_disp_fire) begin
                alu_div_active <= 1'b0;
            end

            if (alu_disp_fire && !alu_br_trap) begin
                alu_cdb_v   <= 1'b1;
                alu_cdb_tag <= alu_sel_tag;
                alu_cdb_rd  <= alu_sel_branch ? 5'd0 : alu_sel_rd;
                alu_cdb_val <= alu_cdb_calc_val;
                alu_cdb_pc  <= alu_sel_pc;
            end else if (o_alu_gnt) begin
                alu_cdb_v <= 1'b0;
            end
        end
    end

    genvar as;
    generate
        for (as = 0; as < ALU_RS_SIZE; as = as + 1) begin : gen_alu_rs_entry
            localparam [1:0] AS_IDX = as;
            wire alloc_as = issue_fire && (cls_alu || cls_brjmp) && (alu_rs_free_idx == AS_IDX);
            wire free_as  = alu_disp_fire && (alu_sel_slot == AS_IDX);

            always @(posedge i_clk) begin
                if (i_rst) begin
                    alu_rs_valid[as]    <= 1'b0;
                    alu_rs_tag[as]      <= 3'd0;
                    alu_rs_pc[as]       <= 32'd0;
                    alu_rs_rd[as]       <= 5'd0;
                    alu_rs_imm[as]      <= 32'd0;
                    alu_rs_op1_sel[as]  <= 1'b0;
                    alu_rs_op2_sel[as]  <= 1'b0;
                    alu_rs_opsel[as]    <= 3'd0;
                    alu_rs_sub[as]      <= 1'b0;
                    alu_rs_unsigned[as] <= 1'b0;
                    alu_rs_arith[as]    <= 1'b0;
                    alu_rs_mext[as]     <= 1'b0;
                    alu_rs_branch[as]   <= 1'b0;
                    alu_rs_jump[as]     <= 1'b0;
                    alu_rs_br_eq[as]    <= 1'b0;
                    alu_rs_br_inv[as]   <= 1'b0;
                    alu_rs_rd_sel[as]   <= 5'd0;
                    alu_rs_pc_sel[as]   <= 1'b0;
                    alu_rs_op1_val[as]  <= 32'd0;
                    alu_rs_q1_v[as]     <= 1'b0;
                    alu_rs_q1_t[as]     <= 3'd0;
                    alu_rs_op2_val[as]  <= 32'd0;
                    alu_rs_q2_v[as]     <= 1'b0;
                    alu_rs_q2_t[as]     <= 3'd0;
                end else if (alloc_as) begin
                    alu_rs_valid[as]    <= 1'b1;
                    alu_rs_tag[as]      <= rob_tail;
                    alu_rs_pc[as]       <= IF_ID_PC;
                    alu_rs_rd[as]       <= r_rd;
                    alu_rs_imm[as]      <= r_immediate;
                    alu_rs_op1_sel[as]  <= r_op1_sel;
                    alu_rs_op2_sel[as]  <= r_op2_sel;
                    alu_rs_opsel[as]    <= r_alu_opsel;
                    alu_rs_sub[as]      <= r_alu_sub;
                    alu_rs_unsigned[as] <= r_alu_unsigned;
                    alu_rs_arith[as]    <= r_alu_arith;
                    alu_rs_mext[as]     <= r_mext;
                    alu_rs_branch[as]   <= r_branch;
                    alu_rs_jump[as]     <= r_jump;
                    alu_rs_br_eq[as]    <= r_branch_equal;
                    alu_rs_br_inv[as]   <= r_branch_invert;
                    alu_rs_rd_sel[as]   <= r_rd_sel;
                    alu_rs_pc_sel[as]   <= r_pc_sel;
                    alu_rs_op1_val[as]  <= iss_op1_val;
                    alu_rs_q1_v[as]     <= iss_op1_qv;
                    alu_rs_q1_t[as]     <= iss_op1_qt;
                    alu_rs_op2_val[as]  <= iss_op2_val;
                    alu_rs_q2_v[as]     <= iss_op2_qv;
                    alu_rs_q2_t[as]     <= iss_op2_qt;
                end else if (free_as) begin
                    alu_rs_valid[as] <= 1'b0;
                end else if (alu_rs_valid[as]) begin
                    if (cdb_valid && alu_rs_q1_v[as] && (alu_rs_q1_t[as] == cdb_tag)) begin
                        alu_rs_op1_val[as] <= cdb_val;
                        alu_rs_q1_v[as]    <= 1'b0;
                    end else if (trap_wb_fire && alu_rs_q1_v[as] && (alu_rs_q1_t[as] == rob_head)) begin
                        alu_rs_op1_val[as] <= trap_wb_val;
                        alu_rs_q1_v[as]    <= 1'b0;
                    end

                    if (cdb_valid && alu_rs_q2_v[as] && (alu_rs_q2_t[as] == cdb_tag)) begin
                        alu_rs_op2_val[as] <= cdb_val;
                        alu_rs_q2_v[as]    <= 1'b0;
                    end else if (trap_wb_fire && alu_rs_q2_v[as] && (alu_rs_q2_t[as] == rob_head)) begin
                        alu_rs_op2_val[as] <= trap_wb_val;
                        alu_rs_q2_v[as]    <= 1'b0;
                    end
                end
            end
        end
    endgenerate

    // 10.5 FP RS Entries, FPU Busy Trackers, FPU CDB Buffers, Sync Flags
    wire fcsr_changed_on_commit = rob_commit && !rob_trap[rob_head] && rob_is_fpu[rob_head] &&
                                  ({commit_frm_next, commit_fflags_next} != arch_fcsr);

    always @(posedge i_clk) begin
        if (i_rst) begin
            fp0_busy_slot_v <= 1'b0;
            fp0_busy_slot   <= 2'd0;
            fp1_busy_slot_v <= 1'b0;
            fp1_busy_slot   <= 2'd0;
            fp0_cdb_v       <= 1'b0;
            fp0_cdb_tag     <= 3'd0;
            fp0_cdb_rd      <= 5'd0;
            fp0_cdb_val     <= 32'd0;
            fp0_cdb_pc      <= 32'd0;
            fp0_cdb_fflags  <= 5'd0;
            fp1_cdb_v       <= 1'b0;
            fp1_cdb_tag     <= 3'd0;
            fp1_cdb_rd      <= 5'd0;
            fp1_cdb_val     <= 32'd0;
            fp1_cdb_pc      <= 32'd0;
            fp1_cdb_fflags  <= 5'd0;
            fp0_needs_sync  <= 1'b0;
            fp1_needs_sync  <= 1'b0;
        end else begin
            if (fp0_sel_valid && fp0_busy) begin
                fp0_busy_slot_v <= 1'b1;
                fp0_busy_slot   <= fp0_sel_slot;
            end else if (fp0_disp_fire) begin
                fp0_busy_slot_v <= 1'b0;
            end

            if (fp1_sel_valid && fp1_busy) begin
                fp1_busy_slot_v <= 1'b1;
                fp1_busy_slot   <= fp1_sel_slot;
            end else if (fp1_disp_fire) begin
                fp1_busy_slot_v <= 1'b0;
            end

            if (fp0_disp_fire && !fp0_rm_trap) begin
                fp0_cdb_v      <= 1'b1;
                fp0_cdb_tag    <= fp_rs_tag[fp0_sel_slot];
                fp0_cdb_rd     <= fp_rs_rd[fp0_sel_slot];
                fp0_cdb_val    <= fp0_res_val;
                fp0_cdb_pc     <= fp_rs_pc[fp0_sel_slot];
                fp0_cdb_fflags <= fp0_fflags;
            end else if (o_fp0_gnt) begin
                fp0_cdb_v <= 1'b0;
            end

            if (fp1_disp_fire && !fp1_rm_trap) begin
                fp1_cdb_v      <= 1'b1;
                fp1_cdb_tag    <= fp_rs_tag[fp1_sel_slot];
                fp1_cdb_rd     <= fp_rs_rd[fp1_sel_slot];
                fp1_cdb_val    <= fp1_res_val;
                fp1_cdb_pc     <= fp_rs_pc[fp1_sel_slot];
                fp1_cdb_fflags <= fp1_fflags;
            end else if (o_fp1_gnt) begin
                fp1_cdb_v <= 1'b0;
            end

            if (fcsr_changed_on_commit)
                fp0_needs_sync <= 1'b1;
            else if (fp0_sync_now)
                fp0_needs_sync <= 1'b0;

            if (fcsr_changed_on_commit)
                fp1_needs_sync <= 1'b1;
            else if (fp1_sync_now)
                fp1_needs_sync <= 1'b0;
        end
    end

    genvar fs;
    generate
        for (fs = 0; fs < FP_RS_SIZE; fs = fs + 1) begin : gen_fp_rs_entry
            localparam [1:0] FS_IDX = fs;
            wire alloc_fs = issue_fire && cls_fpu && (fp_rs_free_idx == FS_IDX);
            wire free_fs  = (fp0_disp_fire && (fp0_sel_slot == FS_IDX)) ||
                            (fp1_disp_fire && (fp1_sel_slot == FS_IDX));

            always @(posedge i_clk) begin
                if (i_rst) begin
                    fp_rs_valid[fs]   <= 1'b0;
                    fp_rs_tag[fs]     <= 3'd0;
                    fp_rs_pc[fs]      <= 32'd0;
                    fp_rs_inst[fs]    <= 32'h00000013;
                    fp_rs_rd[fs]      <= 5'd0;
                    fp_rs_frd_en[fs]  <= 1'b0;
                    fp_rs_is_csr[fs]  <= 1'b0;
                    fp_rs_dyn_rm[fs]  <= 1'b0;
                    fp_rs_op1_val[fs] <= 32'd0;
                    fp_rs_q1_v[fs]    <= 1'b0;
                    fp_rs_q1_t[fs]    <= 3'd0;
                    fp_rs_op2_val[fs] <= 32'd0;
                    fp_rs_q2_v[fs]    <= 1'b0;
                    fp_rs_q2_t[fs]    <= 3'd0;
                    fp_rs_op3_val[fs] <= 32'd0;
                    fp_rs_q3_v[fs]    <= 1'b0;
                    fp_rs_q3_t[fs]    <= 3'd0;
                end else if (alloc_fs) begin
                    fp_rs_valid[fs]   <= 1'b1;
                    fp_rs_tag[fs]     <= rob_tail;
                    fp_rs_pc[fs]      <= IF_ID_PC;
                    fp_rs_inst[fs]    <= IF_ID_instruct;
                    fp_rs_rd[fs]      <= r_rd;
                    fp_rs_frd_en[fs]  <= r_frd_en;
                    fp_rs_is_csr[fs]  <= (inst_opcode == 7'b1110011);
                    fp_rs_dyn_rm[fs]  <= (IF_ID_instruct[14:12] == 3'b111);
                    fp_rs_op1_val[fs] <= iss_op1_val;
                    fp_rs_q1_v[fs]    <= iss_op1_qv;
                    fp_rs_q1_t[fs]    <= iss_op1_qt;
                    fp_rs_op2_val[fs] <= iss_op2_val;
                    fp_rs_q2_v[fs]    <= iss_op2_qv;
                    fp_rs_q2_t[fs]    <= iss_op2_qt;
                    fp_rs_op3_val[fs] <= iss_op3_val;
                    fp_rs_q3_v[fs]    <= iss_op3_qv;
                    fp_rs_q3_t[fs]    <= iss_op3_qt;
                end else if (free_fs) begin
                    fp_rs_valid[fs] <= 1'b0;
                end else if (fp_rs_valid[fs]) begin
                    if (cdb_valid && fp_rs_q1_v[fs] && (fp_rs_q1_t[fs] == cdb_tag)) begin
                        fp_rs_op1_val[fs] <= cdb_val;
                        fp_rs_q1_v[fs]    <= 1'b0;
                    end else if (trap_wb_fire && fp_rs_q1_v[fs] && (fp_rs_q1_t[fs] == rob_head)) begin
                        fp_rs_op1_val[fs] <= trap_wb_val;
                        fp_rs_q1_v[fs]    <= 1'b0;
                    end

                    if (cdb_valid && fp_rs_q2_v[fs] && (fp_rs_q2_t[fs] == cdb_tag)) begin
                        fp_rs_op2_val[fs] <= cdb_val;
                        fp_rs_q2_v[fs]    <= 1'b0;
                    end else if (trap_wb_fire && fp_rs_q2_v[fs] && (fp_rs_q2_t[fs] == rob_head)) begin
                        fp_rs_op2_val[fs] <= trap_wb_val;
                        fp_rs_q2_v[fs]    <= 1'b0;
                    end

                    if (cdb_valid && fp_rs_q3_v[fs] && (fp_rs_q3_t[fs] == cdb_tag)) begin
                        fp_rs_op3_val[fs] <= cdb_val;
                        fp_rs_q3_v[fs]    <= 1'b0;
                    end else if (trap_wb_fire && fp_rs_q3_v[fs] && (fp_rs_q3_t[fs] == rob_head)) begin
                        fp_rs_op3_val[fs] <= trap_wb_val;
                        fp_rs_q3_v[fs]    <= 1'b0;
                    end
                end
            end
        end
    endgenerate

    // 10.6 LSU Pointers, Store Commit Latch, Load CDB Hold Buffer, LSU Entries
    always @(posedge i_clk) begin
        if (i_rst) begin
            lsu_head         <= 3'd0;
            lsu_tail         <= 3'd0;
            lsu_count        <= 4'd0;
            lsu_st_byte_addr <= 32'd0;
            lsu_st_mask      <= 4'd0;
            lsu_st_wdata     <= 32'd0;
            ld_buf_valid     <= 1'b0;
            ld_buf_val       <= 32'd0;
            ld_buf_raw_rdata <= 32'd0;
            ld_buf_word_addr <= 32'd0;
            ld_buf_mask      <= 4'd0;
        end else begin
            if (lsu_pop)
                lsu_head <= lsu_head + 3'd1;
            if (lsu_push)
                lsu_tail <= lsu_tail + 3'd1;
            lsu_count <= lsu_count + (lsu_push ? 4'd1 : 4'd0) - (lsu_pop ? 4'd1 : 4'd0);

            if (lsu_st_prep_fire) begin
                lsu_st_byte_addr <= lsu_eff_addr;
                lsu_st_mask      <= lsu_misaligned ? 4'd0 : lsu_mask;
                lsu_st_wdata     <= lsu_wdata_shifted;
            end

            if (o_ld_gnt) begin
                ld_buf_valid <= 1'b0;
            end else if (dcache_ren && dcache_ready) begin
                // Lost CDB arbitration on cache-ready cycle: capture result so cache isn't re-read
                ld_buf_valid     <= 1'b1;
                ld_buf_val       <= lsu_load_ext;
                ld_buf_raw_rdata <= dcache_rdata;
                ld_buf_word_addr <= {lsu_eff_addr[31:2], 2'b00};
                ld_buf_mask      <= lsu_mask;
            end
        end
    end

    genvar ls;
    generate
        for (ls = 0; ls < LSU_SIZE; ls = ls + 1) begin : gen_lsu_entry
            localparam [2:0] LS_IDX = ls;
            wire alloc_ls = lsu_push && (lsu_tail == LS_IDX);
            wire pop_ls   = lsu_pop  && (lsu_head == LS_IDX);
            wire prep_ls  = lsu_st_prep_fire && (lsu_head == LS_IDX);

            always @(posedge i_clk) begin
                if (i_rst) begin
                    lsu_valid[ls]   <= 1'b0;
                    lsu_tag[ls]     <= 3'd0;
                    lsu_pc[ls]      <= 32'd0;
                    lsu_rd[ls]      <= 5'd0;
                    lsu_is_load[ls] <= 1'b0;
                    lsu_imm[ls]     <= 32'd0;
                    lsu_memb[ls]    <= 1'b0;
                    lsu_memh[ls]    <= 1'b0;
                    lsu_memw[ls]    <= 1'b0;
                    lsu_memu[ls]    <= 1'b0;
                    lsu_op1_val[ls] <= 32'd0;
                    lsu_q1_v[ls]    <= 1'b0;
                    lsu_q1_t[ls]    <= 3'd0;
                    lsu_op2_val[ls] <= 32'd0;
                    lsu_q2_v[ls]    <= 1'b0;
                    lsu_q2_t[ls]    <= 3'd0;
                    lsu_st_prep[ls] <= 1'b0;
                end else if (alloc_ls) begin
                    lsu_valid[ls]   <= 1'b1;
                    lsu_tag[ls]     <= rob_tail;
                    lsu_pc[ls]      <= IF_ID_PC;
                    lsu_rd[ls]      <= r_rd;
                    lsu_is_load[ls] <= r_dmem_ren;
                    lsu_imm[ls]     <= r_immediate;
                    lsu_memb[ls]    <= r_dmem_memb;
                    lsu_memh[ls]    <= r_dmem_memh;
                    lsu_memw[ls]    <= r_dmem_memw;
                    lsu_memu[ls]    <= r_dmem_memu;
                    lsu_op1_val[ls] <= iss_op1_val;
                    lsu_q1_v[ls]    <= iss_op1_qv;
                    lsu_q1_t[ls]    <= iss_op1_qt;
                    lsu_op2_val[ls] <= iss_op2_val;
                    lsu_q2_v[ls]    <= iss_op2_qv;
                    lsu_q2_t[ls]    <= iss_op2_qt;
                    lsu_st_prep[ls] <= 1'b0;
                end else if (pop_ls) begin
                    lsu_valid[ls]   <= 1'b0;
                    lsu_st_prep[ls] <= 1'b0;
                end else if (lsu_valid[ls]) begin
                    if (prep_ls)
                        lsu_st_prep[ls] <= 1'b1;

                    if (cdb_valid && lsu_q1_v[ls] && (lsu_q1_t[ls] == cdb_tag)) begin
                        lsu_op1_val[ls] <= cdb_val;
                        lsu_q1_v[ls]    <= 1'b0;
                    end else if (trap_wb_fire && lsu_q1_v[ls] && (lsu_q1_t[ls] == rob_head)) begin
                        lsu_op1_val[ls] <= trap_wb_val;
                        lsu_q1_v[ls]    <= 1'b0;
                    end

                    if (cdb_valid && lsu_q2_v[ls] && (lsu_q2_t[ls] == cdb_tag)) begin
                        lsu_op2_val[ls] <= cdb_val;
                        lsu_q2_v[ls]    <= 1'b0;
                    end else if (trap_wb_fire && lsu_q2_v[ls] && (lsu_q2_t[ls] == rob_head)) begin
                        lsu_op2_val[ls] <= trap_wb_val;
                        lsu_q2_v[ls]    <= 1'b0;
                    end
                end
            end
        end
    endgenerate

    // ========================================================================
    // 11. Retire Interface (Always from ROB Head in Program Order)
    // ========================================================================
    assign o_retire_valid      = rob_commit;
    assign o_retire_inst       = rob_inst[rob_head];
    assign o_retire_trap       = rob_trap[rob_head];
    assign o_retire_halt       = rob_halt[rob_head];

    assign o_retire_rs1_raddr  = (rob_commit && rob_rs1_is_read[rob_head] && (!rob_trap[rob_head] || rob_legal[rob_head])) ? rob_rs1[rob_head] : 5'd0;
    assign o_retire_rs1_rdata  = (rob_commit && rob_rs1_is_read[rob_head]) ? rob_op1_val[rob_head] : 32'd0;

    assign o_retire_rs2_raddr  = (rob_commit && rob_rs2_is_read[rob_head] && (!rob_trap[rob_head] || rob_legal[rob_head])) ? rob_rs2[rob_head] : 5'd0;
    assign o_retire_rs2_rdata  = (rob_commit && rob_rs2_is_read[rob_head]) ? rob_op2_val[rob_head] : 32'd0;

    assign o_retire_rd_waddr   = actual_rd_waddr;
    assign o_retire_rd_wdata   = actual_rd_wdata;

    assign o_retire_pc         = rob_pc[rob_head];
    assign o_retire_next_pc    = rob_trap[rob_head] ? (rob_pc[rob_head] + 32'd4) : rob_next_pc[rob_head];

    assign o_retire_dmem_addr  = rob_dmem_addr[rob_head];
    assign o_retire_dmem_mask  = rob_dmem_mask[rob_head];
    assign o_retire_dmem_ren   = rob_dmem_ren[rob_head];
    assign o_retire_dmem_wen   = rob_dmem_wen[rob_head];
    assign o_retire_dmem_rdata = rob_dmem_rdata[rob_head];
    assign o_retire_dmem_wdata = rob_dmem_wdata[rob_head];

    assign o_retire_frs1_raddr = (rob_commit && rob_frs1_en[rob_head]) ? rob_rs1[rob_head] : 5'd0;
    assign o_retire_frs1_rdata = (rob_commit && rob_frs1_en[rob_head]) ? rob_op1_val[rob_head] : 32'd0;

    assign o_retire_frs2_raddr = (rob_commit && rob_frs2_en[rob_head]) ? rob_rs2[rob_head] : 5'd0;
    assign o_retire_frs2_rdata = (rob_commit && rob_frs2_en[rob_head]) ? rob_op2_val[rob_head] : 32'd0;

    assign o_retire_frs3_raddr = (rob_commit && rob_frs3_en[rob_head]) ? rob_rs3[rob_head] : 5'd0;
    assign o_retire_frs3_rdata = (rob_commit && rob_frs3_en[rob_head]) ? rob_op3_val[rob_head] : 32'd0;

    assign o_retire_frd_wen    = actual_frd_wen;
    assign o_retire_frd_waddr  = actual_frd_wen ? rob_rd[rob_head] : 5'd0;
    assign o_retire_frd_wdata  = actual_frd_wdata;
    assign o_retire_fflags     = commit_fflags_next;

endmodule

`default_nettype wire
