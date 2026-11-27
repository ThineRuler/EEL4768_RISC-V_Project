`default_nettype none

// Tomasulo-style Speculative Out-of-Order RV32IMF CPU (Phase 8 / Optimized Phase 7)
// - Dynamic Branch Prediction: 64-entry BTB + 128-entry Gshare 2-bit PHT + 8-entry RAS
//   + Decode fallback for cold JAL / JALR / backward branches
// - Speculative OoO execution across unresolved branches with 1-cycle mispredict flush
//   and 1-cycle ROB-based speculative INT/FP RAT recovery
// - 8-entry ROB, 8-entry INT ALU RS, 8-entry FP RS, 8-entry OoO-Load LSU
// - Dual INT ALUs (alu_0 + alu_1) + Dedicated Radix-16+CLZ DIV unit (div_unit)
// - Dual fast FPUs (fpu.v with direct i_arch_fcsr + Radix-16 fpu_div_sqrt)
// - Non-blocking 4-MSHR Data Cache (cache.v) with OoO Load execution & Store-to-Load forwarding
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
    localparam ALU_RS_SIZE = 8;
    localparam FP_RS_SIZE  = 8;
    localparam LSU_SIZE    = 8;
    localparam TAG_W       = 3;

    localparam OFFSET_BITS = $clog2(BLOCK_SIZE);
    localparam WORD_BITS   = (OFFSET_BITS > 2) ? (OFFSET_BITS - 2) : 1;

    localparam [1:0] LD_WAIT = 2'd0;
    localparam [1:0] LD_MISS = 2'd1;
    localparam [1:0] LD_CDB  = 2'd2;
    localparam [1:0] LD_DONE = 2'd3;

    // ========================================================================
    // 1. Frontend: PC, BTB (64) + Gshare PHT (128) + RAS (8), IF/ID
    // ========================================================================
    reg  [31:0] PC;
    reg  [31:0] IF_ID_PC;
    reg  [31:0] IF_ID_instruct;
    reg         IF_ID_valid;
    reg         IF_ID_pred_taken;
    reg  [31:0] IF_ID_pred_pc;
    reg         IF_ID_btb_hit;
    reg  [ 6:0] IF_ID_ghr;
    reg         halted_issued;

    // Branch Target Buffer (64 entries, direct-mapped by PC[7:2])
    reg         btb_valid  [0:63];
    reg  [23:0] btb_tag    [0:63];
    reg  [31:0] btb_target [0:63];
    reg         btb_uncond [0:63];
    reg         btb_is_ret [0:63];

    // Gshare 2-bit Saturating Counter Table (128 entries) + 7-bit GHR
    reg  [ 1:0] pht [0:127];
    reg  [ 6:0] ghr;

    // Return Address Stack (8 entries)
    reg  [31:0] ras_stack [0:7];
    reg  [ 2:0] ras_sp;

    wire [31:0] pc_plus_4       = PC + 32'd4;
    wire [ 5:0] fetch_btb_idx   = PC[7:2];
    wire        fetch_btb_hit   = btb_valid[fetch_btb_idx] && (btb_tag[fetch_btb_idx] == PC[31:8]);
    wire [ 6:0] fetch_pht_idx   = PC[8:2] ^ ghr;
    wire        fetch_pht_taken = pht[fetch_pht_idx][1];
    wire        fetch_pred_taken = fetch_btb_hit && (btb_uncond[fetch_btb_idx] || fetch_pht_taken);
    wire [ 2:0] fetch_ras_top   = ras_sp - 3'd1;
    wire [31:0] fetch_pred_target = btb_is_ret[fetch_btb_idx] ? ras_stack[fetch_ras_top] : btb_target[fetch_btb_idx];
    wire [31:0] fetch_next_pc   = fetch_pred_taken ? fetch_pred_target : pc_plus_4;

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
        (inst_opcode == 7'b0110011) |
        (inst_opcode == 7'b0010011) |
        (inst_opcode == 7'b0000011) |
        (inst_opcode == 7'b0100011) |
        (inst_opcode == 7'b1100011) |
        (inst_opcode == 7'b1100111) |
        (inst_opcode == 7'b0000111) |
        (inst_opcode == 7'b0100111) |
        (r_fpu && !r_frs1_en && !((inst_opcode == 7'b1110011) && IF_ID_instruct[14]))
    );

    wire rs2_is_read = r_legal & (
        (inst_opcode == 7'b0110011) |
        (inst_opcode == 7'b0100011) |
        (inst_opcode == 7'b1100011)
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

    // Branch-prediction & speculation checkpoints per ROB entry
    reg  [31:0] rob_pred_pc     [0:ROB_SIZE-1];
    reg  [ 6:0] rob_ghr_ckpt    [0:ROB_SIZE-1];
    reg  [ 2:0] rob_ras_sp_ckpt [0:ROB_SIZE-1];
    reg         rob_is_call     [0:ROB_SIZE-1];
    reg         rob_is_ret      [0:ROB_SIZE-1];

    wire rob_full  = (rob_count == 4'd8);
    wire rob_empty = (rob_count == 4'd0);

    reg  [ 7:0] arch_fcsr;

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
    wire        mispredict_fire;

    wire rob_commit_store_req = rob_valid[rob_head] && rob_done[rob_head] &&
                                rob_is_store[rob_head] && !rob_trap[rob_head];
    wire rob_commit = rob_valid[rob_head] && rob_done[rob_head] &&
                      (!rob_is_store[rob_head] || rob_trap[rob_head] || dcache_ready);

    wire trap_wb_needed = rob_valid[rob_head] && rob_done[rob_head] &&
                          rob_trap[rob_head] && (rob_int_wen[rob_head] || rob_frd_en[rob_head]);
    wire trap_wb_stall  = trap_wb_needed;
    wire trap_wb_fire   = rob_commit && trap_wb_needed;

    wire issue_stall = IF_ID_valid && (
        halted_issued ||
        trap_wb_stall ||
        rob_full ||
        ((cls_alu || cls_brjmp) && alu_rs_full) ||
        (cls_fpu && fp_rs_full) ||
        (cls_mem && lsu_full)
    );
    wire issue_fire = IF_ID_valid && !issue_stall && !mispredict_fire;

    // ========================================================================
    // 4. Register File & Speculative Register Alias Tables (1-cycle ROB recovery)
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

    reg  [31:0] spec_int_busy;
    reg  [ 2:0] spec_int_tag [0:31];
    reg  [31:0] spec_fp_busy;
    reg  [ 2:0] spec_fp_tag  [0:31];

    wire        rat_int_busy1 = (r_rs1 == 5'd0) ? 1'b0 : spec_int_busy[r_rs1];
    wire [ 2:0] rat_int_tag1  = (r_rs1 == 5'd0) ? 3'd0 : spec_int_tag[r_rs1];
    wire        rat_int_busy2 = (r_rs2 == 5'd0) ? 1'b0 : spec_int_busy[r_rs2];
    wire [ 2:0] rat_int_tag2  = (r_rs2 == 5'd0) ? 3'd0 : spec_int_tag[r_rs2];

    wire        rat_fp_busy1  = spec_fp_busy[r_rs1];
    wire [ 2:0] rat_fp_tag1   = spec_fp_tag[r_rs1];
    wire        rat_fp_busy2  = spec_fp_busy[r_rs2];
    wire [ 2:0] rat_fp_tag2   = spec_fp_tag[r_rs2];
    wire        rat_fp_busy3  = spec_fp_busy[r_rs3];
    wire [ 2:0] rat_fp_tag3   = spec_fp_tag[r_rs3];

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

    // Decode-stage fallback predictor (for cold BTB misses on JAL / ready JALR / backward branch)
    wire        dec_is_jal       = r_legal && r_jump && !r_pc_sel;
    wire        dec_is_jalr_rdy  = r_legal && r_jump && r_pc_sel && !iss_op1_qv;
    wire        dec_cold_bwd_br  = r_legal && r_branch && !IF_ID_btb_hit && !IF_ID_pred_taken && r_immediate[31];
    wire        dec_override_taken = dec_is_jal || dec_is_jalr_rdy || dec_cold_bwd_br;
    wire [31:0] dec_override_target = dec_is_jalr_rdy ? ((iss_op1_val + r_immediate) & ~32'd1) :
                                                        (IF_ID_PC + r_immediate);
    wire        dec_target_valid = (dec_override_target[1:0] == 2'b00);
    wire        dec_redirect     = issue_fire && dec_override_taken && dec_target_valid &&
                                   (!IF_ID_pred_taken || (IF_ID_pred_pc != dec_override_target));

    wire        iss_pred_taken   = (dec_override_taken && dec_target_valid) ? 1'b1 : IF_ID_pred_taken;
    wire [31:0] iss_pred_pc      = (dec_override_taken && dec_target_valid) ? dec_override_target :
                                   IF_ID_pred_taken                         ? IF_ID_pred_pc :
                                                                              (IF_ID_PC + 32'd4);

    wire iss_is_call = r_legal && r_jump && ((r_rd == 5'd1) || (r_rd == 5'd5));
    wire iss_is_ret  = r_legal && r_jump && r_pc_sel && ((r_rs1 == 5'd1) || (r_rs1 == 5'd5)) && (r_rd != r_rs1);

    // ========================================================================
    // 6. INT ALU Reservation Station (8 entries) + Dual ALUs + Dedicated DIV Unit
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

    assign alu_rs_full =
        alu_rs_valid[0] && alu_rs_valid[1] && alu_rs_valid[2] && alu_rs_valid[3] &&
        alu_rs_valid[4] && alu_rs_valid[5] && alu_rs_valid[6] && alu_rs_valid[7];

    wire [2:0] alu_rs_free_idx =
        !alu_rs_valid[0] ? 3'd0 :
        !alu_rs_valid[1] ? 3'd1 :
        !alu_rs_valid[2] ? 3'd2 :
        !alu_rs_valid[3] ? 3'd3 :
        !alu_rs_valid[4] ? 3'd4 :
        !alu_rs_valid[5] ? 3'd5 :
        !alu_rs_valid[6] ? 3'd6 : 3'd7;

    wire [7:0] alu_rs_ready;
    wire [7:0] alu_rs_is_div;
    wire [7:0] alu_rs_is_brj;
    wire [2:0] alu_rs_age [0:7];
    genvar ak, aj;
    generate
        for (ak = 0; ak < 8; ak = ak + 1) begin : gen_alu_rs_rdy
            wire q1_rdy = !alu_rs_q1_v[ak] || (cdb_valid && (cdb_tag == alu_rs_q1_t[ak]));
            wire q2_rdy = !alu_rs_q2_v[ak] || (cdb_valid && (cdb_tag == alu_rs_q2_t[ak]));
            assign alu_rs_ready[ak]  = alu_rs_valid[ak] && q1_rdy && q2_rdy;
            assign alu_rs_is_div[ak] = alu_rs_mext[ak] && alu_rs_opsel[ak][2];
            assign alu_rs_is_brj[ak] = alu_rs_branch[ak] || alu_rs_jump[ak];
            assign alu_rs_age[ak]    = alu_rs_tag[ak] - rob_head;
        end
    endgenerate

    // ALU0: oldest ready non-div (handles branches/jumps + ALU + MUL)
    wire [7:0] alu0_elig;
    wire [7:0] alu0_pick;
    generate
        for (ak = 0; ak < 8; ak = ak + 1) begin : gen_alu0_elig
            assign alu0_elig[ak] = alu_rs_ready[ak] && !alu_rs_is_div[ak];
        end
        for (ak = 0; ak < 8; ak = ak + 1) begin : gen_alu0_pick
            wire [7:0] beaten;
            for (aj = 0; aj < 8; aj = aj + 1) begin : gen_alu0_cmp
                if (aj == ak) begin : gen_s
                    assign beaten[aj] = 1'b0;
                end else begin : gen_o
                    localparam [2:0] AJ_I = aj;
                    localparam [2:0] AK_I = ak;
                    assign beaten[aj] = alu0_elig[aj] &&
                        ((alu_rs_age[aj] < alu_rs_age[ak]) ||
                         ((alu_rs_age[aj] == alu_rs_age[ak]) && (AJ_I < AK_I)));
                end
            end
            assign alu0_pick[ak] = alu0_elig[ak] && !(|beaten);
        end
    endgenerate

    wire       alu0_sel_valid = |alu0_elig;
    wire [2:0] alu0_sel_slot  =
        alu0_pick[0] ? 3'd0 : alu0_pick[1] ? 3'd1 : alu0_pick[2] ? 3'd2 : alu0_pick[3] ? 3'd3 :
        alu0_pick[4] ? 3'd4 : alu0_pick[5] ? 3'd5 : alu0_pick[6] ? 3'd6 : 3'd7;

    // ALU1: oldest ready non-div, non-branch/jump excluding alu0_sel_slot
    wire [7:0] alu1_elig;
    wire [7:0] alu1_pick;
    generate
        for (ak = 0; ak < 8; ak = ak + 1) begin : gen_alu1_elig
            localparam [2:0] AK_I = ak;
            assign alu1_elig[ak] = alu_rs_ready[ak] && !alu_rs_is_div[ak] && !alu_rs_is_brj[ak] &&
                                   !(alu0_sel_valid && (alu0_sel_slot == AK_I));
        end
        for (ak = 0; ak < 8; ak = ak + 1) begin : gen_alu1_pick
            wire [7:0] beaten;
            for (aj = 0; aj < 8; aj = aj + 1) begin : gen_alu1_cmp
                if (aj == ak) begin : gen_s
                    assign beaten[aj] = 1'b0;
                end else begin : gen_o
                    localparam [2:0] AJ_I = aj;
                    localparam [2:0] AK_I = ak;
                    assign beaten[aj] = alu1_elig[aj] &&
                        ((alu_rs_age[aj] < alu_rs_age[ak]) ||
                         ((alu_rs_age[aj] == alu_rs_age[ak]) && (AJ_I < AK_I)));
                end
            end
            assign alu1_pick[ak] = alu1_elig[ak] && !(|beaten);
        end
    endgenerate

    wire       alu1_sel_valid = |alu1_elig;
    wire [2:0] alu1_sel_slot  =
        alu1_pick[0] ? 3'd0 : alu1_pick[1] ? 3'd1 : alu1_pick[2] ? 3'd2 : alu1_pick[3] ? 3'd3 :
        alu1_pick[4] ? 3'd4 : alu1_pick[5] ? 3'd5 : alu1_pick[6] ? 3'd6 : 3'd7;

    // DIV unit: oldest ready div (holds active slot until completion)
    reg        div_active;
    reg  [2:0] div_active_slot;
    wire [7:0] div_elig;
    wire [7:0] div_pick;
    generate
        for (ak = 0; ak < 8; ak = ak + 1) begin : gen_div_elig
            assign div_elig[ak] = alu_rs_ready[ak] && alu_rs_is_div[ak];
        end
        for (ak = 0; ak < 8; ak = ak + 1) begin : gen_div_pick
            wire [7:0] beaten;
            for (aj = 0; aj < 8; aj = aj + 1) begin : gen_div_cmp
                if (aj == ak) begin : gen_s
                    assign beaten[aj] = 1'b0;
                end else begin : gen_o
                    localparam [2:0] AJ_I = aj;
                    localparam [2:0] AK_I = ak;
                    assign beaten[aj] = div_elig[aj] &&
                        ((alu_rs_age[aj] < alu_rs_age[ak]) ||
                         ((alu_rs_age[aj] == alu_rs_age[ak]) && (AJ_I < AK_I)));
                end
            end
            assign div_pick[ak] = div_elig[ak] && !(|beaten);
        end
    endgenerate

    wire       div_sel_valid = div_active || (|div_elig);
    wire [2:0] div_sel_slot  = div_active ? div_active_slot : (
        div_pick[0] ? 3'd0 : div_pick[1] ? 3'd1 : div_pick[2] ? 3'd2 : div_pick[3] ? 3'd3 :
        div_pick[4] ? 3'd4 : div_pick[5] ? 3'd5 : div_pick[6] ? 3'd6 : 3'd7);

    // 3 INT CDB Output Buffers -> Age-ordered Mux into cdb_arb_i's i_alu_* port
    reg        alu0_cdb_v, alu1_cdb_v, div_cdb_v;
    reg  [2:0] alu0_cdb_tag, alu1_cdb_tag, div_cdb_tag;
    reg  [4:0] alu0_cdb_rd,  alu1_cdb_rd,  div_cdb_rd;
    reg [31:0] alu0_cdb_val, alu1_cdb_val, div_cdb_val;
    reg [31:0] alu0_cdb_pc,  alu1_cdb_pc,  div_cdb_pc;

    wire [2:0] alu0_cdb_age = alu0_cdb_tag - rob_head;
    wire [2:0] alu1_cdb_age = alu1_cdb_tag - rob_head;
    wire [2:0] div_cdb_age  = div_cdb_tag  - rob_head;

    wire alu_mux_pick_0 = alu0_cdb_v &&
                          (!alu1_cdb_v || (alu0_cdb_age <= alu1_cdb_age)) &&
                          (!div_cdb_v  || (alu0_cdb_age <= div_cdb_age));
    wire alu_mux_pick_1 = !alu_mux_pick_0 && alu1_cdb_v &&
                          (!div_cdb_v  || (alu1_cdb_age <= div_cdb_age));
    wire alu_mux_pick_div = !alu_mux_pick_0 && !alu_mux_pick_1 && div_cdb_v;

    wire        alu_mux_v   = alu0_cdb_v | alu1_cdb_v | div_cdb_v;
    wire [ 2:0] alu_mux_tag = alu_mux_pick_0 ? alu0_cdb_tag : alu_mux_pick_1 ? alu1_cdb_tag : div_cdb_tag;
    wire [ 4:0] alu_mux_rd  = alu_mux_pick_0 ? alu0_cdb_rd  : alu_mux_pick_1 ? alu1_cdb_rd  : div_cdb_rd;
    wire [31:0] alu_mux_val = alu_mux_pick_0 ? alu0_cdb_val : alu_mux_pick_1 ? alu1_cdb_val : div_cdb_val;
    wire [31:0] alu_mux_pc  = alu_mux_pick_0 ? alu0_cdb_pc  : alu_mux_pick_1 ? alu1_cdb_pc  : div_cdb_pc;

    wire alu0_cdb_gnt = o_alu_gnt && alu_mux_pick_0;
    wire alu1_cdb_gnt = o_alu_gnt && alu_mux_pick_1;
    wire div_cdb_gnt  = o_alu_gnt && alu_mux_pick_div;

    wire alu0_cdb_can_accept = !alu0_cdb_v || alu0_cdb_gnt;
    wire alu1_cdb_can_accept = !alu1_cdb_v || alu1_cdb_gnt;
    wire div_cdb_can_accept  = !div_cdb_v  || div_cdb_gnt;

    // Effective operands for ALU0, ALU1, DIV
    wire [31:0] alu0_op1_eff = (alu_rs_q1_v[alu0_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q1_t[alu0_sel_slot])) ? cdb_val : alu_rs_op1_val[alu0_sel_slot];
    wire [31:0] alu0_op2_eff = (alu_rs_q2_v[alu0_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q2_t[alu0_sel_slot])) ? cdb_val : alu_rs_op2_val[alu0_sel_slot];
    wire [31:0] alu0_op1     = alu_rs_op1_sel[alu0_sel_slot] ? alu_rs_pc[alu0_sel_slot]  : alu0_op1_eff;
    wire [31:0] alu0_op2     = alu_rs_op2_sel[alu0_sel_slot] ? alu_rs_imm[alu0_sel_slot] : alu0_op2_eff;

    wire [31:0] alu1_op1_eff = (alu_rs_q1_v[alu1_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q1_t[alu1_sel_slot])) ? cdb_val : alu_rs_op1_val[alu1_sel_slot];
    wire [31:0] alu1_op2_eff = (alu_rs_q2_v[alu1_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q2_t[alu1_sel_slot])) ? cdb_val : alu_rs_op2_val[alu1_sel_slot];
    wire [31:0] alu1_op1     = alu_rs_op1_sel[alu1_sel_slot] ? alu_rs_pc[alu1_sel_slot]  : alu1_op1_eff;
    wire [31:0] alu1_op2     = alu_rs_op2_sel[alu1_sel_slot] ? alu_rs_imm[alu1_sel_slot] : alu1_op2_eff;

    wire [31:0] div_op1_eff  = (alu_rs_q1_v[div_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q1_t[div_sel_slot])) ? cdb_val : alu_rs_op1_val[div_sel_slot];
    wire [31:0] div_op2_eff  = (alu_rs_q2_v[div_sel_slot] && cdb_valid && (cdb_tag == alu_rs_q2_t[div_sel_slot])) ? cdb_val : alu_rs_op2_val[div_sel_slot];

    wire [31:0] alu0_out_result, alu1_out_result, div_out_result;
    wire        alu0_out_eq, alu0_out_slt, alu0_out_busy;
    wire        alu1_out_eq, alu1_out_slt, alu1_out_busy;
    wire        div_out_eq,  div_out_slt,  div_out_busy;

    wire alu0_disp_fire = alu0_sel_valid && !alu0_out_busy && alu0_cdb_can_accept;
    wire alu1_disp_fire = alu1_sel_valid && !alu1_out_busy && alu1_cdb_can_accept;
    wire div_disp_fire  = div_sel_valid  && !div_out_busy  && div_cdb_can_accept;

    // Branch / Jump resolution on ALU0 + Misprediction Detection
    wire [31:0] alu0_sel_pc       = alu_rs_pc[alu0_sel_slot];
    wire [31:0] alu0_sel_imm      = alu_rs_imm[alu0_sel_slot];
    wire [ 2:0] alu0_sel_tag      = alu_rs_tag[alu0_sel_slot];
    wire [ 4:0] alu0_sel_rd       = alu_rs_rd[alu0_sel_slot];
    wire        alu0_sel_branch   = alu_rs_branch[alu0_sel_slot];
    wire        alu0_sel_jump     = alu_rs_jump[alu0_sel_slot];
    wire [31:0] alu_br_target     = alu0_sel_pc + alu0_sel_imm;
    wire [31:0] alu_jmp_target    = alu_rs_pc_sel[alu0_sel_slot] ? (alu0_out_result & ~32'd1) : alu_br_target;
    wire        alu_br_comp       = alu_rs_br_eq[alu0_sel_slot] ? alu0_out_eq : alu0_out_slt;
    wire        alu_br_taken      = alu0_sel_branch && (alu_br_comp ^ alu_rs_br_inv[alu0_sel_slot]);
    wire        alu_redirect_raw  = alu0_sel_jump || alu_br_taken;
    wire [31:0] alu_next_pc       = alu0_sel_jump ? alu_jmp_target :
                                    alu_br_taken  ? alu_br_target :
                                                    (alu0_sel_pc + 32'd4);
    wire        alu_br_trap       = alu_redirect_raw && (alu_next_pc[1:0] != 2'b00);
    wire        alu_resolve_fire  = alu0_disp_fire && (alu0_sel_branch || alu0_sel_jump);
    wire [31:0] effective_next_pc = alu_br_trap ? (alu0_sel_pc + 32'd4) : alu_next_pc;

    assign mispredict_fire = alu_resolve_fire && (effective_next_pc != rob_pred_pc[alu0_sel_tag]);
    wire [ 2:0] misp_b_age = alu0_sel_tag - rob_head;

    wire [31:0] alu0_cdb_calc_val =
        alu0_sel_branch                 ? 32'd0 :
        alu_rs_rd_sel[alu0_sel_slot][1] ? alu0_sel_imm :
        alu_rs_rd_sel[alu0_sel_slot][2] ? (alu0_sel_pc + 32'd4) :
                                          alu0_out_result;

    wire [31:0] alu1_cdb_calc_val =
        alu_rs_rd_sel[alu1_sel_slot][1] ? alu_rs_imm[alu1_sel_slot] :
        alu_rs_rd_sel[alu1_sel_slot][2] ? (alu_rs_pc[alu1_sel_slot] + 32'd4) :
                                          alu1_out_result;

    wire div_slot_flushed = mispredict_fire && div_sel_valid && ((alu_rs_tag[div_sel_slot] - rob_head) > misp_b_age);

    alu alu_0 (
        .i_clk      (i_clk),
        .i_rst      (i_rst),
        .i_opsel    (alu0_sel_valid ? alu_rs_opsel[alu0_sel_slot]    : 3'b000),
        .i_sub      (alu0_sel_valid ? alu_rs_sub[alu0_sel_slot]      : 1'b0),
        .i_unsigned (alu0_sel_valid ? alu_rs_unsigned[alu0_sel_slot] : 1'b0),
        .i_arith    (alu0_sel_valid ? alu_rs_arith[alu0_sel_slot]    : 1'b0),
        .i_mext     (alu0_sel_valid ? alu_rs_mext[alu0_sel_slot]     : 1'b0),
        .i_advance  (alu0_disp_fire),
        .i_op1      (alu0_op1),
        .i_op2      (alu0_op2),
        .o_result   (alu0_out_result),
        .o_eq       (alu0_out_eq),
        .o_slt      (alu0_out_slt),
        .o_busy     (alu0_out_busy)
    );

    alu alu_1 (
        .i_clk      (i_clk),
        .i_rst      (i_rst),
        .i_opsel    (alu1_sel_valid ? alu_rs_opsel[alu1_sel_slot]    : 3'b000),
        .i_sub      (alu1_sel_valid ? alu_rs_sub[alu1_sel_slot]      : 1'b0),
        .i_unsigned (alu1_sel_valid ? alu_rs_unsigned[alu1_sel_slot] : 1'b0),
        .i_arith    (alu1_sel_valid ? alu_rs_arith[alu1_sel_slot]    : 1'b0),
        .i_mext     (alu1_sel_valid ? alu_rs_mext[alu1_sel_slot]     : 1'b0),
        .i_advance  (alu1_disp_fire),
        .i_op1      (alu1_op1),
        .i_op2      (alu1_op2),
        .o_result   (alu1_out_result),
        .o_eq       (alu1_out_eq),
        .o_slt      (alu1_out_slt),
        .o_busy     (alu1_out_busy)
    );

    alu div_unit (
        .i_clk      (i_clk),
        .i_rst      (i_rst || div_slot_flushed),
        .i_opsel    (div_sel_valid ? alu_rs_opsel[div_sel_slot] : 3'b000),
        .i_sub      (1'b0),
        .i_unsigned (1'b0),
        .i_arith    (1'b0),
        .i_mext     (div_sel_valid),
        .i_advance  (div_disp_fire),
        .i_op1      (div_op1_eff),
        .i_op2      (div_op2_eff),
        .o_result   (div_out_result),
        .o_eq       (div_out_eq),
        .o_slt      (div_out_slt),
        .o_busy     (div_out_busy)
    );

    // ========================================================================
    // 7. FP Reservation Station (8 entries) + Dual FPUs (Direct arch_fcsr)
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

    assign fp_rs_full =
        fp_rs_valid[0] && fp_rs_valid[1] && fp_rs_valid[2] && fp_rs_valid[3] &&
        fp_rs_valid[4] && fp_rs_valid[5] && fp_rs_valid[6] && fp_rs_valid[7];

    wire [2:0] fp_rs_free_idx =
        !fp_rs_valid[0] ? 3'd0 :
        !fp_rs_valid[1] ? 3'd1 :
        !fp_rs_valid[2] ? 3'd2 :
        !fp_rs_valid[3] ? 3'd3 :
        !fp_rs_valid[4] ? 3'd4 :
        !fp_rs_valid[5] ? 3'd5 :
        !fp_rs_valid[6] ? 3'd6 : 3'd7;

    reg        fp0_busy_slot_v, fp1_busy_slot_v;
    reg  [2:0] fp0_busy_slot,   fp1_busy_slot;

    reg        fp0_cdb_v, fp1_cdb_v;
    reg  [2:0] fp0_cdb_tag, fp1_cdb_tag;
    reg  [4:0] fp0_cdb_rd,  fp1_cdb_rd;
    reg [31:0] fp0_cdb_val, fp1_cdb_val;
    reg [31:0] fp0_cdb_pc,  fp1_cdb_pc;
    reg  [4:0] fp0_cdb_fflags, fp1_cdb_fflags;

    wire fp0_cdb_can_accept = !fp0_cdb_v || o_fp0_gnt;
    wire fp1_cdb_can_accept = !fp1_cdb_v || o_fp1_gnt;

    wire [7:0] fp_rs_ready;
    wire [7:0] fp_rs_older_csr;
    wire [2:0] fp_rs_age [0:7];
    genvar fk, fj, rj;
    generate
        for (fk = 0; fk < 8; fk = fk + 1) begin : gen_fp_rs_rdy
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
            assign fp_rs_ready[fk]     = fp_rs_valid[fk] && q1_rdy && q2_rdy && q3_rdy &&
                                         (!(fp_rs_dyn_rm[fk] || fp_rs_is_csr[fk]) || !fp_rs_older_csr[fk]);
        end
    endgenerate

    wire [7:0] fp0_elig;
    wire [7:0] fp1_elig;
    generate
        for (fk = 0; fk < 8; fk = fk + 1) begin : gen_fp_elig
            localparam [2:0] FK_IDX = fk;
            wire not_on_fp1 = !(fp1_busy_slot_v && (fp1_busy_slot == FK_IDX));
            wire not_on_fp0 = !(fp0_busy_slot_v && (fp0_busy_slot == FK_IDX));
            wire csr_ok0    = !fp_rs_is_csr[fk] || (fp_rs_tag[fk] == rob_head);
            assign fp0_elig[fk] = fp_rs_ready[fk] && not_on_fp1 && csr_ok0;
            assign fp1_elig[fk] = fp_rs_ready[fk] && not_on_fp0 && !fp_rs_is_csr[fk];
        end
    endgenerate

    wire [7:0] fp0_pick;
    generate
        for (fk = 0; fk < 8; fk = fk + 1) begin : gen_fp0_pick
            wire [7:0] beaten;
            for (fj = 0; fj < 8; fj = fj + 1) begin : gen_fp0_cmp
                if (fj == fk) begin : gen_s
                    assign beaten[fj] = 1'b0;
                end else begin : gen_o
                    localparam [2:0] FJ_I = fj;
                    localparam [2:0] FK_I = fk;
                    assign beaten[fj] = fp0_elig[fj] &&
                        ((fp_rs_age[fj] < fp_rs_age[fk]) ||
                         ((fp_rs_age[fj] == fp_rs_age[fk]) && (FJ_I < FK_I)));
                end
            end
            assign fp0_pick[fk] = fp0_elig[fk] && !(|beaten);
        end
    endgenerate

    wire       fp0_sel_valid = fp0_busy_slot_v || (|fp0_elig);
    wire [2:0] fp0_sel_slot  = fp0_busy_slot_v ? fp0_busy_slot : (
        fp0_pick[0] ? 3'd0 : fp0_pick[1] ? 3'd1 : fp0_pick[2] ? 3'd2 : fp0_pick[3] ? 3'd3 :
        fp0_pick[4] ? 3'd4 : fp0_pick[5] ? 3'd5 : fp0_pick[6] ? 3'd6 : 3'd7);

    wire [7:0] fp1_elig_excl;
    wire [7:0] fp1_pick;
    generate
        for (fk = 0; fk < 8; fk = fk + 1) begin : gen_fp1_excl
            localparam [2:0] FK_IDX = fk;
            assign fp1_elig_excl[fk] = fp1_elig[fk] && !(fp0_sel_valid && (fp0_sel_slot == FK_IDX));
        end
        for (fk = 0; fk < 8; fk = fk + 1) begin : gen_fp1_pick
            wire [7:0] beaten;
            for (fj = 0; fj < 8; fj = fj + 1) begin : gen_fp1_cmp
                if (fj == fk) begin : gen_s
                    assign beaten[fj] = 1'b0;
                end else begin : gen_o
                    localparam [2:0] FJ_I = fj;
                    localparam [2:0] FK_I = fk;
                    assign beaten[fj] = fp1_elig_excl[fj] &&
                        ((fp_rs_age[fj] < fp_rs_age[fk]) ||
                         ((fp_rs_age[fj] == fp_rs_age[fk]) && (FJ_I < FK_I)));
                end
            end
            assign fp1_pick[fk] = fp1_elig_excl[fk] && !(|beaten);
        end
    endgenerate

    wire       fp1_sel_valid = fp1_busy_slot_v || (|fp1_elig_excl);
    wire [2:0] fp1_sel_slot  = fp1_busy_slot_v ? fp1_busy_slot : (
        fp1_pick[0] ? 3'd0 : fp1_pick[1] ? 3'd1 : fp1_pick[2] ? 3'd2 : fp1_pick[3] ? 3'd3 :
        fp1_pick[4] ? 3'd4 : fp1_pick[5] ? 3'd5 : fp1_pick[6] ? 3'd6 : 3'd7);

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

    wire fp0_slot_flushed = mispredict_fire && fp0_sel_valid && ((fp_rs_tag[fp0_sel_slot] - rob_head) > misp_b_age);
    wire fp1_slot_flushed = mispredict_fire && fp1_sel_valid && ((fp_rs_tag[fp1_sel_slot] - rob_head) > misp_b_age);

    fpu fpu_0 (
        .i_clk        (i_clk),
        .i_rst        (i_rst || fp0_slot_flushed),
        .i_inst       (fp_rs_inst[fp0_sel_slot]),
        .i_valid      (fp0_sel_valid),
        .i_advance    (fp0_disp_fire),
        .i_frs1       (fp0_op1_eff),
        .i_frs2       (fp0_op2_eff),
        .i_frs3       (fp0_op3_eff),
        .i_xrs1       (fp0_op1_eff),
        .o_fp_result  (fp0_fp_res),
        .o_int_result (fp0_int_res),
        .o_fflags     (fp0_fflags),
        .o_busy       (fp0_busy),
        .o_rm_trap    (fp0_rm_trap),
        .i_arch_fcsr  (arch_fcsr)
    );

    fpu fpu_1 (
        .i_clk        (i_clk),
        .i_rst        (i_rst || fp1_slot_flushed),
        .i_inst       (fp_rs_inst[fp1_sel_slot]),
        .i_valid      (fp1_sel_valid),
        .i_advance    (fp1_disp_fire),
        .i_frs1       (fp1_op1_eff),
        .i_frs2       (fp1_op2_eff),
        .i_frs3       (fp1_op3_eff),
        .i_xrs1       (fp1_op1_eff),
        .o_fp_result  (fp1_fp_res),
        .o_int_result (fp1_int_res),
        .o_fflags     (fp1_fflags),
        .o_busy       (fp1_busy),
        .o_rm_trap    (fp1_rm_trap),
        .i_arch_fcsr  (arch_fcsr)
    );

    wire [31:0] fp0_res_val = fp_rs_frd_en[fp0_sel_slot] ? fp0_fp_res : fp0_int_res;
    wire [31:0] fp1_res_val = fp_rs_frd_en[fp1_sel_slot] ? fp1_fp_res : fp1_int_res;

    // ========================================================================
    // 8. Out-of-Order Non-Blocking Load-Store Unit (8 entries) + Non-Blocking Cache
    // ========================================================================
    reg  [ 2:0] lsu_head, lsu_tail;
    reg  [ 3:0] lsu_count;

    reg         lsu_valid       [0:LSU_SIZE-1];
    reg  [ 2:0] lsu_tag         [0:LSU_SIZE-1];
    reg  [31:0] lsu_pc          [0:LSU_SIZE-1];
    reg  [ 4:0] lsu_rd          [0:LSU_SIZE-1];
    reg         lsu_is_load     [0:LSU_SIZE-1];
    reg  [31:0] lsu_imm         [0:LSU_SIZE-1];
    reg         lsu_memb        [0:LSU_SIZE-1];
    reg         lsu_memh        [0:LSU_SIZE-1];
    reg         lsu_memw        [0:LSU_SIZE-1];
    reg         lsu_memu        [0:LSU_SIZE-1];
    reg  [31:0] lsu_op1_val     [0:LSU_SIZE-1];
    reg         lsu_q1_v        [0:LSU_SIZE-1];
    reg  [ 2:0] lsu_q1_t        [0:LSU_SIZE-1];
    reg  [31:0] lsu_op2_val     [0:LSU_SIZE-1];
    reg         lsu_q2_v        [0:LSU_SIZE-1];
    reg  [ 2:0] lsu_q2_t        [0:LSU_SIZE-1];
    reg         lsu_st_prepped  [0:LSU_SIZE-1];
    reg  [31:0] lsu_st_baddr    [0:LSU_SIZE-1];
    reg  [ 3:0] lsu_st_mask     [0:LSU_SIZE-1];
    reg  [31:0] lsu_st_wdata    [0:LSU_SIZE-1];

    reg  [ 1:0] lsu_ld_state    [0:LSU_SIZE-1];
    reg  [31:0] lsu_ld_val      [0:LSU_SIZE-1];
    reg  [31:0] lsu_ld_raw      [0:LSU_SIZE-1];
    reg  [31:0] lsu_ld_waddr    [0:LSU_SIZE-1];
    reg  [ 3:0] lsu_ld_mask     [0:LSU_SIZE-1];
    reg  [31:0] lsu_ld_blk_addr [0:LSU_SIZE-1];
    reg  [ 1:0] lsu_ld_byte_off [0:LSU_SIZE-1];

    assign lsu_full = (lsu_count == 4'd8);

    wire [31:0] lsu_eff_addr   [0:LSU_SIZE-1];
    wire [31:0] lsu_waddr_c    [0:LSU_SIZE-1];
    wire [31:0] lsu_blk_addr_c [0:LSU_SIZE-1];
    wire [ 1:0] lsu_boff_c     [0:LSU_SIZE-1];
    wire        lsu_misal_c    [0:LSU_SIZE-1];
    wire [ 3:0] lsu_mask_c     [0:LSU_SIZE-1];
    wire [31:0] lsu_wdata_c    [0:LSU_SIZE-1];
    wire [ 7:0] lsu_st_prep_fire;
    wire [ 7:0] lsu_ld_trap_fire;
    wire [ 7:0] lsu_ld_fwd_fire;
    wire [31:0] lsu_ld_fwd_raw [0:LSU_SIZE-1];
    wire [ 7:0] lsu_ld_dcache_elig;
    wire [ 2:0] lsu_age        [0:LSU_SIZE-1];

    genvar ls, pj, pk;
    generate
        for (ls = 0; ls < 8; ls = ls + 1) begin : gen_lsu_comb
            localparam [2:0] LS_I = ls;
            assign lsu_age[ls]        = LS_I - lsu_head;
            assign lsu_eff_addr[ls]   = lsu_op1_val[ls] + lsu_imm[ls];
            assign lsu_waddr_c[ls]    = {lsu_eff_addr[ls][31:2], 2'b00};
            assign lsu_blk_addr_c[ls] = {lsu_eff_addr[ls][31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
            assign lsu_boff_c[ls]     = lsu_eff_addr[ls][1:0];
            assign lsu_misal_c[ls]    = (lsu_memh[ls] && lsu_boff_c[ls][0]) ||
                                        (lsu_memw[ls] && (lsu_boff_c[ls] != 2'b00));
            assign lsu_mask_c[ls] =
                lsu_memw[ls] ? 4'b1111 :
                lsu_memh[ls] ? (lsu_boff_c[ls][1] ? 4'b1100 : 4'b0011) :
                lsu_memb[ls] ? (
                    (lsu_boff_c[ls] == 2'b00) ? 4'b0001 :
                    (lsu_boff_c[ls] == 2'b01) ? 4'b0010 :
                    (lsu_boff_c[ls] == 2'b10) ? 4'b0100 : 4'b1000
                ) : 4'b0000;
            assign lsu_wdata_c[ls] =
                lsu_memw[ls] ? lsu_op2_val[ls] :
                lsu_memh[ls] ? (lsu_boff_c[ls][1] ? {lsu_op2_val[ls][15:0], 16'b0} : {16'b0, lsu_op2_val[ls][15:0]}) :
                lsu_memb[ls] ? (
                    (lsu_boff_c[ls] == 2'b00) ? {24'b0, lsu_op2_val[ls][7:0]} :
                    (lsu_boff_c[ls] == 2'b01) ? {16'b0, lsu_op2_val[ls][7:0], 8'b0} :
                    (lsu_boff_c[ls] == 2'b10) ? {8'b0, lsu_op2_val[ls][7:0], 16'b0} :
                                                {lsu_op2_val[ls][7:0], 24'b0}
                ) : lsu_op2_val[ls];

            assign lsu_st_prep_fire[ls] = lsu_valid[ls] && !lsu_is_load[ls] &&
                                          !lsu_st_prepped[ls] && !lsu_q1_v[ls] && !lsu_q2_v[ls];

            wire ld_base_rdy = lsu_valid[ls] && lsu_is_load[ls] &&
                               (lsu_ld_state[ls] == LD_WAIT) && !lsu_q1_v[ls];
            assign lsu_ld_trap_fire[ls] = ld_base_rdy && lsu_misal_c[ls];

            wire [7:0] older_st_addr_unk;
            wire [7:0] older_st_match;
            wire [7:0] older_st_ym;
            for (pj = 0; pj < 8; pj = pj + 1) begin : gen_older_st
                wire is_older_st = lsu_valid[pj] && !lsu_is_load[pj] && (lsu_age[pj] < lsu_age[ls]);
                assign older_st_addr_unk[pj] = is_older_st && lsu_q1_v[pj];
                assign older_st_match[pj]    = is_older_st && !lsu_q1_v[pj] && !lsu_misal_c[pj] &&
                                               (lsu_waddr_c[pj] == lsu_waddr_c[ls]);
            end

            for (pj = 0; pj < 8; pj = pj + 1) begin : gen_ym_st
                wire [7:0] younger_match;
                for (pk = 0; pk < 8; pk = pk + 1) begin : gen_ym_cmp
                    if (pk == pj) begin : gen_s
                        assign younger_match[pk] = 1'b0;
                    end else begin : gen_o
                        assign younger_match[pk] = older_st_match[pk] && (lsu_age[pk] > lsu_age[pj]);
                    end
                end
                assign older_st_ym[pj] = older_st_match[pj] && !(|younger_match);
            end

            wire any_older_st_unk   = |older_st_addr_unk;
            wire any_older_st_match = |older_st_match;

            wire [2:0] ym_slot =
                older_st_ym[0] ? 3'd0 : older_st_ym[1] ? 3'd1 : older_st_ym[2] ? 3'd2 : older_st_ym[3] ? 3'd3 :
                older_st_ym[4] ? 3'd4 : older_st_ym[5] ? 3'd5 : older_st_ym[6] ? 3'd6 : 3'd7;

            wire ym_data_rdy  = !lsu_q2_v[ym_slot];
            wire ym_full_cov  = ((lsu_mask_c[ym_slot] & lsu_mask_c[ls]) == lsu_mask_c[ls]) &&
                                ((lsu_mask_c[ym_slot] == 4'b1111) || (lsu_mask_c[ym_slot] == lsu_mask_c[ls]));

            wire can_fwd_now  = ld_base_rdy && !lsu_misal_c[ls] && !any_older_st_unk &&
                                any_older_st_match && ym_data_rdy && ym_full_cov;
            wire blocked_by_st = any_older_st_unk || (any_older_st_match && !(ym_data_rdy && ym_full_cov));

            assign lsu_ld_fwd_fire[ls]    = can_fwd_now;
            assign lsu_ld_fwd_raw[ls]     = lsu_wdata_c[ym_slot];
            assign lsu_ld_dcache_elig[ls] = ld_base_rdy && !lsu_misal_c[ls] && !blocked_by_st && !any_older_st_match;
        end
    endgenerate

    wire [7:0] ld_dcache_pick;
    generate
        for (ls = 0; ls < 8; ls = ls + 1) begin : gen_ld_dc_pick
            wire [7:0] beaten;
            for (pj = 0; pj < 8; pj = pj + 1) begin : gen_ld_dc_cmp
                if (pj == ls) begin : gen_s
                    assign beaten[pj] = 1'b0;
                end else begin : gen_o
                    localparam [2:0] PJ_I = pj;
                    localparam [2:0] LS_I = ls;
                    assign beaten[pj] = lsu_ld_dcache_elig[pj] &&
                        ((lsu_age[pj] < lsu_age[ls]) || ((lsu_age[pj] == lsu_age[ls]) && (PJ_I < LS_I)));
                end
            end
            assign ld_dcache_pick[ls] = lsu_ld_dcache_elig[ls] && !(|beaten);
        end
    endgenerate

    wire       ld_dcache_pick_valid = |lsu_ld_dcache_elig;
    wire [2:0] ld_dcache_pick_slot  =
        ld_dcache_pick[0] ? 3'd0 : ld_dcache_pick[1] ? 3'd1 : ld_dcache_pick[2] ? 3'd2 : ld_dcache_pick[3] ? 3'd3 :
        ld_dcache_pick[4] ? 3'd4 : ld_dcache_pick[5] ? 3'd5 : ld_dcache_pick[6] ? 3'd6 : 3'd7;

    wire                    dcache_wen           = rob_commit_store_req;
    wire                    dcache_ren           = !rob_commit_store_req && ld_dcache_pick_valid;
    wire [31:0]             dcache_addr          = rob_commit_store_req ? lsu_st_baddr[lsu_head] : lsu_eff_addr[ld_dcache_pick_slot];
    wire [ 3:0]             dcache_mask          = rob_commit_store_req ? lsu_st_mask[lsu_head]  : lsu_mask_c[ld_dcache_pick_slot];
    wire [31:0]             dcache_wdata         = lsu_st_wdata[lsu_head];
    wire [31:0]             dcache_rdata;
    wire                    dcache_miss_accepted;
    wire                    dcache_fill_valid;
    wire [31:0]             dcache_fill_addr;
    wire [BLOCK_SIZE*8-1:0] dcache_fill_data;

    cache #(
        .EVICT_POLICY (EVICT_POLICY),
        .NUM_WAYS     (NUM_WAYS),
        .CACHE_SIZE   (CACHE_SIZE),
        .BLOCK_SIZE   (BLOCK_SIZE),
        .PREFETCH_EN  (PREFETCH_EN)
    ) dcache_i (
        .i_clk           (i_clk),
        .i_rst           (i_rst),
        .i_ren           (dcache_ren),
        .i_wen           (dcache_wen),
        .i_addr          (dcache_addr),
        .i_mask          (dcache_mask),
        .i_wdata         (dcache_wdata),
        .o_ready         (dcache_ready),
        .o_rdata         (dcache_rdata),
        .o_miss_accepted (dcache_miss_accepted),
        .o_fill_valid    (dcache_fill_valid),
        .o_fill_addr     (dcache_fill_addr),
        .o_fill_data     (dcache_fill_data),
        .o_mem_rd        (o_mem_rd),
        .o_mem_rd_addr   (o_mem_rd_addr),
        .i_mem_ready     (i_mem_ready),
        .i_mem_valid     (i_mem_valid),
        .i_mem_rd_data   (i_mem_rd_data),
        .o_mem_wr        (o_mem_wr),
        .o_mem_wr_addr   (o_mem_wr_addr),
        .o_mem_wr_data   (o_mem_wr_data),
        .o_mem_wr_be     (o_mem_wr_be)
    );

    wire dcache_hit_fire  = dcache_ren && dcache_ready;
    wire dcache_miss_fire = dcache_ren && !dcache_ready && dcache_miss_accepted;

    wire [1:0] dc_hit_boff = lsu_boff_c[ld_dcache_pick_slot];
    wire [7:0] dc_hit_byte =
        (dc_hit_boff == 2'b00) ? dcache_rdata[ 7: 0] :
        (dc_hit_boff == 2'b01) ? dcache_rdata[15: 8] :
        (dc_hit_boff == 2'b10) ? dcache_rdata[23:16] : dcache_rdata[31:24];
    wire [15:0] dc_hit_half = dc_hit_boff[1] ? dcache_rdata[31:16] : dcache_rdata[15:0];
    wire [31:0] dc_hit_ext  =
        lsu_memw[ld_dcache_pick_slot] ? dcache_rdata :
        lsu_memh[ld_dcache_pick_slot] ? (lsu_memu[ld_dcache_pick_slot] ? {16'b0, dc_hit_half} : {{16{dc_hit_half[15]}}, dc_hit_half}) :
        lsu_memb[ld_dcache_pick_slot] ? (lsu_memu[ld_dcache_pick_slot] ? {24'b0, dc_hit_byte} : {{24{dc_hit_byte[7]}}, dc_hit_byte}) :
                                        32'd0;

    wire [31:0] lsu_ld_fwd_ext  [0:LSU_SIZE-1];
    wire [31:0] lsu_ld_fill_raw [0:LSU_SIZE-1];
    wire [31:0] lsu_ld_fill_ext [0:LSU_SIZE-1];
    generate
        for (ls = 0; ls < 8; ls = ls + 1) begin : gen_lsu_ext
            wire [31:0] fw_r = lsu_ld_fwd_raw[ls];
            wire [ 7:0] fw_b =
                (lsu_boff_c[ls] == 2'b00) ? fw_r[ 7: 0] :
                (lsu_boff_c[ls] == 2'b01) ? fw_r[15: 8] :
                (lsu_boff_c[ls] == 2'b10) ? fw_r[23:16] : fw_r[31:24];
            wire [15:0] fw_h = lsu_boff_c[ls][1] ? fw_r[31:16] : fw_r[15:0];
            assign lsu_ld_fwd_ext[ls] =
                lsu_memw[ls] ? fw_r :
                lsu_memh[ls] ? (lsu_memu[ls] ? {16'b0, fw_h} : {{16{fw_h[15]}}, fw_h}) :
                lsu_memb[ls] ? (lsu_memu[ls] ? {24'b0, fw_b} : {{24{fw_b[7]}}, fw_b}) : 32'd0;

            wire [WORD_BITS-1:0] fill_widx = (OFFSET_BITS > 2) ? lsu_ld_waddr[ls][OFFSET_BITS-1:2] : {WORD_BITS{1'b0}};
            wire [OFFSET_BITS+3-1:0] fill_bidx = {fill_widx, 5'b00000};
            wire [31:0] fl_r = dcache_fill_data[fill_bidx +: 32];
            wire [ 7:0] fl_b =
                (lsu_ld_byte_off[ls] == 2'b00) ? fl_r[ 7: 0] :
                (lsu_ld_byte_off[ls] == 2'b01) ? fl_r[15: 8] :
                (lsu_ld_byte_off[ls] == 2'b10) ? fl_r[23:16] : fl_r[31:24];
            wire [15:0] fl_h = lsu_ld_byte_off[ls][1] ? fl_r[31:16] : fl_r[15:0];
            assign lsu_ld_fill_raw[ls] = fl_r;
            assign lsu_ld_fill_ext[ls] =
                lsu_memw[ls] ? fl_r :
                lsu_memh[ls] ? (lsu_memu[ls] ? {16'b0, fl_h} : {{16{fl_h[15]}}, fl_h}) :
                lsu_memb[ls] ? (lsu_memu[ls] ? {24'b0, fl_b} : {{24{fl_b[7]}}, fl_b}) : 32'd0;
        end
    endgenerate

    wire [7:0] ld_cdb_elig;
    wire [7:0] ld_cdb_pick;
    generate
        for (ls = 0; ls < 8; ls = ls + 1) begin : gen_ld_cdb_elig
            assign ld_cdb_elig[ls] = lsu_valid[ls] && lsu_is_load[ls] && (lsu_ld_state[ls] == LD_CDB);
        end
        for (ls = 0; ls < 8; ls = ls + 1) begin : gen_ld_cdb_pick
            wire [7:0] beaten;
            for (pj = 0; pj < 8; pj = pj + 1) begin : gen_ld_cdb_cmp
                if (pj == ls) begin : gen_s
                    assign beaten[pj] = 1'b0;
                end else begin : gen_o
                    localparam [2:0] PJ_I = pj;
                    localparam [2:0] LS_I = ls;
                    assign beaten[pj] = ld_cdb_elig[pj] &&
                        ((lsu_age[pj] < lsu_age[ls]) || ((lsu_age[pj] == lsu_age[ls]) && (PJ_I < LS_I)));
                end
            end
            assign ld_cdb_pick[ls] = ld_cdb_elig[ls] && !(|beaten);
        end
    endgenerate

    wire       ld_cdb_req       = |ld_cdb_elig;
    wire [2:0] ld_cdb_pick_slot =
        ld_cdb_pick[0] ? 3'd0 : ld_cdb_pick[1] ? 3'd1 : ld_cdb_pick[2] ? 3'd2 : ld_cdb_pick[3] ? 3'd3 :
        ld_cdb_pick[4] ? 3'd4 : ld_cdb_pick[5] ? 3'd5 : ld_cdb_pick[6] ? 3'd6 : 3'd7;

    wire lsu_st_pop = rob_commit && rob_is_store[rob_head];
    wire lsu_head_ld_pop = lsu_valid[lsu_head] && lsu_is_load[lsu_head] && (
        (lsu_ld_state[lsu_head] == LD_DONE) ||
        (o_ld_gnt && (ld_cdb_pick_slot == lsu_head)) ||
        lsu_ld_trap_fire[lsu_head]
    );
    wire lsu_pop  = lsu_head_ld_pop || lsu_st_pop;
    wire lsu_push = issue_fire && cls_mem;

    // ========================================================================
    // 9. CDB Arbiter (cdb_arb.v) & Complete Interface
    // ========================================================================
    cdb_arb #(.TAG_W(3)) cdb_arb_i (
        .i_rob_head  (rob_head),
        .i_alu_v     (alu_mux_v),
        .i_alu_tag   (alu_mux_tag),
        .i_alu_rd    (alu_mux_rd),
        .i_alu_val   (alu_mux_val),
        .i_alu_pc    (alu_mux_pc),
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
        .i_ld_tag    (lsu_tag[ld_cdb_pick_slot]),
        .i_ld_rd     (lsu_rd[ld_cdb_pick_slot]),
        .i_ld_val    (lsu_ld_val[ld_cdb_pick_slot]),
        .i_ld_pc     (lsu_pc[ld_cdb_pick_slot]),
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
    // 10. Sequential State Updates & 1-Cycle Speculative RAT Recovery
    // ========================================================================

    // 10.1 Frontend, BTB, PHT, GHR, RAS
    integer bi, pi_idx;
    wire [5:0] upd_btb_idx = alu0_sel_pc[7:2];
    wire [6:0] upd_pht_idx = alu0_sel_pc[8:2] ^ rob_ghr_ckpt[alu0_sel_tag];
    wire [1:0] upd_pht_old = pht[upd_pht_idx];
    wire [1:0] upd_pht_new = alu_br_taken ?
        ((upd_pht_old == 2'b11) ? 2'b11 : (upd_pht_old + 2'd1)) :
        ((upd_pht_old == 2'b00) ? 2'b00 : (upd_pht_old - 2'd1));

    always @(posedge i_clk) begin
        if (i_rst) begin
            PC               <= RESET_ADDR;
            IF_ID_PC         <= 32'd0;
            IF_ID_instruct   <= 32'h00000013;
            IF_ID_valid      <= 1'b0;
            IF_ID_pred_taken <= 1'b0;
            IF_ID_pred_pc    <= 32'd0;
            IF_ID_btb_hit    <= 1'b0;
            IF_ID_ghr        <= 7'd0;
            halted_issued    <= 1'b0;
            ghr              <= 7'd0;
            ras_sp           <= 3'd0;

            for (bi = 0; bi < 64; bi = bi + 1) begin
                btb_valid[bi]  <= 1'b0;
                btb_tag[bi]    <= 24'd0;
                btb_target[bi] <= 32'd0;
                btb_uncond[bi] <= 1'b0;
                btb_is_ret[bi] <= 1'b0;
            end
            for (pi_idx = 0; pi_idx < 128; pi_idx = pi_idx + 1) begin
                pht[pi_idx] <= 2'b01; // weakly not taken
            end
            for (bi = 0; bi < 8; bi = bi + 1) begin
                ras_stack[bi] <= RESET_ADDR;
            end
        end else begin
            // Frontend PC & IF/ID update
            if (mispredict_fire) begin
                PC               <= effective_next_pc;
                IF_ID_PC         <= 32'd0;
                IF_ID_instruct   <= 32'h00000013;
                IF_ID_valid      <= 1'b0;
                IF_ID_pred_taken <= 1'b0;
                IF_ID_pred_pc    <= 32'd0;
                IF_ID_btb_hit    <= 1'b0;
            end else if (dec_redirect) begin
                PC               <= dec_override_target;
                IF_ID_PC         <= 32'd0;
                IF_ID_instruct   <= 32'h00000013;
                IF_ID_valid      <= 1'b0;
                IF_ID_pred_taken <= 1'b0;
                IF_ID_pred_pc    <= 32'd0;
                IF_ID_btb_hit    <= 1'b0;
            end else if (!(issue_stall || halted_issued || trap_wb_stall)) begin
                PC               <= fetch_next_pc;
                IF_ID_PC         <= PC;
                IF_ID_instruct   <= i_imem_rdata;
                IF_ID_valid      <= 1'b1;
                IF_ID_pred_taken <= fetch_pred_taken;
                IF_ID_pred_pc    <= fetch_next_pc;
                IF_ID_btb_hit    <= fetch_btb_hit;
                IF_ID_ghr        <= ghr;
            end

            // Halt state (squash if speculative ebreak flushed)
            if (mispredict_fire)
                halted_issued <= 1'b0;
            else if (issue_fire && r_halt)
                halted_issued <= 1'b1;

            // GHR update / restore
            if (mispredict_fire) begin
                if (alu0_sel_branch)
                    ghr <= {rob_ghr_ckpt[alu0_sel_tag][5:0], alu_br_taken};
                else
                    ghr <= rob_ghr_ckpt[alu0_sel_tag];
            end else if (issue_fire && r_legal && r_branch) begin
                ghr <= {ghr[5:0], iss_pred_taken};
            end

            // RAS update / restore
            if (mispredict_fire) begin
                if (rob_is_ret[alu0_sel_tag] && !rob_is_call[alu0_sel_tag])
                    ras_sp <= rob_ras_sp_ckpt[alu0_sel_tag] - 3'd1;
                else if (rob_is_call[alu0_sel_tag] && !rob_is_ret[alu0_sel_tag]) begin
                    ras_stack[rob_ras_sp_ckpt[alu0_sel_tag]] <= alu0_sel_pc + 32'd4;
                    ras_sp <= rob_ras_sp_ckpt[alu0_sel_tag] + 3'd1;
                end else
                    ras_sp <= rob_ras_sp_ckpt[alu0_sel_tag];
            end else if (issue_fire) begin
                if (iss_is_ret && !iss_is_call)
                    ras_sp <= ras_sp - 3'd1;
                else if (iss_is_call && !iss_is_ret) begin
                    ras_stack[ras_sp] <= IF_ID_PC + 32'd4;
                    ras_sp <= ras_sp + 3'd1;
                end else if (iss_is_call && iss_is_ret) begin
                    ras_stack[ras_sp - 3'd1] <= IF_ID_PC + 32'd4;
                end
            end

            // Branch Predictor Training at Execute
            if (alu_resolve_fire && !alu_br_trap) begin
                if (alu0_sel_branch) begin
                    pht[upd_pht_idx] <= upd_pht_new;
                    if (alu_br_taken || (btb_valid[upd_btb_idx] && (btb_tag[upd_btb_idx] == alu0_sel_pc[31:8]))) begin
                        btb_valid[upd_btb_idx]  <= 1'b1;
                        btb_tag[upd_btb_idx]    <= alu0_sel_pc[31:8];
                        btb_target[upd_btb_idx] <= alu_br_target;
                        btb_uncond[upd_btb_idx] <= 1'b0;
                        btb_is_ret[upd_btb_idx] <= 1'b0;
                    end
                end else if (alu0_sel_jump) begin
                    btb_valid[upd_btb_idx]  <= 1'b1;
                    btb_tag[upd_btb_idx]    <= alu0_sel_pc[31:8];
                    btb_target[upd_btb_idx] <= alu_jmp_target;
                    btb_uncond[upd_btb_idx] <= 1'b1;
                    btb_is_ret[upd_btb_idx] <= rob_is_ret[alu0_sel_tag];
                end
            end
        end
    end

    // 10.1b Speculative RAT with 1-Cycle ROB Recovery on Mispredict
    wire [2:0] rob_slot_age [0:7];
    wire [7:0] rob_survives_age;
    genvar sk;
    generate
        for (sk = 0; sk < 8; sk = sk + 1) begin : gen_rob_surv
            localparam [2:0] SK_I = sk;
            assign rob_slot_age[sk]     = rob_head + SK_I;
            assign rob_survives_age[sk] = rob_valid[rob_slot_age[sk]] &&
                                          !(rob_commit && (sk == 0)) &&
                                          (SK_I <= misp_b_age);
        end
    endgenerate

    wire int_issue_valid  = issue_fire && !r_frd_en && (r_rd != 5'd0);
    wire int_commit_valid = rob_commit && !rob_frd_en[rob_head] && (rob_rd[rob_head] != 5'd0);
    wire fp_issue_valid   = issue_fire && r_frd_en;
    wire fp_commit_valid  = rob_commit && rob_frd_en[rob_head];

    genvar rr;
    generate
        for (rr = 0; rr < 32; rr = rr + 1) begin : gen_spec_rat
            localparam [4:0] RR_IDX = rr;

            wire [7:0] int_surv_wr;
            wire [7:0] fp_surv_wr;
            for (sk = 0; sk < 8; sk = sk + 1) begin : gen_rat_rec
                assign int_surv_wr[sk] = rob_survives_age[sk] && rob_int_wen[rob_slot_age[sk]] && (rob_rd[rob_slot_age[sk]] == RR_IDX);
                assign fp_surv_wr[sk]  = rob_survives_age[sk] && rob_frd_en[rob_slot_age[sk]]  && (rob_rd[rob_slot_age[sk]] == RR_IDX);
            end

            wire       int_rec_busy = |int_surv_wr;
            wire [2:0] int_rec_tag  =
                int_surv_wr[7] ? rob_slot_age[7] : int_surv_wr[6] ? rob_slot_age[6] :
                int_surv_wr[5] ? rob_slot_age[5] : int_surv_wr[4] ? rob_slot_age[4] :
                int_surv_wr[3] ? rob_slot_age[3] : int_surv_wr[2] ? rob_slot_age[2] :
                int_surv_wr[1] ? rob_slot_age[1] : rob_slot_age[0];

            wire       fp_rec_busy = |fp_surv_wr;
            wire [2:0] fp_rec_tag  =
                fp_surv_wr[7] ? rob_slot_age[7] : fp_surv_wr[6] ? rob_slot_age[6] :
                fp_surv_wr[5] ? rob_slot_age[5] : fp_surv_wr[4] ? rob_slot_age[4] :
                fp_surv_wr[3] ? rob_slot_age[3] : fp_surv_wr[2] ? rob_slot_age[2] :
                fp_surv_wr[1] ? rob_slot_age[1] : rob_slot_age[0];

            always @(posedge i_clk) begin
                if (i_rst) begin
                    spec_int_busy[rr] <= 1'b0;
                    spec_int_tag[rr]  <= 3'd0;
                end else if (rr == 0) begin
                    spec_int_busy[rr] <= 1'b0;
                    spec_int_tag[rr]  <= 3'd0;
                end else if (mispredict_fire) begin
                    spec_int_busy[rr] <= int_rec_busy;
                    if (int_rec_busy)
                        spec_int_tag[rr] <= int_rec_tag;
                end else if (int_issue_valid && (r_rd == RR_IDX)) begin
                    spec_int_busy[rr] <= 1'b1;
                    spec_int_tag[rr]  <= rob_tail;
                end else if (int_commit_valid && (rob_rd[rob_head] == RR_IDX) &&
                             spec_int_busy[rr] && (spec_int_tag[rr] == rob_head)) begin
                    spec_int_busy[rr] <= 1'b0;
                end
            end

            always @(posedge i_clk) begin
                if (i_rst) begin
                    spec_fp_busy[rr] <= 1'b0;
                    spec_fp_tag[rr]  <= 3'd0;
                end else if (mispredict_fire) begin
                    spec_fp_busy[rr] <= fp_rec_busy;
                    if (fp_rec_busy)
                        spec_fp_tag[rr] <= fp_rec_tag;
                end else if (fp_issue_valid && (r_rd == RR_IDX)) begin
                    spec_fp_busy[rr] <= 1'b1;
                    spec_fp_tag[rr]  <= rob_tail;
                end else if (fp_commit_valid && (rob_rd[rob_head] == RR_IDX) &&
                             spec_fp_busy[rr] && (spec_fp_tag[rr] == rob_head)) begin
                    spec_fp_busy[rr] <= 1'b0;
                end
            end
        end
    endgenerate

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

            if (mispredict_fire) begin
                rob_tail  <= alu0_sel_tag + 3'd1;
                rob_count <= {1'b0, misp_b_age} + 4'd1 - (rob_commit ? 4'd1 : 4'd0);
            end else begin
                if (issue_fire)
                    rob_tail <= rob_tail + 3'd1;
                rob_count <= rob_count + (issue_fire ? 4'd1 : 4'd0) - (rob_commit ? 4'd1 : 4'd0);
            end

            if (rob_commit && !rob_trap[rob_head] && rob_is_fpu[rob_head])
                arch_fcsr <= {commit_frm_next, commit_fflags_next};
        end
    end

    // 10.3 ROB Entries
    genvar rg, ls2;
    generate
        for (rg = 0; rg < ROB_SIZE; rg = rg + 1) begin : gen_rob_entry
            localparam [2:0] RG_TAG = rg;
            wire alloc_this   = issue_fire && (rob_tail == RG_TAG);
            wire commit_this  = rob_commit && (rob_head == RG_TAG);
            wire flush_this   = mispredict_fire && rob_valid[rg] && ((RG_TAG - rob_head) > misp_b_age);
            wire cdb_this     = cdb_valid  && (cdb_tag  == RG_TAG);
            wire alu_res_this = alu_resolve_fire && (alu0_sel_tag == RG_TAG);
            wire fp0_rm_this  = fp0_disp_fire && fp0_rm_trap && (fp_rs_tag[fp0_sel_slot] == RG_TAG);
            wire fp1_rm_this  = fp1_disp_fire && fp1_rm_trap && (fp_rs_tag[fp1_sel_slot] == RG_TAG);

            wire [7:0] ld_trap_vec;
            wire [7:0] st_prep_vec;
            for (ls2 = 0; ls2 < 8; ls2 = ls2 + 1) begin : gen_lsu_rob_match
                assign ld_trap_vec[ls2] = lsu_ld_trap_fire[ls2] && (lsu_tag[ls2] == RG_TAG);
                assign st_prep_vec[ls2] = lsu_st_prep_fire[ls2] && (lsu_tag[ls2] == RG_TAG);
            end
            wire ld_trap_this = |ld_trap_vec;
            wire st_prep_this = |st_prep_vec;
            wire [2:0] st_prep_slot =
                st_prep_vec[0] ? 3'd0 : st_prep_vec[1] ? 3'd1 : st_prep_vec[2] ? 3'd2 : st_prep_vec[3] ? 3'd3 :
                st_prep_vec[4] ? 3'd4 : st_prep_vec[5] ? 3'd5 : st_prep_vec[6] ? 3'd6 : 3'd7;

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
                    rob_pred_pc[rg]     <= 32'd0;
                    rob_ghr_ckpt[rg]    <= 7'd0;
                    rob_ras_sp_ckpt[rg] <= 3'd0;
                    rob_is_call[rg]     <= 1'b0;
                    rob_is_ret[rg]      <= 1'b0;
                end else if (flush_this) begin
                    rob_valid[rg]      <= 1'b0;
                    rob_done[rg]       <= 1'b0;
                    rob_is_fpu_csr[rg] <= 1'b0;
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
                    rob_pred_pc[rg]     <= iss_pred_pc;
                    rob_ghr_ckpt[rg]    <= ghr;
                    rob_ras_sp_ckpt[rg] <= ras_sp;
                    rob_is_call[rg]     <= iss_is_call;
                    rob_is_ret[rg]      <= iss_is_ret;
                end else if (commit_this) begin
                    rob_valid[rg]      <= 1'b0;
                    rob_done[rg]       <= 1'b0;
                    rob_is_fpu_csr[rg] <= 1'b0;
                end else if (rob_valid[rg]) begin
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

                    if (cdb_this) begin
                        rob_done[rg]  <= 1'b1;
                        rob_value[rg] <= cdb_val;
                        if (o_fp0_gnt)
                            rob_fflags[rg] <= fp0_cdb_fflags;
                        else if (o_fp1_gnt)
                            rob_fflags[rg] <= fp1_cdb_fflags;
                        if (o_ld_gnt) begin
                            rob_dmem_addr[rg]  <= lsu_ld_waddr[ld_cdb_pick_slot];
                            rob_dmem_mask[rg]  <= lsu_ld_mask[ld_cdb_pick_slot];
                            rob_dmem_ren[rg]   <= 1'b1;
                            rob_dmem_rdata[rg] <= lsu_ld_raw[ld_cdb_pick_slot];
                        end
                    end

                    if (alu_res_this) begin
                        rob_next_pc[rg] <= alu_next_pc;
                        if (alu_br_trap) begin
                            rob_trap[rg] <= 1'b1;
                            rob_done[rg] <= 1'b1;
                        end
                    end

                    if (fp0_disp_fire && !fp0_rm_trap && (fp_rs_tag[fp0_sel_slot] == RG_TAG) &&
                        fp_rs_is_csr[fp0_sel_slot]) begin
                        rob_frm_new[rg] <= fp0_csr_new_frm;
                        rob_frm_wen[rg] <= fp0_csr_frm_we;
                    end

                    if (fp0_rm_this || fp1_rm_this || ld_trap_this) begin
                        rob_trap[rg] <= 1'b1;
                        rob_done[rg] <= 1'b1;
                    end

                    if (st_prep_this) begin
                        rob_done[rg]       <= 1'b1;
                        rob_trap[rg]       <= rob_trap[rg] | lsu_misal_c[st_prep_slot];
                        rob_dmem_addr[rg]  <= lsu_waddr_c[st_prep_slot];
                        rob_dmem_mask[rg]  <= lsu_misal_c[st_prep_slot] ? 4'd0 : lsu_mask_c[st_prep_slot];
                        rob_dmem_wen[rg]   <= !lsu_misal_c[st_prep_slot];
                        rob_dmem_wdata[rg] <= lsu_wdata_c[st_prep_slot];
                    end
                end
            end
        end
    endgenerate

    // 10.4 ALU0, ALU1, DIV Unit CDB Buffers (with Mispredict Flush)
    wire alu1_cdb_flush   = mispredict_fire && alu1_cdb_v && ((alu1_cdb_tag - rob_head) > misp_b_age);
    wire alu1_disp_flush  = mispredict_fire && ((alu_rs_tag[alu1_sel_slot] - rob_head) > misp_b_age);
    wire div_cdb_flush    = mispredict_fire && div_cdb_v && ((div_cdb_tag - rob_head) > misp_b_age);

    always @(posedge i_clk) begin
        if (i_rst) begin
            div_active      <= 1'b0;
            div_active_slot <= 3'd0;
            alu0_cdb_v      <= 1'b0;
            alu0_cdb_tag    <= 3'd0;
            alu0_cdb_rd     <= 5'd0;
            alu0_cdb_val    <= 32'd0;
            alu0_cdb_pc     <= 32'd0;
            alu1_cdb_v      <= 1'b0;
            alu1_cdb_tag    <= 3'd0;
            alu1_cdb_rd     <= 5'd0;
            alu1_cdb_val    <= 32'd0;
            alu1_cdb_pc     <= 32'd0;
            div_cdb_v       <= 1'b0;
            div_cdb_tag     <= 3'd0;
            div_cdb_rd      <= 5'd0;
            div_cdb_val     <= 32'd0;
            div_cdb_pc      <= 32'd0;
        end else begin
            if (div_slot_flushed) begin
                div_active <= 1'b0;
            end else if (div_sel_valid && div_out_busy) begin
                div_active      <= 1'b1;
                div_active_slot <= div_sel_slot;
            end else if (div_disp_fire) begin
                div_active      <= 1'b0;
            end

            if (alu0_disp_fire && !alu_br_trap) begin
                alu0_cdb_v   <= 1'b1;
                alu0_cdb_tag <= alu0_sel_tag;
                alu0_cdb_rd  <= alu0_sel_branch ? 5'd0 : alu0_sel_rd;
                alu0_cdb_val <= alu0_cdb_calc_val;
                alu0_cdb_pc  <= alu0_sel_pc;
            end else if (alu0_cdb_gnt) begin
                alu0_cdb_v <= 1'b0;
            end

            if (alu1_cdb_flush) begin
                alu1_cdb_v <= 1'b0;
            end else if (alu1_disp_fire && !alu1_disp_flush) begin
                alu1_cdb_v   <= 1'b1;
                alu1_cdb_tag <= alu_rs_tag[alu1_sel_slot];
                alu1_cdb_rd  <= alu_rs_rd[alu1_sel_slot];
                alu1_cdb_val <= alu1_cdb_calc_val;
                alu1_cdb_pc  <= alu_rs_pc[alu1_sel_slot];
            end else if (alu1_cdb_gnt) begin
                alu1_cdb_v <= 1'b0;
            end

            if (div_cdb_flush) begin
                div_cdb_v <= 1'b0;
            end else if (div_disp_fire && !div_slot_flushed) begin
                div_cdb_v   <= 1'b1;
                div_cdb_tag <= alu_rs_tag[div_sel_slot];
                div_cdb_rd  <= alu_rs_rd[div_sel_slot];
                div_cdb_val <= div_out_result;
                div_cdb_pc  <= alu_rs_pc[div_sel_slot];
            end else if (div_cdb_gnt) begin
                div_cdb_v <= 1'b0;
            end
        end
    end

    // ALU RS Entries
    genvar as;
    generate
        for (as = 0; as < ALU_RS_SIZE; as = as + 1) begin : gen_alu_rs_entry
            localparam [2:0] AS_IDX = as;
            wire alloc_as = issue_fire && (cls_alu || cls_brjmp) && (alu_rs_free_idx == AS_IDX);
            wire flush_as = mispredict_fire && alu_rs_valid[as] && ((alu_rs_tag[as] - rob_head) > misp_b_age);
            wire free_as  = (alu0_disp_fire && (alu0_sel_slot == AS_IDX)) ||
                            (alu1_disp_fire && (alu1_sel_slot == AS_IDX)) ||
                            (div_disp_fire  && (div_sel_slot  == AS_IDX));

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
                end else if (flush_as) begin
                    alu_rs_valid[as] <= 1'b0;
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

    // 10.5 FP RS Entries, FPU Busy Trackers, FPU CDB Buffers (with Mispredict Flush)
    wire fp0_cdb_flush = mispredict_fire && fp0_cdb_v && ((fp0_cdb_tag - rob_head) > misp_b_age);
    wire fp1_cdb_flush = mispredict_fire && fp1_cdb_v && ((fp1_cdb_tag - rob_head) > misp_b_age);

    always @(posedge i_clk) begin
        if (i_rst) begin
            fp0_busy_slot_v <= 1'b0;
            fp0_busy_slot   <= 3'd0;
            fp1_busy_slot_v <= 1'b0;
            fp1_busy_slot   <= 3'd0;
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
        end else begin
            if (fp0_slot_flushed) begin
                fp0_busy_slot_v <= 1'b0;
            end else if (fp0_sel_valid && fp0_busy) begin
                fp0_busy_slot_v <= 1'b1;
                fp0_busy_slot   <= fp0_sel_slot;
            end else if (fp0_disp_fire) begin
                fp0_busy_slot_v <= 1'b0;
            end

            if (fp1_slot_flushed) begin
                fp1_busy_slot_v <= 1'b0;
            end else if (fp1_sel_valid && fp1_busy) begin
                fp1_busy_slot_v <= 1'b1;
                fp1_busy_slot   <= fp1_sel_slot;
            end else if (fp1_disp_fire) begin
                fp1_busy_slot_v <= 1'b0;
            end

            if (fp0_cdb_flush) begin
                fp0_cdb_v <= 1'b0;
            end else if (fp0_disp_fire && !fp0_rm_trap && !fp0_slot_flushed) begin
                fp0_cdb_v      <= 1'b1;
                fp0_cdb_tag    <= fp_rs_tag[fp0_sel_slot];
                fp0_cdb_rd     <= fp_rs_rd[fp0_sel_slot];
                fp0_cdb_val    <= fp0_res_val;
                fp0_cdb_pc     <= fp_rs_pc[fp0_sel_slot];
                fp0_cdb_fflags <= fp0_fflags;
            end else if (o_fp0_gnt) begin
                fp0_cdb_v <= 1'b0;
            end

            if (fp1_cdb_flush) begin
                fp1_cdb_v <= 1'b0;
            end else if (fp1_disp_fire && !fp1_rm_trap && !fp1_slot_flushed) begin
                fp1_cdb_v      <= 1'b1;
                fp1_cdb_tag    <= fp_rs_tag[fp1_sel_slot];
                fp1_cdb_rd     <= fp_rs_rd[fp1_sel_slot];
                fp1_cdb_val    <= fp1_res_val;
                fp1_cdb_pc     <= fp_rs_pc[fp1_sel_slot];
                fp1_cdb_fflags <= fp1_fflags;
            end else if (o_fp1_gnt) begin
                fp1_cdb_v <= 1'b0;
            end
        end
    end

    genvar fs;
    generate
        for (fs = 0; fs < FP_RS_SIZE; fs = fs + 1) begin : gen_fp_rs_entry
            localparam [2:0] FS_IDX = fs;
            wire alloc_fs = issue_fire && cls_fpu && (fp_rs_free_idx == FS_IDX);
            wire flush_fs = mispredict_fire && fp_rs_valid[fs] && ((fp_rs_tag[fs] - rob_head) > misp_b_age);
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
                end else if (flush_fs) begin
                    fp_rs_valid[fs] <= 1'b0;
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

    // 10.6 LSU Pointers & Entries (with Mispredict Flush)
    wire [7:0] lsu_flush_vec;
    wire [7:0] lsu_surv_vec;
    generate
        for (ls = 0; ls < LSU_SIZE; ls = ls + 1) begin : gen_lsu_surv
            localparam [2:0] LS_I = ls;
            assign lsu_flush_vec[ls] = mispredict_fire && lsu_valid[ls] && ((lsu_tag[ls] - rob_head) > misp_b_age);
            assign lsu_surv_vec[ls]  = lsu_valid[ls] && !lsu_flush_vec[ls] && !(lsu_pop && (lsu_head == LS_I));
        end
    endgenerate

    wire [3:0] surv_lsu_count =
        {3'd0, lsu_surv_vec[0]} + {3'd0, lsu_surv_vec[1]} + {3'd0, lsu_surv_vec[2]} + {3'd0, lsu_surv_vec[3]} +
        {3'd0, lsu_surv_vec[4]} + {3'd0, lsu_surv_vec[5]} + {3'd0, lsu_surv_vec[6]} + {3'd0, lsu_surv_vec[7]};
    wire [2:0] next_lsu_head = lsu_pop ? (lsu_head + 3'd1) : lsu_head;

    always @(posedge i_clk) begin
        if (i_rst) begin
            lsu_head  <= 3'd0;
            lsu_tail  <= 3'd0;
            lsu_count <= 4'd0;
        end else if (mispredict_fire) begin
            lsu_head  <= next_lsu_head;
            lsu_tail  <= next_lsu_head + surv_lsu_count[2:0];
            lsu_count <= surv_lsu_count;
        end else begin
            if (lsu_pop)
                lsu_head <= lsu_head + 3'd1;
            if (lsu_push)
                lsu_tail <= lsu_tail + 3'd1;
            lsu_count <= lsu_count + (lsu_push ? 4'd1 : 4'd0) - (lsu_pop ? 4'd1 : 4'd0);
        end
    end

    generate
        for (ls = 0; ls < LSU_SIZE; ls = ls + 1) begin : gen_lsu_entry
            localparam [2:0] LS_IDX = ls;
            wire alloc_ls     = lsu_push && (lsu_tail == LS_IDX);
            wire pop_ls       = lsu_pop  && (lsu_head == LS_IDX);
            wire flush_ls     = lsu_flush_vec[ls];
            wire prep_ls      = lsu_st_prep_fire[ls];
            wire ld_trap_ls   = lsu_ld_trap_fire[ls];
            wire ld_fwd_ls    = lsu_ld_fwd_fire[ls];
            wire ld_dc_hit_ls = dcache_hit_fire  && (ld_dcache_pick_slot == LS_IDX);
            wire ld_dc_mis_ls = dcache_miss_fire && (ld_dcache_pick_slot == LS_IDX);
            wire ld_fill_ls   = dcache_fill_valid && lsu_valid[ls] && lsu_is_load[ls] &&
                                (lsu_ld_state[ls] == LD_MISS) && (lsu_ld_blk_addr[ls] == dcache_fill_addr);
            wire ld_gnt_ls    = o_ld_gnt && (ld_cdb_pick_slot == LS_IDX);

            always @(posedge i_clk) begin
                if (i_rst) begin
                    lsu_valid[ls]       <= 1'b0;
                    lsu_tag[ls]         <= 3'd0;
                    lsu_pc[ls]          <= 32'd0;
                    lsu_rd[ls]          <= 5'd0;
                    lsu_is_load[ls]     <= 1'b0;
                    lsu_imm[ls]         <= 32'd0;
                    lsu_memb[ls]        <= 1'b0;
                    lsu_memh[ls]        <= 1'b0;
                    lsu_memw[ls]        <= 1'b0;
                    lsu_memu[ls]        <= 1'b0;
                    lsu_op1_val[ls]     <= 32'd0;
                    lsu_q1_v[ls]        <= 1'b0;
                    lsu_q1_t[ls]        <= 3'd0;
                    lsu_op2_val[ls]     <= 32'd0;
                    lsu_q2_v[ls]        <= 1'b0;
                    lsu_q2_t[ls]        <= 3'd0;
                    lsu_st_prepped[ls]  <= 1'b0;
                    lsu_st_baddr[ls]    <= 32'd0;
                    lsu_st_mask[ls]     <= 4'd0;
                    lsu_st_wdata[ls]    <= 32'd0;
                    lsu_ld_state[ls]    <= LD_WAIT;
                    lsu_ld_val[ls]      <= 32'd0;
                    lsu_ld_raw[ls]      <= 32'd0;
                    lsu_ld_waddr[ls]    <= 32'd0;
                    lsu_ld_mask[ls]     <= 4'd0;
                    lsu_ld_blk_addr[ls] <= 32'd0;
                    lsu_ld_byte_off[ls] <= 2'd0;
                end else if (pop_ls || flush_ls) begin
                    lsu_valid[ls]      <= 1'b0;
                    lsu_st_prepped[ls] <= 1'b0;
                    lsu_ld_state[ls]   <= LD_WAIT;
                end else if (alloc_ls) begin
                    lsu_valid[ls]       <= 1'b1;
                    lsu_tag[ls]         <= rob_tail;
                    lsu_pc[ls]          <= IF_ID_PC;
                    lsu_rd[ls]          <= r_rd;
                    lsu_is_load[ls]     <= r_dmem_ren;
                    lsu_imm[ls]         <= r_immediate;
                    lsu_memb[ls]        <= r_dmem_memb;
                    lsu_memh[ls]        <= r_dmem_memh;
                    lsu_memw[ls]        <= r_dmem_memw;
                    lsu_memu[ls]        <= r_dmem_memu;
                    lsu_op1_val[ls]     <= iss_op1_val;
                    lsu_q1_v[ls]        <= iss_op1_qv;
                    lsu_q1_t[ls]        <= iss_op1_qt;
                    lsu_op2_val[ls]     <= iss_op2_val;
                    lsu_q2_v[ls]        <= iss_op2_qv;
                    lsu_q2_t[ls]        <= iss_op2_qt;
                    lsu_st_prepped[ls]  <= 1'b0;
                    lsu_ld_state[ls]    <= LD_WAIT;
                end else if (lsu_valid[ls]) begin
                    if (prep_ls) begin
                        lsu_st_prepped[ls] <= 1'b1;
                        lsu_st_baddr[ls]   <= lsu_eff_addr[ls];
                        lsu_st_mask[ls]    <= lsu_misal_c[ls] ? 4'd0 : lsu_mask_c[ls];
                        lsu_st_wdata[ls]   <= lsu_wdata_c[ls];
                    end

                    if (ld_trap_ls) begin
                        lsu_ld_state[ls] <= LD_DONE;
                    end else if (ld_fwd_ls) begin
                        lsu_ld_state[ls] <= LD_CDB;
                        lsu_ld_val[ls]   <= lsu_ld_fwd_ext[ls];
                        lsu_ld_raw[ls]   <= lsu_ld_fwd_raw[ls];
                        lsu_ld_waddr[ls] <= lsu_waddr_c[ls];
                        lsu_ld_mask[ls]  <= lsu_mask_c[ls];
                    end else if (ld_dc_hit_ls) begin
                        lsu_ld_state[ls] <= LD_CDB;
                        lsu_ld_val[ls]   <= dc_hit_ext;
                        lsu_ld_raw[ls]   <= dcache_rdata;
                        lsu_ld_waddr[ls] <= lsu_waddr_c[ls];
                        lsu_ld_mask[ls]  <= lsu_mask_c[ls];
                    end else if (ld_dc_mis_ls) begin
                        lsu_ld_state[ls]    <= LD_MISS;
                        lsu_ld_waddr[ls]    <= lsu_waddr_c[ls];
                        lsu_ld_mask[ls]     <= lsu_mask_c[ls];
                        lsu_ld_blk_addr[ls] <= lsu_blk_addr_c[ls];
                        lsu_ld_byte_off[ls] <= lsu_boff_c[ls];
                    end else if (ld_fill_ls) begin
                        lsu_ld_state[ls] <= LD_CDB;
                        lsu_ld_val[ls]   <= lsu_ld_fill_ext[ls];
                        lsu_ld_raw[ls]   <= lsu_ld_fill_raw[ls];
                    end else if (ld_gnt_ls) begin
                        lsu_ld_state[ls] <= LD_DONE;
                    end

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
