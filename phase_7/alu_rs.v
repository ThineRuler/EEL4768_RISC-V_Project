`default_nettype none

module alu_rs (
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire [ 2:0] i_rob_head,

    // Allocate the ROB and RS together on issue valid and ready.
    input  wire        i_issue_valid,
    output wire        o_issue_ready,
    input  wire [ 2:0] i_issue_tag,
    input  wire [ 4:0] i_issue_rd,
    input  wire [31:0] i_issue_pc,
    input  wire [31:0] i_issue_imm,
    input  wire        i_src1_ready,
    input  wire [ 2:0] i_src1_tag,
    input  wire [31:0] i_src1_value,
    input  wire        i_src2_ready,
    input  wire [ 2:0] i_src2_tag,
    input  wire [31:0] i_src2_value,

    // Use the Phase 6 decoder controls. LUI is rd_sel[1], JALR is pc_sel.
    input  wire [ 2:0] i_alu_opsel,
    input  wire        i_alu_sub,
    input  wire        i_alu_unsigned,
    input  wire        i_alu_arith,
    input  wire        i_mext,
    input  wire        i_op1_pc,
    input  wire        i_op2_imm,
    input  wire        i_lui,
    input  wire        i_branch,
    input  wire        i_branch_equal,
    input  wire        i_branch_invert,
    input  wire        i_jump,
    input  wire        i_jalr,

    input  wire        i_cdb_valid,
    input  wire [ 2:0] i_cdb_tag,
    input  wire [31:0] i_cdb_value,

    input  wire        i_result_grant,
    output wire        o_result_valid,
    output reg  [ 2:0] o_result_tag,
    output reg  [ 4:0] o_result_rd,
    output reg  [31:0] o_result_pc,
    output reg  [31:0] o_result_value,

    // A not-taken branch still resolves and supplies its fall-through PC.
    output wire        o_ctrl_resolved,
    output wire        o_redirect_valid,
    output wire [ 2:0] o_ctrl_tag,
    output wire [31:0] o_ctrl_next_pc
);
    reg [3:0] occupied;
    reg [2:0] tag [0:3];
    reg [4:0] rd [0:3];
    reg [31:0] pc [0:3];
    reg [31:0] imm [0:3];
    reg [2:0] opsel [0:3];
    reg [3:0] sub, unsigned_cmp, arith, mext, op1_pc, op2_imm;
    reg [3:0] lui, branch, branch_equal, branch_invert, jump, jalr;
    reg [3:0] src1_ready, src2_ready;
    reg [2:0] src1_tag [0:3];
    reg [2:0] src2_tag [0:3];
    reg [31:0] src1_value [0:3];
    reg [31:0] src2_value [0:3];

    wire [3:0] eligible, free_slot;
    wire [2:0] age [0:3];
    wire pick1, pick3, pick23;
    wire [2:0] age01, age23;
    wire [1:0] selected, issue_slot, exec_slot;
    wire issue_fire, start_exec, enqueue_result, room_for_result;
    wire issue_ignores_src1, issue_ignores_src2, bypass_src1, bypass_src2;
    wire [31:0] alu_op1, alu_op2, alu_value, pc_plus4, pc_relative;
    wire alu_busy, alu_eq, alu_slt, branch_taken;
    reg result_valid;
    reg dividing;
    reg [1:0] divide_slot;

    assign pick1 = eligible[1] && (!eligible[0] || age[1] < age[0]);
    assign pick3 = eligible[3] && (!eligible[2] || age[3] < age[2]);
    assign age01 = pick1 ? age[1] : age[0];
    assign age23 = pick3 ? age[3] : age[2];
    assign pick23 = (|eligible[3:2]) && (!(|eligible[1:0]) || age23 < age01);
    assign selected = pick23 ? {1'b1, pick3} : {1'b0, pick1};

    assign o_issue_ready = !i_rst && (|free_slot);
    assign issue_fire = i_issue_valid && o_issue_ready;
    assign issue_slot = free_slot[0] ? 2'd0 : free_slot[1] ? 2'd1 :
                        free_slot[2] ? 2'd2 : 2'd3;
    assign issue_ignores_src1 = i_op1_pc || i_lui || (i_jump && !i_jalr);
    assign issue_ignores_src2 = i_op2_imm || i_lui || i_jump;
    assign bypass_src1 = !i_src1_ready && i_cdb_valid && i_src1_tag == i_cdb_tag;
    assign bypass_src2 = !i_src2_ready && i_cdb_valid && i_src2_tag == i_cdb_tag;

    genvar slot;
    generate
        for (slot = 0; slot < 4; slot = slot + 1) begin : station
            localparam [1:0] SLOT = slot[1:0];
            wire releasing;
            assign releasing = enqueue_result && exec_slot == SLOT;
            assign free_slot[slot] = !occupied[slot] || releasing;
            assign eligible[slot] = occupied[slot] && src1_ready[slot] && src2_ready[slot];
            assign age[slot] = tag[slot] - i_rob_head;

            always @(posedge i_clk) begin
                if (i_rst) begin
                    occupied[slot] <= 1'b0;
                    src1_ready[slot] <= 1'b0;
                    src2_ready[slot] <= 1'b0;
                end else if (issue_fire && issue_slot == SLOT) begin
                    occupied[slot] <= 1'b1;
                    tag[slot] <= i_issue_tag;
                    rd[slot] <= i_issue_rd;
                    pc[slot] <= i_issue_pc;
                    imm[slot] <= i_issue_imm;
                    opsel[slot] <= i_alu_opsel;
                    sub[slot] <= i_alu_sub;
                    unsigned_cmp[slot] <= i_alu_unsigned;
                    arith[slot] <= i_alu_arith;
                    mext[slot] <= i_mext;
                    op1_pc[slot] <= i_op1_pc;
                    op2_imm[slot] <= i_op2_imm;
                    lui[slot] <= i_lui;
                    branch[slot] <= i_branch;
                    branch_equal[slot] <= i_branch_equal;
                    branch_invert[slot] <= i_branch_invert;
                    jump[slot] <= i_jump;
                    jalr[slot] <= i_jalr;
                    src1_ready[slot] <= issue_ignores_src1 || i_src1_ready || bypass_src1;
                    src2_ready[slot] <= issue_ignores_src2 || i_src2_ready || bypass_src2;
                    src1_tag[slot] <= i_src1_tag;
                    src2_tag[slot] <= i_src2_tag;
                    src1_value[slot] <= issue_ignores_src1 ? 32'd0 :
                                        bypass_src1 ? i_cdb_value : i_src1_value;
                    src2_value[slot] <= issue_ignores_src2 ? 32'd0 :
                                        bypass_src2 ? i_cdb_value : i_src2_value;
                end else if (releasing) begin
                    occupied[slot] <= 1'b0;
                end else if (occupied[slot] && i_cdb_valid) begin
                    if (!src1_ready[slot] && src1_tag[slot] == i_cdb_tag) begin
                        src1_ready[slot] <= 1'b1;
                        src1_value[slot] <= i_cdb_value;
                    end
                    if (!src2_ready[slot] && src2_tag[slot] == i_cdb_tag) begin
                        src2_ready[slot] <= 1'b1;
                        src2_value[slot] <= i_cdb_value;
                    end
                end
            end
        end
    endgenerate

    assign o_result_valid = !i_rst && result_valid;
    assign room_for_result = !result_valid || i_result_grant;
    assign start_exec = !i_rst && !dividing && room_for_result && (|eligible);
    assign exec_slot = dividing ? divide_slot : selected;
    assign enqueue_result = !i_rst && room_for_result && (start_exec || dividing) && !alu_busy;

    assign alu_op1 = op1_pc[exec_slot] ? pc[exec_slot] : src1_value[exec_slot];
    assign alu_op2 = op2_imm[exec_slot] ? imm[exec_slot] : src2_value[exec_slot];

    alu integer_alu (
        .i_clk(i_clk), .i_rst(i_rst),
        .i_opsel(opsel[exec_slot]), .i_sub(sub[exec_slot]),
        .i_unsigned(unsigned_cmp[exec_slot]), .i_arith(arith[exec_slot]),
        .i_mext((start_exec || dividing) && mext[exec_slot]),
        .i_advance(enqueue_result), .i_op1(alu_op1), .i_op2(alu_op2),
        .o_result(alu_value), .o_eq(alu_eq), .o_slt(alu_slt), .o_busy(alu_busy)
    );

    assign pc_plus4 = pc[exec_slot] + 32'd4;
    assign pc_relative = pc[exec_slot] + imm[exec_slot];
    assign branch_taken = branch[exec_slot] &&
        ((branch_equal[exec_slot] ? alu_eq : alu_slt) ^ branch_invert[exec_slot]);
    assign o_ctrl_resolved = start_exec && (branch[exec_slot] || jump[exec_slot]);
    assign o_redirect_valid = o_ctrl_resolved && (branch_taken || jump[exec_slot]);
    assign o_ctrl_tag = tag[exec_slot];
    assign o_ctrl_next_pc = o_redirect_valid ?
                           (jalr[exec_slot] ? {alu_value[31:1], 1'b0} : pc_relative) : pc_plus4;

    always @(posedge i_clk) begin
        if (i_rst) begin
            result_valid <= 1'b0;
            dividing <= 1'b0;
            divide_slot <= 2'd0;
        end else begin
            if (start_exec && alu_busy) begin
                // Keep the operands and tag in this station until DIV/REM finishes.
                dividing <= 1'b1;
                divide_slot <= exec_slot;
            end else if (enqueue_result) begin
                dividing <= 1'b0;
            end

            if (room_for_result) begin
                result_valid <= enqueue_result;
                if (enqueue_result) begin
                    o_result_tag <= tag[exec_slot];
                    o_result_rd <= branch[exec_slot] ? 5'd0 : rd[exec_slot];
                    o_result_pc <= pc[exec_slot];
                    o_result_value <= branch[exec_slot] ? 32'd0 :
                                      jump[exec_slot] ? pc_plus4 :
                                      lui[exec_slot] ? imm[exec_slot] : alu_value;
                end
            end
        end
    end
endmodule

`default_nettype wire
