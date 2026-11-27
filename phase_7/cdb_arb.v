`default_nettype none

// Single-CDB age-ordered arbiter across INT ALU, dual FP ALUs, and LSU Load.
// Lower (tag - i_rob_head) mod 2^TAG_W wins; ties break deterministically
// (ALU > FP0 > FP1 > Load) to guarantee strict mutual exclusion.
module cdb_arb #(
    parameter TAG_W = 3
) (
    input  wire [TAG_W-1:0] i_rob_head,

    input  wire             i_alu_v,
    input  wire [TAG_W-1:0] i_alu_tag,
    input  wire [4:0]       i_alu_rd,
    input  wire [31:0]      i_alu_val,
    input  wire [31:0]      i_alu_pc,

    input  wire             i_fp0_v,
    input  wire [TAG_W-1:0] i_fp0_tag,
    input  wire [4:0]       i_fp0_rd,
    input  wire [31:0]      i_fp0_val,
    input  wire [31:0]      i_fp0_pc,

    input  wire             i_fp1_v,
    input  wire [TAG_W-1:0] i_fp1_tag,
    input  wire [4:0]       i_fp1_rd,
    input  wire [31:0]      i_fp1_val,
    input  wire [31:0]      i_fp1_pc,

    input  wire             i_ld_v,
    input  wire [TAG_W-1:0] i_ld_tag,
    input  wire [4:0]       i_ld_rd,
    input  wire [31:0]      i_ld_val,
    input  wire [31:0]      i_ld_pc,

    output wire             o_alu_gnt,
    output wire             o_fp0_gnt,
    output wire             o_fp1_gnt,
    output wire             o_ld_gnt,

    output wire             o_cdb_valid,
    output wire [TAG_W-1:0] o_cdb_tag,
    output wire [4:0]       o_cdb_rd,
    output wire [31:0]      o_cdb_val,
    output wire [31:0]      o_cdb_pc
);

    wire [TAG_W-1:0] alu_age = i_alu_tag - i_rob_head;
    wire [TAG_W-1:0] fp0_age = i_fp0_tag - i_rob_head;
    wire [TAG_W-1:0] fp1_age = i_fp1_tag - i_rob_head;
    wire [TAG_W-1:0] ld_age  = i_ld_tag  - i_rob_head;

    assign o_alu_gnt = i_alu_v &&
                       (!i_fp0_v || (alu_age <= fp0_age)) &&
                       (!i_fp1_v || (alu_age <= fp1_age)) &&
                       (!i_ld_v  || (alu_age <= ld_age));

    assign o_fp0_gnt = !o_alu_gnt && i_fp0_v &&
                       (!i_fp1_v || (fp0_age <= fp1_age)) &&
                       (!i_ld_v  || (fp0_age <= ld_age));

    assign o_fp1_gnt = !o_alu_gnt && !o_fp0_gnt && i_fp1_v &&
                       (!i_ld_v  || (fp1_age <= ld_age));

    assign o_ld_gnt  = !o_alu_gnt && !o_fp0_gnt && !o_fp1_gnt && i_ld_v;

    assign o_cdb_valid = o_alu_gnt | o_fp0_gnt | o_fp1_gnt | o_ld_gnt;

    assign o_cdb_tag = o_alu_gnt ? i_alu_tag :
                       o_fp0_gnt ? i_fp0_tag :
                       o_fp1_gnt ? i_fp1_tag :
                                   i_ld_tag;

    assign o_cdb_rd  = o_alu_gnt ? i_alu_rd :
                       o_fp0_gnt ? i_fp0_rd :
                       o_fp1_gnt ? i_fp1_rd :
                                   i_ld_rd;

    assign o_cdb_val = o_alu_gnt ? i_alu_val :
                       o_fp0_gnt ? i_fp0_val :
                       o_fp1_gnt ? i_fp1_val :
                                   i_ld_val;

    assign o_cdb_pc  = o_alu_gnt ? i_alu_pc :
                       o_fp0_gnt ? i_fp0_pc :
                       o_fp1_gnt ? i_fp1_pc :
                                   i_ld_pc;

endmodule

`default_nettype wire
