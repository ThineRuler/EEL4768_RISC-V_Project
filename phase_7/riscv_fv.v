// Graded FV harness for cdb_arb and rat.
// Compiles cleanly under Verilog-2005 (iverilog -g2005) as well as SystemVerilog
// under SymbiYosys (`ifdef FORMAL).
`default_nettype none

module riscv_fv (
    input  wire        i_clk,
    input  wire        i_rst,

    // CDB arbiter symbolic inputs
    input  wire [2:0]  i_rob_head,
    input  wire        i_alu_v,
    input  wire [2:0]  i_alu_tag,
    input  wire [4:0]  i_alu_rd,
    input  wire [31:0] i_alu_val,
    input  wire [31:0] i_alu_pc,

    input  wire        i_fp0_v,
    input  wire [2:0]  i_fp0_tag,
    input  wire [4:0]  i_fp0_rd,
    input  wire [31:0] i_fp0_val,
    input  wire [31:0] i_fp0_pc,

    input  wire        i_fp1_v,
    input  wire [2:0]  i_fp1_tag,
    input  wire [4:0]  i_fp1_rd,
    input  wire [31:0] i_fp1_val,
    input  wire [31:0] i_fp1_pc,

    input  wire        i_ld_v,
    input  wire [2:0]  i_ld_tag,
    input  wire [4:0]  i_ld_rd,
    input  wire [31:0] i_ld_val,
    input  wire [31:0] i_ld_pc,

    // INT RAT symbolic inputs
    input  wire [4:0]  i_int_raddr1,
    input  wire [4:0]  i_int_raddr2,
    input  wire [4:0]  i_int_raddr3,
    input  wire        i_int_issue_en,
    input  wire [4:0]  i_int_issue_rd,
    input  wire [2:0]  i_int_issue_tag,
    input  wire        i_int_commit_en,
    input  wire [4:0]  i_int_commit_rd,
    input  wire [2:0]  i_int_commit_tag,

    // FP RAT symbolic inputs
    input  wire [4:0]  i_fp_raddr1,
    input  wire [4:0]  i_fp_raddr2,
    input  wire [4:0]  i_fp_raddr3,
    input  wire        i_fp_issue_en,
    input  wire [4:0]  i_fp_issue_rd,
    input  wire [2:0]  i_fp_issue_tag,
    input  wire        i_fp_commit_en,
    input  wire [4:0]  i_fp_commit_rd,
    input  wire [2:0]  i_fp_commit_tag
);

    wire        o_alu_gnt, o_fp0_gnt, o_fp1_gnt, o_ld_gnt, o_cdb_valid;
    wire [2:0]  o_cdb_tag;
    wire [4:0]  o_cdb_rd;
    wire [31:0] o_cdb_val;
    wire [31:0] o_cdb_pc;

    cdb_arb #(.TAG_W(3)) dut_cdb (
        .i_rob_head  (i_rob_head),
        .i_alu_v     (i_alu_v),
        .i_alu_tag   (i_alu_tag),
        .i_alu_rd    (i_alu_rd),
        .i_alu_val   (i_alu_val),
        .i_alu_pc    (i_alu_pc),
        .i_fp0_v     (i_fp0_v),
        .i_fp0_tag   (i_fp0_tag),
        .i_fp0_rd    (i_fp0_rd),
        .i_fp0_val   (i_fp0_val),
        .i_fp0_pc    (i_fp0_pc),
        .i_fp1_v     (i_fp1_v),
        .i_fp1_tag   (i_fp1_tag),
        .i_fp1_rd    (i_fp1_rd),
        .i_fp1_val   (i_fp1_val),
        .i_fp1_pc    (i_fp1_pc),
        .i_ld_v      (i_ld_v),
        .i_ld_tag    (i_ld_tag),
        .i_ld_rd     (i_ld_rd),
        .i_ld_val    (i_ld_val),
        .i_ld_pc     (i_ld_pc),
        .o_alu_gnt   (o_alu_gnt),
        .o_fp0_gnt   (o_fp0_gnt),
        .o_fp1_gnt   (o_fp1_gnt),
        .o_ld_gnt    (o_ld_gnt),
        .o_cdb_valid (o_cdb_valid),
        .o_cdb_tag   (o_cdb_tag),
        .o_cdb_rd    (o_cdb_rd),
        .o_cdb_val   (o_cdb_val),
        .o_cdb_pc    (o_cdb_pc)
    );

    wire        o_int_busy1, o_int_busy2, o_int_busy3;
    wire [2:0]  o_int_tag1,  o_int_tag2,  o_int_tag3;

    rat #(.TAG_W(3), .ZERO_REG(1)) dut_rat_int (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .i_raddr1     (i_int_raddr1),
        .o_busy1      (o_int_busy1),
        .o_tag1       (o_int_tag1),
        .i_raddr2     (i_int_raddr2),
        .o_busy2      (o_int_busy2),
        .o_tag2       (o_int_tag2),
        .i_raddr3     (i_int_raddr3),
        .o_busy3      (o_int_busy3),
        .o_tag3       (o_int_tag3),
        .i_issue_en   (i_int_issue_en),
        .i_issue_rd   (i_int_issue_rd),
        .i_issue_tag  (i_int_issue_tag),
        .i_commit_en  (i_int_commit_en),
        .i_commit_rd  (i_int_commit_rd),
        .i_commit_tag (i_int_commit_tag)
    );

    wire        o_fp_busy1, o_fp_busy2, o_fp_busy3;
    wire [2:0]  o_fp_tag1,  o_fp_tag2,  o_fp_tag3;

    rat #(.TAG_W(3), .ZERO_REG(0)) dut_rat_fp (
        .i_clk        (i_clk),
        .i_rst        (i_rst),
        .i_raddr1     (i_fp_raddr1),
        .o_busy1      (o_fp_busy1),
        .o_tag1       (o_fp_tag1),
        .i_raddr2     (i_fp_raddr2),
        .o_busy2      (o_fp_busy2),
        .o_tag2       (o_fp_tag2),
        .i_raddr3     (i_fp_raddr3),
        .o_busy3      (o_fp_busy3),
        .o_tag3       (o_fp_tag3),
        .i_issue_en   (i_fp_issue_en),
        .i_issue_rd   (i_fp_issue_rd),
        .i_issue_tag  (i_fp_issue_tag),
        .i_commit_en  (i_fp_commit_en),
        .i_commit_rd  (i_fp_commit_rd),
        .i_commit_tag (i_fp_commit_tag)
    );

`ifdef FORMAL
    reg        f_past_valid = 1'b0;
    reg        past_rst;
    reg        past_int_issue_en;
    reg [4:0]  past_int_issue_rd;
    reg [2:0]  past_int_issue_tag;
    reg        past_int_commit_en;
    reg [4:0]  past_int_commit_rd;
    reg [2:0]  past_int_commit_tag;
    reg [4:0]  past_int_raddr2;
    reg        past_int_busy2;
    reg [2:0]  past_int_tag2;

    reg        past_fp_issue_en;
    reg [4:0]  past_fp_issue_rd;
    reg [2:0]  past_fp_issue_tag;

    always @(posedge i_clk) begin
        f_past_valid        <= 1'b1;
        past_rst            <= i_rst;
        past_int_issue_en   <= i_int_issue_en;
        past_int_issue_rd   <= i_int_issue_rd;
        past_int_issue_tag  <= i_int_issue_tag;
        past_int_commit_en  <= i_int_commit_en;
        past_int_commit_rd  <= i_int_commit_rd;
        past_int_commit_tag <= i_int_commit_tag;
        past_int_raddr2     <= i_int_raddr2;
        past_int_busy2      <= o_int_busy2;
        past_int_tag2       <= o_int_tag2;
        past_fp_issue_en    <= i_fp_issue_en;
        past_fp_issue_rd    <= i_fp_issue_rd;
        past_fp_issue_tag   <= i_fp_issue_tag;
    end

    wire [2:0] alu_age = i_alu_tag - i_rob_head;
    wire [2:0] fp0_age = i_fp0_tag - i_rob_head;
    wire [2:0] fp1_age = i_fp1_tag - i_rob_head;
    wire [2:0] ld_age  = i_ld_tag  - i_rob_head;

    wire [2:0] gnt_count = {2'b0, o_alu_gnt} + {2'b0, o_fp0_gnt} +
                           {2'b0, o_fp1_gnt} + {2'b0, o_ld_gnt};

    // Combinational CDB arbiter properties
    always @(*) begin
        if (!f_past_valid)
            assume (i_rst);

        // 1. Mutual exclusion: at most one CDB grant
        assert (gnt_count <= 3'd1);

        // 2-5. Grants only when requested
        assert (!o_alu_gnt || i_alu_v);
        assert (!o_fp0_gnt || i_fp0_v);
        assert (!o_fp1_gnt || i_fp1_v);
        assert (!o_ld_gnt  || i_ld_v);

        // 6. Progress: o_cdb_valid iff at least one request
        assert (o_cdb_valid == (i_alu_v || i_fp0_v || i_fp1_v || i_ld_v));
        assert (o_cdb_valid == (gnt_count == 3'd1));

        // 7-8. Output payload matches granted requester
        if (o_alu_gnt) begin
            assert (o_cdb_tag == i_alu_tag);
            assert (o_cdb_rd  == i_alu_rd);
            assert (o_cdb_val == i_alu_val);
            assert (o_cdb_pc  == i_alu_pc);
        end
        if (o_ld_gnt) begin
            assert (o_cdb_tag == i_ld_tag);
            assert (o_cdb_rd  == i_ld_rd);
            assert (o_cdb_val == i_ld_val);
            assert (o_cdb_pc  == i_ld_pc);
        end

        // 9-11. Age-ordered arbitration invariants
        if (i_alu_v && i_ld_v && (alu_age < ld_age))
            assert (!o_ld_gnt);
        if (i_alu_v && i_ld_v && (ld_age < alu_age))
            assert (!o_alu_gnt);
        if (i_fp0_v && i_fp1_v && (fp0_age < fp1_age))
            assert (!o_fp1_gnt);
        if (i_fp0_v && i_fp1_v && (fp1_age < fp0_age))
            assert (!o_fp0_gnt);

        // 12. INT x0 is never renamed
        if (i_int_raddr1 == 5'd0) begin
            assert (!o_int_busy1);
            assert (o_int_tag1 == 3'd0);
        end
    end

    // Sequential RAT properties
    always @(posedge i_clk) begin
        if (f_past_valid) begin
            // 13. Reset clears busy in both INT and FP RATs
            if (past_rst) begin
                assert (!o_int_busy1);
                assert (!o_fp_busy1);
            end

            // 14. INT issue sets busy+tag (takes priority over simultaneous commit)
            if (!past_rst && past_int_issue_en && (past_int_issue_rd != 5'd0) &&
                (i_int_raddr1 == past_int_issue_rd)) begin
                assert (o_int_busy1);
                assert (o_int_tag1 == past_int_issue_tag);
            end

            // 15. FP issue sets busy+tag (including f0!)
            if (!past_rst && past_fp_issue_en && (i_fp_raddr1 == past_fp_issue_rd)) begin
                assert (o_fp_busy1);
                assert (o_fp_tag1 == past_fp_issue_tag);
            end

            // 16. Commit with matching tag clears busy when no simultaneous issue to same rd
            if (!past_rst && past_int_commit_en && (past_int_commit_rd != 5'd0) &&
                !(past_int_issue_en && (past_int_issue_rd == past_int_commit_rd)) &&
                (past_int_raddr2 == past_int_commit_rd) && past_int_busy2 &&
                (past_int_tag2 == past_int_commit_tag) &&
                (i_int_raddr2 == past_int_commit_rd)) begin
                assert (!o_int_busy2);
            end

            // 17. Commit with stale tag preserves younger mapping (WAW safety)
            if (!past_rst && past_int_busy2 &&
                !(past_int_issue_en && (past_int_issue_rd == past_int_raddr2)) &&
                (!past_int_commit_en || (past_int_commit_rd != past_int_raddr2) ||
                 (past_int_tag2 != past_int_commit_tag)) &&
                (i_int_raddr2 == past_int_raddr2)) begin
                assert (o_int_busy2);
                assert (o_int_tag2 == past_int_tag2);
            end
        end
    end
`endif

endmodule

`default_nettype wire
