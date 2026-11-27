`timescale 1ns / 1ps
`default_nettype none

module alu_cdb_tb;
    reg clk = 0;
    always #5 clk = ~clk;

    reg rst = 1;
    reg [2:0] head = 6;
    reg issue = 0;
    reg [2:0] issue_tag = 0, q1 = 7, q2 = 7, opsel = 0;
    reg [31:0] s1 = 0, s2 = 0, imm = 0;
    reg ready1 = 1, ready2 = 1;
    reg jump = 0, jalr = 0, branch = 0, mext = 0;
    reg load_valid = 0;
    wire issue_ready, int_valid, cdb_valid, resolved, redirect;
    wire [2:0] int_tag, cdb_tag, ctrl_tag;
    wire [4:0] int_rd, cdb_rd;
    wire [31:0] int_value, int_pc, cdb_value, cdb_pc, next_pc;
    wire [3:0] grant;

    alu_rs rs (
        .i_clk(clk), .i_rst(rst), .i_rob_head(head),
        .i_issue_valid(issue), .o_issue_ready(issue_ready),
        .i_issue_tag(issue_tag), .i_issue_rd(5'd9), .i_issue_pc(32'h00400000),
        .i_issue_imm(imm), .i_src1_ready(ready1), .i_src1_tag(q1), .i_src1_value(s1),
        .i_src2_ready(ready2), .i_src2_tag(q2), .i_src2_value(s2),
        .i_alu_opsel(opsel), .i_alu_sub(1'b0), .i_alu_unsigned(1'b0),
        .i_alu_arith(1'b0), .i_mext(mext), .i_op1_pc(1'b0), .i_op2_imm(jalr),
        .i_lui(1'b0), .i_branch(branch), .i_branch_equal(1'b1),
        .i_branch_invert(1'b0), .i_jump(jump), .i_jalr(jalr),
        .i_cdb_valid(cdb_valid), .i_cdb_tag(cdb_tag), .i_cdb_value(cdb_value),
        .i_result_grant(grant[0]), .o_result_valid(int_valid), .o_result_tag(int_tag),
        .o_result_rd(int_rd), .o_result_pc(int_pc), .o_result_value(int_value),
        .o_ctrl_resolved(resolved), .o_redirect_valid(redirect), .o_ctrl_tag(ctrl_tag),
        .o_ctrl_next_pc(next_pc)
    );

    cdb_arb arb (
        .i_rob_head(head), .i_req({2'b00, load_valid, int_valid}),
        .i_tag({6'd0, 3'd7, int_tag}), .i_value({64'd0, 32'd42, int_value}),
        .i_rd({10'd0, 5'd10, int_rd}), .i_pc({64'd0, 32'h003ffffc, int_pc}),
        .o_grant(grant), .o_cdb_valid(cdb_valid), .o_cdb_tag(cdb_tag),
        .o_cdb_value(cdb_value), .o_cdb_rd(cdb_rd), .o_cdb_pc(cdb_pc)
    );

    // Drive inputs between edges and allow combinational outputs to settle.
    task tick;
        begin @(posedge clk); #1; end
    endtask

    task expect_result;
        input [2:0] expected_tag;
        input [31:0] expected_value;
        input [4:0] expected_rd;
        input [31:0] expected_pc;
        begin
            if ({cdb_valid, cdb_tag, cdb_value, cdb_rd, cdb_pc} !==
                {1'b1, expected_tag, expected_value, expected_rd, expected_pc})
                $fatal(1, "expected tag %d value %h rd %d PC %h, got %b %d %h %d %h",
                    expected_tag, expected_value, expected_rd, expected_pc,
                    cdb_valid, cdb_tag, cdb_value, cdb_rd, cdb_pc);
        end
    endtask

    initial begin
        tick;
        rst = 0;
        // Older ADD waits on both inputs. A younger independent ADD passes it.
        issue = 1; issue_tag = 0; ready1 = 0; ready2 = 0;
        tick;
        issue_tag = 1; ready1 = 1; ready2 = 1; s1 = 3; s2 = 9;
        tick;
        if (int_valid !== 1'b0) $fatal(1, "ALU completed in its issue cycle");
        issue = 0;
        tick;
        expect_result(1, 12, 9, 32'h00400000);

        // The older load wins and wakes both operands of tag 0. Tag 2 issues
        // on that same edge, so its dependency must be bypassed at allocation.
        load_valid = 1; issue = 1; issue_tag = 2; ready1 = 0; s2 = 10;
        #1;
        expect_result(7, 42, 10, 32'h003ffffc);
        tick;
        if ({int_valid, int_tag, int_value} !== {1'b1, 3'd1, 32'd12})
            $fatal(1, "ALU lost its pending result while the load was granted");
        issue = 0; load_valid = 0;
        #1;
        expect_result(1, 12, 9, 32'h00400000);
        tick;
        expect_result(0, 84, 9, 32'h00400000);
        tick;
        expect_result(2, 52, 9, 32'h00400000);
        tick;
        if (cdb_valid !== 1'b0) $fatal(1, "ALU broadcast was repeated");

        // The redirect resolves a cycle ahead of the jump's registered link.
        issue = 1; issue_tag = 3; ready1 = 1; ready2 = 0;
        s1 = 32'h1001; imm = 2; jump = 1; jalr = 1;
        tick;
        if ({resolved, redirect, ctrl_tag, next_pc, cdb_valid} !==
            {1'b1, 1'b1, 3'd3, 32'h1002, 1'b0}) $fatal(1, "bad JALR resolve");
        issue = 0;
        tick;
        expect_result(3, 32'h00400004, 9, 32'h00400000);
        tick;

        // A not-taken branch resolves and completes even without a redirect.
        issue = 1; issue_tag = 4; ready2 = 1; s1 = 1; s2 = 2;
        jump = 0; jalr = 0; branch = 1;
        tick;
        if ({resolved, redirect, ctrl_tag, next_pc} !== {1'b1, 1'b0, 3'd4, 32'h00400004})
            $fatal(1, "bad not-taken branch resolve");
        issue = 0;
        tick;
        expect_result(4, 0, 0, 32'h00400000);
        tick;

        // The Phase 6 divider keeps ownership until its result is ready.
        issue = 1; issue_tag = 5; branch = 0; mext = 1; opsel = 3'b100;
        s1 = 20; s2 = 3;
        tick;
        issue = 0;
        repeat (33) begin
            if (int_valid !== 1'b0) $fatal(1, "divider completed early");
            tick;
        end
        if (int_valid !== 1'b0) $fatal(1, "divider result was not registered");
        tick;
        expect_result(5, 6, 9, 32'h00400000);
        tick;
        if (cdb_valid !== 1'b0) $fatal(1, "divider broadcast was repeated");

        // Fill all four stations, then issue into the slot released by dispatch.
        issue = 1; issue_tag = 0; ready1 = 0; ready2 = 1;
        mext = 0; opsel = 0; s2 = 1;
        repeat (4) begin
            tick;
            issue_tag = issue_tag + 3'd1;
        end
        if (issue_ready !== 1'b0) $fatal(1, "full RS accepted a fifth entry");
        issue = 0; load_valid = 1;
        tick;
        if (issue_ready !== 1'b1) $fatal(1, "dispatch did not make room for issue");
        issue = 1; load_valid = 0; ready1 = 1; s1 = 8;
        tick;
        issue = 0;
        expect_result(0, 43, 9, 32'h00400000);
        tick;
        expect_result(1, 43, 9, 32'h00400000);
        tick;
        expect_result(2, 43, 9, 32'h00400000);
        tick;
        expect_result(3, 43, 9, 32'h00400000);
        tick;
        expect_result(4, 9, 9, 32'h00400000);
        tick;
        if (cdb_valid !== 1'b0) $fatal(1, "RS refill repeated a result");

        $display("ALU RS and CDB directed tests passed");
        $finish;
    end
endmodule

`default_nettype wire
