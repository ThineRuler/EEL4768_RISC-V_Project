`default_nettype none

// CDB portion of the Phase 7 harness. Add the team's RAT instance and
// properties to this same module before running the course's riscv_fv.sby.
module riscv_fv (
    input wire         i_clk,
    input wire         i_rst,
    input wire [  2:0] i_rob_head,
    input wire [  3:0] i_req,
    input wire [ 11:0] i_tag,
    input wire [127:0] i_value,
    input wire [ 19:0] i_rd,
    input wire [127:0] i_pc
);
`ifdef FORMAL
    wire [3:0] grant;
    wire valid;
    wire [2:0] tag;
    wire [31:0] value, pc;
    wire [4:0] rd;
    wire [2:0] winner_age;

    cdb_arb dut_cdb (
        .i_rob_head(i_rob_head), .i_req(i_req), .i_tag(i_tag),
        .i_value(i_value), .i_rd(i_rd), .i_pc(i_pc), .o_grant(grant),
        .o_cdb_valid(valid), .o_cdb_tag(tag), .o_cdb_value(value),
        .o_cdb_rd(rd), .o_cdb_pc(pc)
    );

    assign winner_age = tag - i_rob_head;

    always @(*) begin
        assert (valid == (|i_req));
        assert (valid == (|grant));
        assert ((grant & ~i_req) == 4'b0000);
        assert ((grant & (grant - 4'd1)) == 4'b0000);
        assert (i_req != 4'b0001 || grant == 4'b0001);
        assert (i_req != 4'b0010 || grant == 4'b0010);
        assert (i_req != 4'b0100 || grant == 4'b0100);
        assert (i_req != 4'b1000 || grant == 4'b1000);
    end

    genvar producer;
    generate
        for (producer = 0; producer < 4; producer = producer + 1) begin : check_producer
            wire [2:0] request_age;
            assign request_age = i_tag[3*producer +: 3] - i_rob_head;

            always @(*) begin
                assert (!grant[producer] || tag == i_tag[3*producer +: 3]);
                assert (!grant[producer] || value == i_value[32*producer +: 32]);
                assert (!grant[producer] || rd == i_rd[5*producer +: 5]);
                assert (!grant[producer] || pc == i_pc[32*producer +: 32]);
                assert (!i_req[producer] || winner_age <= request_age);
                assert (!(i_req[producer] && request_age == winner_age) ||
                        (|((4'b1111 >> (3 - producer)) & grant)));
            end
        end
    endgenerate
`endif
endmodule

`default_nettype wire
