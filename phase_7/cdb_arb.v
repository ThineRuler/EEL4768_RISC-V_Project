`default_nettype none

module cdb_arb (
    input  wire [  2:0] i_rob_head,
    // Producer index: 0 = integer, 1 = load, 2 = FP0, 3 = FP1.
    // Producer 0 occupies the least significant lane of each packed input.
    input  wire [  3:0] i_req,
    input  wire [ 11:0] i_tag,
    input  wire [127:0] i_value,
    input  wire [ 19:0] i_rd,
    input  wire [127:0] i_pc,

    output wire [  3:0] o_grant,
    output wire         o_cdb_valid,
    output wire [  2:0] o_cdb_tag,
    output wire [ 31:0] o_cdb_value,
    output wire [  4:0] o_cdb_rd,
    output wire [ 31:0] o_cdb_pc
);
    wire [2:0] age0, age1, age2, age3;
    wire [2:0] age01, age23;
    wire pick1, pick3, pick23;
    wire valid01, valid23;
    wire [1:0] winner;

    // Three-bit subtraction gives distance from the head modulo eight.
    assign age0 = i_tag[2:0]  - i_rob_head;
    assign age1 = i_tag[5:3]  - i_rob_head;
    assign age2 = i_tag[8:6]  - i_rob_head;
    assign age3 = i_tag[11:9] - i_rob_head;

    assign valid01 = i_req[0] | i_req[1];
    assign valid23 = i_req[2] | i_req[3];
    assign pick1 = i_req[1] && (!i_req[0] || age1 < age0);
    assign pick3 = i_req[3] && (!i_req[2] || age3 < age2);
    assign age01 = pick1 ? age1 : age0;
    assign age23 = pick3 ? age3 : age2;
    assign pick23 = valid23 && (!valid01 || age23 < age01);
    assign winner = pick23 ? {1'b1, pick3} : {1'b0, pick1};

    assign o_cdb_valid = |i_req;
    assign o_grant[0] = o_cdb_valid && winner == 2'd0;
    assign o_grant[1] = o_cdb_valid && winner == 2'd1;
    assign o_grant[2] = o_cdb_valid && winner == 2'd2;
    assign o_grant[3] = o_cdb_valid && winner == 2'd3;

    assign o_cdb_tag = winner == 2'd0 ? i_tag[2:0] :
                       winner == 2'd1 ? i_tag[5:3] :
                       winner == 2'd2 ? i_tag[8:6] : i_tag[11:9];
    assign o_cdb_value = winner == 2'd0 ? i_value[31:0] :
                         winner == 2'd1 ? i_value[63:32] :
                         winner == 2'd2 ? i_value[95:64] : i_value[127:96];
    assign o_cdb_rd = winner == 2'd0 ? i_rd[4:0] :
                      winner == 2'd1 ? i_rd[9:5] :
                      winner == 2'd2 ? i_rd[14:10] : i_rd[19:15];
    assign o_cdb_pc = winner == 2'd0 ? i_pc[31:0] :
                      winner == 2'd1 ? i_pc[63:32] :
                      winner == 2'd2 ? i_pc[95:64] : i_pc[127:96];
endmodule

`default_nettype wire
