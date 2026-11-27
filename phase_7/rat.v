`default_nettype none

// 32-entry Register Alias Table (RAT) mapping architectural registers to ROB tags.
// When ZERO_REG == 1 (integer RF), register 0 (x0) is never renamed.
// When ZERO_REG == 0 (FP RF), register 0 (f0) is renamed normally.
module rat #(
    parameter TAG_W    = 3,
    parameter ZERO_REG = 1
) (
    input  wire             i_clk,
    input  wire             i_rst,

    input  wire [4:0]       i_raddr1,
    output wire             o_busy1,
    output wire [TAG_W-1:0] o_tag1,

    input  wire [4:0]       i_raddr2,
    output wire             o_busy2,
    output wire [TAG_W-1:0] o_tag2,

    input  wire [4:0]       i_raddr3,
    output wire             o_busy3,
    output wire [TAG_W-1:0] o_tag3,

    input  wire             i_issue_en,
    input  wire [4:0]       i_issue_rd,
    input  wire [TAG_W-1:0] i_issue_tag,

    input  wire             i_commit_en,
    input  wire [4:0]       i_commit_rd,
    input  wire [TAG_W-1:0] i_commit_tag
);

    reg [31:0]      busy;
    reg [TAG_W-1:0] tags [0:31];

    assign o_busy1 = (ZERO_REG && (i_raddr1 == 5'd0)) ? 1'b0 : busy[i_raddr1];
    assign o_tag1  = (ZERO_REG && (i_raddr1 == 5'd0)) ? {TAG_W{1'b0}} : tags[i_raddr1];

    assign o_busy2 = (ZERO_REG && (i_raddr2 == 5'd0)) ? 1'b0 : busy[i_raddr2];
    assign o_tag2  = (ZERO_REG && (i_raddr2 == 5'd0)) ? {TAG_W{1'b0}} : tags[i_raddr2];

    assign o_busy3 = (ZERO_REG && (i_raddr3 == 5'd0)) ? 1'b0 : busy[i_raddr3];
    assign o_tag3  = (ZERO_REG && (i_raddr3 == 5'd0)) ? {TAG_W{1'b0}} : tags[i_raddr3];

    wire issue_valid  = i_issue_en  && !(ZERO_REG && (i_issue_rd  == 5'd0));
    wire commit_valid = i_commit_en && !(ZERO_REG && (i_commit_rd == 5'd0));

    genvar r;
    generate
        for (r = 0; r < 32; r = r + 1) begin : gen_rat_entry
            localparam [4:0] REG_IDX = r;
            always @(posedge i_clk) begin
                if (i_rst) begin
                    busy[r] <= 1'b0;
                    tags[r] <= {TAG_W{1'b0}};
                end else if (ZERO_REG && (r == 0)) begin
                    busy[r] <= 1'b0;
                    tags[r] <= {TAG_W{1'b0}};
                end else if (issue_valid && (i_issue_rd == REG_IDX)) begin
                    // Younger issue always overrides any simultaneous commit to the same rd.
                    busy[r] <= 1'b1;
                    tags[r] <= i_issue_tag;
                end else if (commit_valid && (i_commit_rd == REG_IDX) &&
                             busy[r] && (tags[r] == i_commit_tag)) begin
                    // Only clear if no younger in-flight instruction has overwritten the mapping.
                    busy[r] <= 1'b0;
                end
            end
        end
    endgenerate

endmodule

`default_nettype wire
