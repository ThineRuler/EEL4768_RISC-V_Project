`timescale 1ns / 1ps
`default_nettype none

module prefetch_tb;

    localparam integer BLOCK_SIZE = 128;

    reg clk;
    reg rst;
    reg ren, wen;
    reg [31:0] addr;
    reg [3:0] mask;
    reg [31:0] wdata;

    wire ready;
    wire [31:0] rdata;
    wire mem_rd;
    wire [31:0] mem_rd_addr;
    reg mem_valid;
    reg [BLOCK_SIZE*8-1:0] mem_rd_data;
    wire mem_wr;
    wire [31:0] mem_wr_addr;
    wire [BLOCK_SIZE*8-1:0] mem_wr_data;
    wire [BLOCK_SIZE-1:0] mem_wr_be;

    cache #(
        .EVICT_POLICY(0),
        .NUM_WAYS(4),
        .CACHE_SIZE(16),
        .BLOCK_SIZE(BLOCK_SIZE),
        .PREFETCH_EN(1)
    ) dut (
        .i_clk(clk),
        .i_rst(rst),
        .i_ren(ren),
        .i_wen(wen),
        .i_addr(addr),
        .i_mask(mask),
        .i_wdata(wdata),
        .o_ready(ready),
        .o_rdata(rdata),
        .o_mem_rd(mem_rd),
        .o_mem_rd_addr(mem_rd_addr),
        .i_mem_valid(mem_valid),
        .i_mem_rd_data(mem_rd_data),
        .o_mem_wr(mem_wr),
        .o_mem_wr_addr(mem_wr_addr),
        .o_mem_wr_data(mem_wr_data),
        .o_mem_wr_be(mem_wr_be)
    );

    reg [7:0] memory [0:65535];

    // Pipelined main-memory model. Responses return in request order.
    reg [31:0] q_addr [0:7];
    integer q_due [0:7];
    integer q_head, q_tail, q_count;
    integer cycle_count;
    integer i, b;

    integer pass_count, fail_count;
    integer rd_count;
    reg [31:0] rd0_addr, rd1_addr, rd2_addr;

    always #5 clk = ~clk;

    task check;
        input condition;
        input [511:0] msg;
        begin
            if (condition) begin
                pass_count = pass_count + 1;
                $display("[PASS] %0s", msg);
            end else begin
                fail_count = fail_count + 1;
                $display("[FAIL] %0s", msg);
            end
        end
    endtask

    always @(posedge clk) begin
        if (rst) begin
            mem_valid <= 1'b0;
            mem_rd_data <= {(BLOCK_SIZE*8){1'b0}};
            q_head <= 0;
            q_tail <= 0;
            q_count <= 0;
            cycle_count <= 0;
        end else begin
            cycle_count <= cycle_count + 1;
            mem_valid <= 1'b0;

            // Return the oldest outstanding request when its latency expires.
            if (q_count > 0 && q_due[q_head] <= cycle_count) begin
                mem_valid <= 1'b1;
                for (b = 0; b < BLOCK_SIZE; b = b + 1)
                    mem_rd_data[b*8 +: 8] <=
                        memory[(q_addr[q_head] + b) & 16'hFFFF];

                q_head <= (q_head + 1) & 7;
                q_count <= q_count - 1;
            end

            // Accept a new read every cycle.
            if (mem_rd) begin
            $display("[MEM READ] cycle=%0d address=%h", cycle_count, mem_rd_addr);
            
                q_addr[q_tail] <= mem_rd_addr;
                q_due[q_tail] <= cycle_count + 4;
                q_tail <= (q_tail + 1) & 7;
                q_count <= q_count + 1;

                if (rd_count == 0)
                    rd0_addr = mem_rd_addr;
                else if (rd_count == 1)
                    rd1_addr = mem_rd_addr;
                else if (rd_count == 2)
                    rd2_addr = mem_rd_addr;

                rd_count = rd_count + 1;
            end
        end
    end

    task do_read;
        input [31:0] a;
        input [31:0] expected;
        input [511:0] msg;
        integer guard;
        begin
            @(negedge clk);
            ren = 1'b1;
            wen = 1'b0;
            addr = a;
            mask = 4'b0000;
            wdata = 32'd0;

            guard = 0;
            @(posedge clk);
            while (!ready && guard < 100) begin
                guard = guard + 1;
                @(posedge clk);
            end

            #1;
            check(guard < 100 && rdata === expected, msg);

            @(negedge clk);
            ren = 1'b0;
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        ren = 1'b0;
        wen = 1'b0;
        addr = 32'd0;
        mask = 4'b0000;
        wdata = 32'd0;
        mem_valid = 1'b0;
        mem_rd_data = {(BLOCK_SIZE*8){1'b0}};

        pass_count = 0;
        fail_count = 0;
        rd_count = 0;
        rd0_addr = 0;
        rd1_addr = 0;
        rd2_addr = 0;
        q_head = 0;
        q_tail = 0;
        q_count = 0;
        cycle_count = 0;

        for (i = 0; i < 65536; i = i + 1)
            memory[i] = 8'h00;

        // Demand line 0x1000 -> 0x11223344
        memory[16'h1000] = 8'h44;
        memory[16'h1001] = 8'h33;
        memory[16'h1002] = 8'h22;
        memory[16'h1003] = 8'h11;

        // Next line 0x1080 -> 0xAABBCCDD
        memory[16'h1080] = 8'hDD;
        memory[16'h1081] = 8'hCC;
        memory[16'h1082] = 8'hBB;
        memory[16'h1083] = 8'hAA;

        repeat (2) @(posedge clk);
        rst = 1'b0;

        do_read(32'h00001000, 32'h11223344,
                "Initial load returns data from demand line");

        do_read(32'h00001080, 32'hAABBCCDD,
                "Next-line load hits the prefetched line");

        check(rd_count == 2,
              "Exactly two memory reads occurred: demand + prefetch");
        check(rd0_addr == 32'h00001000,
              "First memory request was the demand line 0x1000");
        check(rd1_addr == 32'h00001080,
              "Second memory request was the next line 0x1080");

        $display("==============================================");
        $display("Prefetch test: %0d passed, %0d failed",
                 pass_count, fail_count);
        $display("==============================================");

        if (fail_count == 0)
            $display("ALL PREFETCH BASIC TESTS PASSED");
        else
            $display("PREFETCH BASIC TEST FAILED");

        #20;
        $finish;
    end

endmodule

`default_nettype wire
