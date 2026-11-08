`timescale 1ns / 1ps
`default_nettype none

module prefetch_dirty_victim_tb;

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
        .NUM_WAYS(1),
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

    reg [31:0] q_addr [0:7];
    integer q_due [0:7];
    integer q_is_write [0:7];
    reg [BLOCK_SIZE*8-1:0] q_wdata [0:7];

    integer q_head, q_tail, q_count;
    integer cycle_count;
    integer i, b;

    integer pass_count, fail_count;
    integer rd_count, wr_count;

    reg [31:0] rd0_addr, rd1_addr, rd2_addr;
    reg [31:0] wr0_addr, wr1_addr;
    reg saw_prefetch_read;
    reg saw_writeback;

    always #5 clk = ~clk;

    task check;
        input condition;
        input [511:0] msg;
        begin
            if (condition) begin
                pass_count = pass_count + 1;
                $display("[PASS] %0s", msg);
            end
            else begin
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
        end
        else begin
            cycle_count <= cycle_count + 1;
            mem_valid <= 1'b0;

            // Return the oldest outstanding READ.
            if (q_count > 0 && q_due[q_head] <= cycle_count &&
                !q_is_write[q_head]) begin

                mem_valid <= 1'b1;

                for (b = 0; b < BLOCK_SIZE; b = b + 1)
                    mem_rd_data[b*8 +: 8] <=
                        memory[(q_addr[q_head] + b) & 16'hFFFF];

                q_head <= (q_head + 1) & 7;
                q_count <= q_count - 1;
            end

            // Accept a memory request.
            if (mem_rd) begin
                $display("[MEM READ] cycle=%0d address=%h",
                         cycle_count, mem_rd_addr);

                q_addr[q_tail] <= mem_rd_addr;
                q_due[q_tail] <= cycle_count + 3;
                q_is_write[q_tail] <= 1'b0;
                q_wdata[q_tail] <= {(BLOCK_SIZE*8){1'b0}};

                q_tail <= (q_tail + 1) & 7;
                q_count <= q_count + 1;

                if (rd_count == 0)
                    rd0_addr = mem_rd_addr;
                else if (rd_count == 1)
                    rd1_addr = mem_rd_addr;
                else if (rd_count == 2)
                    rd2_addr = mem_rd_addr;

                rd_count = rd_count + 1;

                if (mem_rd_addr == 32'h00000180)
                    saw_prefetch_read = 1'b1;
            end

            if (mem_wr) begin
                $display("[MEM WRITE] cycle=%0d address=%h",
                         cycle_count, mem_wr_addr);

                saw_writeback = 1'b1;

                if (wr_count == 0)
                    wr0_addr = mem_wr_addr;
                else if (wr_count == 1)
                    wr1_addr = mem_wr_addr;

                wr_count = wr_count + 1;

                // Write the cache block back into main memory.
                for (b = 0; b < BLOCK_SIZE; b = b + 1)
                    if (mem_wr_be[b])
                        memory[(mem_wr_addr + b) & 16'hFFFF] =
                            mem_wr_data[b*8 +: 8];
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

    task do_write;
        input [31:0] a;
        input [31:0] data;
        input [511:0] msg;
        integer guard;
        begin
            @(negedge clk);
            ren = 1'b0;
            wen = 1'b1;
            addr = a;
            mask = 4'b1111;
            wdata = data;

            guard = 0;
            @(posedge clk);

            while (!ready && guard < 100) begin
                guard = guard + 1;
                @(posedge clk);
            end

            #1;
            check(guard < 100, msg);

            @(negedge clk);
            wen = 1'b0;
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
        wr_count = 0;

        rd0_addr = 0;
        rd1_addr = 0;
        rd2_addr = 0;
        wr0_addr = 0;
        wr1_addr = 0;

        saw_prefetch_read = 1'b0;
        saw_writeback = 1'b0;

        q_head = 0;
        q_tail = 0;
        q_count = 0;
        cycle_count = 0;

        for (i = 0; i < 65536; i = i + 1)
            memory[i] = 8'h00;

        // Line 0x0100 contains the original demand data.
        memory[16'h0100] = 8'h44;
        memory[16'h0101] = 8'h33;
        memory[16'h0102] = 8'h22;
        memory[16'h0103] = 8'h11;

        // Line 0x0180 is the next line and therefore the prefetch target.
        memory[16'h0180] = 8'hDD;
        memory[16'h0181] = 8'hCC;
        memory[16'h0182] = 8'hBB;
        memory[16'h0183] = 8'hAA;

        // 0x4180 maps to the SAME cache set as 0x0180,
        // but has a different tag. It will be the dirty victim.
        memory[16'h4180] = 8'hEF;
        memory[16'h4181] = 8'hBE;
        memory[16'h4182] = 8'hAD;
        memory[16'h4183] = 8'hDE;

        repeat (2) @(posedge clk);
        rst = 1'b0;

        // Fill line 0x4180.
        do_read(32'h00004180, 32'hDEADBEEF,
                "Initial fill of victim line succeeds");

        // Make the 0x0900 cache line dirty.
        do_write(32'h00004180, 32'hCAFEBABE,
                 "Write makes the victim line dirty");

        // Access 0x0100. Its next line is 0x0180.
        // The prefetch should eventually install 0x0180.
        do_read(32'h00000100, 32'h11223344,
                "Demand load succeeds while starting the prefetch");

        // The important checks:
        // The dirty victim 0x0900 must have been written back.
        check(saw_writeback,
              "Dirty victim generated a memory writeback");

        check(wr0_addr == 32'h00004180,
              "Writeback address is the dirty victim line 0x4180");

        // The prefetched line must have been requested.
        check(saw_prefetch_read,
              "Next-line prefetch request for 0x0180 was generated");

        // Now verify the original dirty data really reached memory.
        check(memory[16'h4180] == 8'hBE &&
              memory[16'h4181] == 8'hBA &&
              memory[16'h4182] == 8'hFE &&
              memory[16'h4183] == 8'hCA,
              "Dirty victim data was written back correctly");

        $display("==============================================");
        $display("Dirty-victim prefetch test: %0d passed, %0d failed",
                 pass_count, fail_count);
        $display("==============================================");

        if (fail_count == 0)
            $display("ALL DIRTY-VICTIM PREFETCH TESTS PASSED");
        else
            $display("DIRTY-VICTIM PREFETCH TEST FAILED");

        #30;
        $finish;
    end

endmodule

`default_nettype wire
