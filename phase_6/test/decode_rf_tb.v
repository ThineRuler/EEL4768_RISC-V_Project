`timescale 1ns / 1ps
`default_nettype none

module decode_rf_tb;

    reg clk;
    reg rst;

    // RF signals
    reg  [ 4:0] rs1_raddr, rs2_raddr, rd_waddr;
    reg  [31:0] rd_wdata;
    wire [31:0] rs1_rdata, rs2_rdata;

    reg  [ 4:0] frs1_raddr, frs2_raddr, frs3_raddr, frd_waddr;
    reg         frd_wen;
    reg  [31:0] frd_wdata;
    wire [31:0] frs1_rdata, frs2_rdata, frs3_rdata;

    // Decoder signals
    reg  [31:0] inst;
    wire        legal;
    wire        halt;
    wire [ 4:0] dec_rs1, dec_rs2, dec_rd, dec_rs3;
    wire [31:0] dec_imm;
    wire        op1_sel, op2_sel;
    wire [ 2:0] alu_opsel;
    wire        alu_sub, alu_unsigned, alu_arith;
    wire        branch, jump, branch_equal, branch_unsigned, branch_invert;
    wire        dmem_ren, dmem_wen;
    wire [ 1:0] dmem_align;
    wire        dmem_memb, dmem_memh, dmem_memw, dmem_memu;
    wire [ 4:0] rd_sel;
    wire        mext;
    wire        fpu;
    wire        frs1_en, frs2_en, frs3_en, frd_en;
    wire        pc_sel;

    // Instantiate Register File with BYPASS_EN = 1
    rf #(.BYPASS_EN(1)) uut_rf (
        .i_clk         (clk),
        .i_rst         (rst),
        .i_rs1_raddr   (rs1_raddr),
        .o_rs1_rdata   (rs1_rdata),
        .i_rs2_raddr   (rs2_raddr),
        .o_rs2_rdata   (rs2_rdata),
        .i_rd_waddr    (rd_waddr),
        .i_rd_wdata    (rd_wdata),
        .i_frs1_raddr  (frs1_raddr),
        .o_frs1_rdata  (frs1_rdata),
        .i_frs2_raddr  (frs2_raddr),
        .o_frs2_rdata  (frs2_rdata),
        .i_frs3_raddr  (frs3_raddr),
        .o_frs3_rdata  (frs3_rdata),
        .i_frd_wen     (frd_wen),
        .i_frd_waddr   (frd_waddr),
        .i_frd_wdata   (frd_wdata)
    );

    // Instantiate Decoder
    decoder uut_decoder (
        .i_inst             (inst),
        .o_legal            (legal),
        .o_halt             (halt),
        .o_rs1              (dec_rs1),
        .o_rs2              (dec_rs2),
        .o_rd               (dec_rd),
        .o_immediate        (dec_imm),
        .o_op1_sel          (op1_sel),
        .o_op2_sel          (op2_sel),
        .o_alu_opsel        (alu_opsel),
        .o_alu_sub          (alu_sub),
        .o_alu_unsigned     (alu_unsigned),
        .o_alu_arith        (alu_arith),
        .o_branch           (branch),
        .o_jump             (jump),
        .o_branch_equal     (branch_equal),
        .o_branch_unsigned  (branch_unsigned),
        .o_branch_invert    (branch_invert),
        .o_dmem_ren         (dmem_ren),
        .o_dmem_wen         (dmem_wen),
        .o_dmem_align       (dmem_align),
        .o_dmem_memb        (dmem_memb),
        .o_dmem_memh        (dmem_memh),
        .o_dmem_memw        (dmem_memw),
        .o_dmem_memu        (dmem_memu),
        .o_rd_sel           (rd_sel),
        .o_mext             (mext),
        .o_fpu              (fpu),
        .o_frs1_en          (frs1_en),
        .o_frs2_en          (frs2_en),
        .o_frs3_en          (frs3_en),
        .o_rs3              (dec_rs3),
        .o_frd_en           (frd_en),
        .o_pc_sel           (pc_sel)
    );

    always #5 clk = ~clk;

    integer pass_count = 0;
    integer fail_count = 0;

    task check_dec(
        input        exp_legal,
        input        exp_mext,
        input        exp_fpu,
        input        exp_frs1_en,
        input        exp_frs2_en,
        input        exp_frs3_en,
        input        exp_frd_en,
        input [ 4:0] exp_rd_sel,
        input [127:0] test_name
    );
        begin
            #1;
            if (legal === exp_legal &&
                mext === exp_mext &&
                fpu === exp_fpu &&
                frs1_en === exp_frs1_en &&
                frs2_en === exp_frs2_en &&
                frs3_en === exp_frs3_en &&
                frd_en === exp_frd_en &&
                rd_sel === exp_rd_sel) begin
                $display("[PASS] DEC: %0s", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] DEC: %0s -> legal:%b(%b) mext:%b(%b) fpu:%b(%b) frs1:%b(%b) frs2:%b(%b) frs3:%b(%b) frd:%b(%b) rd_sel:%05b(%05b)",
                         test_name, legal, exp_legal, mext, exp_mext, fpu, exp_fpu,
                         frs1_en, exp_frs1_en, frs2_en, exp_frs2_en, frs3_en, exp_frs3_en,
                         frd_en, exp_frd_en, rd_sel, exp_rd_sel);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        clk = 0;
        rst = 1;
        rs1_raddr = 0; rs2_raddr = 0; rd_waddr = 0; rd_wdata = 0;
        frs1_raddr = 0; frs2_raddr = 0; frs3_raddr = 0; frd_waddr = 0; frd_wen = 0; frd_wdata = 0;
        inst = 32'h00000013; // nop

        #20;
        @(posedge clk);
        rst <= 0;
        @(posedge clk);

        $display("==================================================");
        $display("         TESTING REGISTER FILE (rf.v)             ");
        $display("==================================================");

        // 1. Check synchronous reset: all FP registers must be +0.0 (32'h00000000)
        frs1_raddr = 5'd0; frs2_raddr = 5'd15; frs3_raddr = 5'd31;
        #1;
        if (frs1_rdata === 32'h0 && frs2_rdata === 32'h0 && frs3_rdata === 32'h0) begin
            $display("[PASS] RF: FP registers initialized to 0 on reset");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] RF: FP registers not 0 on reset! f0=%h f15=%h f31=%h", frs1_rdata, frs2_rdata, frs3_rdata);
            fail_count = fail_count + 1;
        end

        // 2. Test f0 writability (f0 is an ordinary register, NOT hardwired to zero!)
        @(posedge clk);
        frd_wen   <= 1'b1;
        frd_waddr <= 5'd0;
        frd_wdata <= 32'hC0C80000; // -6.25
        @(posedge clk);
        frd_wen   <= 1'b0;
        #1;
        frs1_raddr = 5'd0;
        #1;
        if (frs1_rdata === 32'hC0C80000) begin
            $display("[PASS] RF: f0 is successfully writable (value: 0x%08h)", frs1_rdata);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] RF: f0 write failed! Expected 0xC0C80000, got 0x%08h", frs1_rdata);
            fail_count = fail_count + 1;
        end

        // 3. Test x0 hardwired to zero (integer x0 must stay 0)
        @(posedge clk);
        rd_waddr <= 5'd0;
        rd_wdata <= 32'hDEADBEEF;
        @(posedge clk);
        rd_waddr <= 5'd0;
        rd_wdata <= 32'd0;
        #1;
        rs1_raddr = 5'd0;
        #1;
        if (rs1_rdata === 32'd0) begin
            $display("[PASS] RF: x0 is hardwired to zero (got 0x00000000)");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] RF: x0 was overwritten! Got 0x%08h", rs1_rdata);
            fail_count = fail_count + 1;
        end

        // 4. Test 3 asynchronous FP read ports simultaneously
        @(posedge clk);
        frd_wen <= 1; frd_waddr <= 5'd1; frd_wdata <= 32'h11111111;
        @(posedge clk);
        frd_wen <= 1; frd_waddr <= 5'd2; frd_wdata <= 32'h22222222;
        @(posedge clk);
        frd_wen <= 1; frd_waddr <= 5'd3; frd_wdata <= 32'h33333333;
        @(posedge clk);
        frd_wen <= 0;
        #1;
        frs1_raddr = 5'd1; frs2_raddr = 5'd2; frs3_raddr = 5'd3;
        #1;
        if (frs1_rdata === 32'h11111111 && frs2_rdata === 32'h22222222 && frs3_rdata === 32'h33333333) begin
            $display("[PASS] RF: 3 independent FP read ports verified");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] RF: 3 read ports mismatch! f1=%h f2=%h f3=%h", frs1_rdata, frs2_rdata, frs3_rdata);
            fail_count = fail_count + 1;
        end

        // 5. Test RF Bypass mode (write-through forwarding in same cycle)
        @(posedge clk);
        frd_wen   <= 1'b1;
        frd_waddr <= 5'd7;
        frd_wdata <= 32'hCAFE1234;
        frs1_raddr <= 5'd7;
        #1; // Same cycle combinational check!
        if (frs1_rdata === 32'hCAFE1234) begin
            $display("[PASS] RF: Same-cycle bypass forwarding verified on f7");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] RF: Same-cycle bypass failed! Expected 0xCAFE1234, got 0x%08h", frs1_rdata);
            fail_count = fail_count + 1;
        end
        @(posedge clk);
        frd_wen <= 1'b0;

        $display("\n==================================================");
        $display("           TESTING DECODER (decoder.v)            ");
        $display("==================================================");

        // RV32M: mul x5, x6, x7 (opcode=0110011, funct3=000, funct7=0000001, rd=5, rs1=6, rs2=7)
        inst = 32'b0000001_00111_00110_000_00101_0110011;
        check_dec(1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 5'b00001, "mul x5, x6, x7 (RV32M)");

        // RV32M: div x10, x11, x12 (opcode=0110011, funct3=100, funct7=0000001)
        inst = 32'b0000001_01100_01011_100_01010_0110011;
        check_dec(1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 5'b00001, "div x10, x11, x12 (RV32M)");

        // Illegal OP funct7: opcode=0110011 with funct7=0000010 -> illegal
        inst = 32'b0000010_00001_00001_000_00001_0110011;
        #1;
        if (legal === 1'b0) begin
            $display("[PASS] DEC: Illegal OP funct7 correctly rejected");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] DEC: Illegal OP funct7 was accepted as legal!");
            fail_count = fail_count + 1;
        end

        // RV32F: flw f1, 12(x2) (opcode=0000111, funct3=010, rd=1, rs1=2)
        inst = {12'd12, 5'd2, 3'b010, 5'd1, 7'b0000111};
        check_dec(1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 5'b01000, "flw f1, 12(x2)");

        // RV32F: fsw f3, 8(x4) (opcode=0100111, funct3=010, rs1=4, rs2=3)
        inst = {7'd0, 5'd3, 5'd4, 3'b010, 5'd8, 7'b0100111};
        check_dec(1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 5'b00000, "fsw f3, 8(x4)");

        // RV32F: fadd.s f3, f1, f2 (opcode=1010011, funct7=0000000, rm=000)
        inst = 32'b0000000_00010_00001_000_00011_1010011;
        check_dec(1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 5'b00000, "fadd.s f3, f1, f2");

        // RV32F: fmadd.s f4, f1, f2, f3 (opcode=1000011, fmt=00, rs3=3)
        // inst: [31:27]=rs3(3), [26:25]=fmt(00), [24:20]=rs2(2), [19:15]=rs1(1), [14:12]=rm(0), [11:7]=rd(4), [6:0]=1000011
        inst = {5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1000011};
        check_dec(1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 5'b00000, "fmadd.s f4, f1, f2, f3 (Fused R4)");
        #1;
        if (dec_rs3 === 5'd3) begin
            $display("[PASS] DEC: fmadd.s rs3 correctly decoded as 5'd3");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] DEC: fmadd.s rs3 mismatch! Expected 3, got %0d", dec_rs3);
            fail_count = fail_count + 1;
        end

        // RV32F Fused with non-zero fmt (fmt=01) -> illegal!
        inst = {5'd3, 2'b01, 5'd2, 5'd1, 3'b000, 5'd4, 7'b1000011};
        #1;
        if (legal === 1'b0) begin
            $display("[PASS] DEC: Fused fmt != 00 rejected as illegal");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] DEC: Fused fmt != 00 accepted as legal!");
            fail_count = fail_count + 1;
        end

        // RV32F: fcvt.w.s x5, f1 (opcode=1010011, funct7=1100000, rs2=00000, rm=000)
        inst = 32'b1100000_00000_00001_000_00101_1010011;
        check_dec(1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 5'b10000, "fcvt.w.s x5, f1 (writes int rd)");

        // RV32F: fmv.x.w x8, f2 (opcode=1010011, funct7=1110000, rs2=00000, rm=000)
        inst = 32'b1110000_00000_00010_000_01000_1010011;
        check_dec(1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 5'b10000, "fmv.x.w x8, f2");

        // RV32F: fmv.w.x f2, x8 (opcode=1010011, funct7=1111000, rs2=00000, rm=000)
        inst = 32'b1111000_00000_01000_000_00010_1010011;
        check_dec(1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 5'b00000, "fmv.w.x f2, x8 (writes FP rd)");

        // Illegal Static Rounding Mode: rm = 3'b101 or 3'b110 -> illegal!
        inst = 32'b0000000_00010_00001_101_00011_1010011; // fadd.s with rm=101
        #1;
        if (legal === 1'b0) begin
            $display("[PASS] DEC: Static rm=101 rejected as illegal");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] DEC: Static rm=101 accepted as legal!");
            fail_count = fail_count + 1;
        end

        // CSR: csrrw x5, fcsr, x6 (SYSTEM opcode 1110011, funct3=001, csr=0x003, rs1=6, rd=5)
        inst = {12'h003, 5'd6, 3'b001, 5'd5, 7'b1110011};
        check_dec(1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 5'b10000, "csrrw x5, fcsr, x6");

        // CSR: csrrs x1, fflags, x0 (SYSTEM, funct3=010, csr=0x001)
        inst = {12'h001, 5'd0, 3'b010, 5'd1, 7'b1110011};
        check_dec(1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 5'b10000, "csrrs x1, fflags, x0");

        // Illegal CSR address: csr=0x300 (mstatus) -> illegal in phase 6!
        inst = {12'h300, 5'd0, 3'b001, 5'd1, 7'b1110011};
        #1;
        if (legal === 1'b0) begin
            $display("[PASS] DEC: Unsupported CSR address 0x300 rejected as illegal");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] DEC: Unsupported CSR address 0x300 accepted as legal!");
            fail_count = fail_count + 1;
        end

        $display("\n==================================================");
        $display("                 SUMMARY REPORT                   ");
        $display("==================================================");
        $display("Total Passed: %0d", pass_count);
        $display("Total Failed: %0d", fail_count);
        if (fail_count == 0) begin
            $display(">>> ALL DECODER & RF TESTS PASSED! <<<");
        end else begin
            $display(">>> SOME TESTS FAILED! CHECK LOGS ABOVE. <<<");
        end
        $finish;
    end

endmodule
`default_nettype wire
