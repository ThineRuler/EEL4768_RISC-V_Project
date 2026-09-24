`timescale 1ns/1ps
`default_nettype none

module alu_tb();

    // Signals
    reg clk;
    reg  [2:0]  i_opsel;
    reg         i_sub;
    reg         i_unsigned;
    reg         i_arith;
    reg  [31:0] i_op1;
    reg  [31:0] i_op2;
    wire [31:0] o_result;
    wire        o_eq;
    wire        o_slt;
    reg [31:0] expected;

    
    // Test counters
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;
    integer local_error = 0;
    
    // DUT
    alu dut (
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith),
        .i_op1(i_op1),
        .i_op2(i_op2),
        .o_result(o_result),
        .o_eq(o_eq),
        .o_slt(o_slt)
    );
    
    // Test task
    task run_test(input [2:0] opsel, input sub, input is_unsigned, input arith,
                  input [31:0] op1, input [31:0] op2);
        integer result_match, eq_match, slt_match;
        
        begin
            test_count = test_count + 1;
            i_opsel = opsel;
            i_sub = sub;
            i_unsigned = is_unsigned;
            i_arith = arith;
            i_op1 = op1;
            i_op2 = op2;

            expected = compute_result(opsel, sub, is_unsigned, arith, op1, op2);
            
            @(posedge clk);
            
            result_match = (o_result == expected);
            eq_match = (o_eq == (op1 == op2));
            slt_match = (o_slt == compute_slt(op1, op2, is_unsigned));
            
            if (result_match && eq_match && slt_match) begin
                pass_count = pass_count + 1;
                $display("[PASS] Test %0d: opsel=%b, sub=%b, is_unsigned=%b, arith=%b, op1=%h, op2=%h -> result=%h (exp %h), eq=%b (exp %b), slt=%b (exp %b)",
                        test_count, opsel, sub, is_unsigned, arith, op1, op2, o_result, expected, o_eq, (op1 == op2), o_slt, compute_slt(op1, op2, is_unsigned));
            end else begin
                fail_count = fail_count + 1;
                $display("[FAIL] Test %0d: opsel=%b, sub=%b, is_unsigned=%b, arith=%b, op1=%h, op2=%h",
                        test_count, opsel, sub, is_unsigned, arith, op1, op2);
                if (!result_match)
                    $display("       Result mismatch: got %h, expected %h", o_result, expected);
                if (!eq_match)
                    $display("       EQ mismatch: got %b, expected %b", o_eq, (op1 == op2));
                if (!slt_match)
                    $display("       SLT mismatch: got %b, expected %b", o_slt, compute_slt(op1, op2, is_unsigned));
                local_error = local_error + 1;
            end
        end
    endtask
    
    // Helper function to compute expected results
    function [31:0] compute_result(input [2:0] opsel, input sub, input is_unsigned, input arith,
                                   input [31:0] op1, input [31:0] op2);
        reg [31:0] result;
        reg [4:0]  shamt;
        begin
            shamt = op2[4:0];
            
            case(opsel)
                3'b000: result = sub ? (op1 - op2) : (op1 + op2);
                3'b001: result = op1 << shamt;
                3'b010, 3'b011: result = compute_slt(op1, op2, is_unsigned) ? 32'b1 : 32'b0;
                3'b100: result = op1 ^ op2;
                3'b101: result = (arith) ? ({{32{op1[31]}}, op1} >>> shamt) : (op1 >> shamt);
                3'b110: result = op1 | op2;
                3'b111: result = op1 & op2;
                default: result = 32'bx;
            endcase
            
            compute_result = result;
        end
    endfunction
    
    // Helper function for SLT
    function compute_slt(input [31:0] op1, input [31:0] op2, input is_unsigned);
        reg signed [31:0] signed_op1, signed_op2;
        begin
            signed_op1 = $signed(op1);
            signed_op2 = $signed(op2);
            
            compute_slt = is_unsigned ? ($unsigned(op1) < $unsigned(op2)) : (signed_op1 < signed_op2);
        end
    endfunction

    initial begin
        $display("========== ALU Testbench ==========");
        $display("Starting ALU tests...\n");

        clk = 0;
        
        // ===== ADD Tests =====
        $display("--- ADD Tests ---");
        //      (opsel,  sub, is_unsigned, 
        //                           arith, op1,         op2,          expected_result)
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000000); // 0 + 0 = 0        both zero
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'h00000005, 32'h00000005); // 5 + 5 = 10       both same
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'h0000000A, 32'h00000014); // 10 + 20 = 30     first less than second
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'h00000005, 32'h00000003); // 5 + 3 = 8        second less than first

        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFB, 32'h00000005); // -5 + 5 = 0       negative first operand
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'h0000000F, 32'hFFFFFFFB); // 15 + -5 = 10     negative second operand
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'hFFFFFFEC, 32'hFFFFFFF6); // -20 + -10 = -30  both negative operands, first less
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'hFFFFFFF6, 32'hFFFFFFEC); // -10 + -20 = -30  both negative operands, second less
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFB, 32'hFFFFFFFB); // -5 + -5 = -10    both same negative operands
        
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFF, 32'h00000001); // -1 + 1 = 0       wrap around case
        run_test(3'b000, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'hFFFFFFFF); // 1 + -1 = 0       wrap around case
        if (local_error > 0) begin
            $display("[ADD FAILURE]");
            local_error = 0;
        end
        
        // ===== SUB Tests =====
        $display("\n--- SUB Tests ---");
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'h00000000, 32'h00000000); // 0 - 0 = 0        both zero
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'h00000005, 32'h00000005); // 5 - 5 = 0        both same
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'h0000000A, 32'h00000014); // 10 - 20 = -10    first less than second
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'h00000005, 32'h00000003); // 5 - 3 = 2        second less than first
        
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'hFFFFFFFB, 32'h00000005); // -5 - 5 = -10     negative first operand
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'h0000000F, 32'hFFFFFFFB); // 15 - -5 = 20     negative second operand
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'hFFFFFFEC, 32'hFFFFFFF6); // -20 - -10 = -10  both negative operands, first less
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'hFFFFFFF6, 32'hFFFFFFEC); // -10 - -20 = 10   both negative operands, second less
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'hFFFFFFFB, 32'hFFFFFFFB); // -5 - -5 = 0      both same negative operands

        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'hFFFFFFFF, 32'h00000001); // -1 - 1 = -2      wrap around case
        run_test(3'b000, 1'b1, 1'b0, 1'b0, 32'h00000001, 32'hFFFFFFFF); // 1 - -1 = 2       wrap around case
        if (local_error > 0) begin
            $display("[SUB FAILURE]");
            local_error = 0;
        end
        
        // ===== SLL Tests =====
        $display("\n--- SLL Tests ---");
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000000);
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'h00000005, 32'h00000001);
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'h0000000A, 32'h00000002);
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'h00000003, 32'h00000003);
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFB, 32'h00000001);
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'hFFFFFFF6, 32'h00000002);
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFF, 32'h00000001);
        run_test(3'b001, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'h0000001F);
        if (local_error > 0) begin
            $display("[SLL FAILURE]");
            local_error = 0;
        end
        
        // ===== SLT Tests =====
        $display("\n--- SLT Tests ---"); // signed comparisons
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000000); // 0 < 0 = 0
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'h00000002); // 1 < 2 = 1
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000002, 32'h00000001); // 2 < 1 = 0
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFE, 32'hFFFFFFFF); // -2 < -1 = 1
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFF, 32'hFFFFFFFE); // -1 < -2 = 0
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFF, 32'h00000001); // -1 < 1 = 1
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'hFFFFFFFF); // 1 < -1 = 0
        
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000000); // 0 < 0 = 0
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'h00000002); // 1 < 2 = 1
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000002, 32'h00000001);
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFE, 32'hFFFFFFFF);
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFF, 32'hFFFFFFFE);
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'hFFFFFFFF, 32'h00000001);
        run_test(3'b010, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'hFFFFFFFF);
        if (local_error > 0) begin
            $display("[SLT FAILURE]");
            local_error = 0;
        end
        
        // ===== SLTU Tests =====
        $display("\n--- SLTU Tests ---");
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000000, 32'h00000000);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000001, 32'h00000002);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000002, 32'h00000001);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'hFFFFFFFE, 32'hFFFFFFFF);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'hFFFFFFFF, 32'hFFFFFFFE);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'hFFFFFFFF, 32'h00000001);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000001, 32'hFFFFFFFF);
        
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000000, 32'h00000000);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000001, 32'h00000002);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000002, 32'h00000001);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'hFFFFFFFE, 32'hFFFFFFFF);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'hFFFFFFFF, 32'hFFFFFFFE);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'hFFFFFFFF, 32'h00000001);
        run_test(3'b011, 1'b0, 1'b1, 1'b0, 32'h00000001, 32'hFFFFFFFF);
        if (local_error > 0) begin
            $display("[SLTU FAILURE]");
            local_error = 0;
        end
        
        // ===== XOR Tests =====
        $display("\n--- XOR Tests ---");
        run_test(3'b100, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000000);
        run_test(3'b100, 1'b0, 1'b0, 1'b0, 32'h11111111, 32'h11111111);
        run_test(3'b100, 1'b0, 1'b0, 1'b0, 32'h01010101, 32'h01010101);
        run_test(3'b100, 1'b0, 1'b0, 1'b0, 32'h01010101, 32'h10101010);
        if (local_error > 0) begin
            $display("[XOR FAILURE]");
            local_error = 0;
        end
        
        // ===== SRL Tests =====
        $display("\n--- SRL Tests ---");
        run_test(3'b101, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'h00000000);
        run_test(3'b101, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b0, 32'h00000010, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b0, 32'h00000100, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b0, 32'h00000100, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b0, 32'h80000000, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b0, 32'h80000000, 32'h0000001F);
        if (local_error > 0) begin
            $display("[SRL FAILURE]");
            local_error = 0;
        end
        
        // ===== SRA Tests =====
        $display("\n--- SRA Tests ---");
        run_test(3'b101, 1'b0, 1'b0, 1'b1, 32'h00000001, 32'h00000000);
        run_test(3'b101, 1'b0, 1'b0, 1'b1, 32'h00000001, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b1, 32'h00000010, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b1, 32'h00000100, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b1, 32'h00000100, 32'h00000001);
        run_test(3'b101, 1'b0, 1'b0, 1'b1, 32'h80000000, 32'h00000001); // should be 0xC0000000
        run_test(3'b101, 1'b0, 1'b0, 1'b1, 32'h80000000, 32'h0000001F); // should be 0xFFFFFFFF
        if (local_error > 0) begin
            $display("[SRA FAILURE]");
            local_error = 0;
        end
        
        // ===== OR Tests =====
        $display("\n--- OR Tests ---");
        run_test(3'b110, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000000);
        run_test(3'b110, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'h00000000);
        run_test(3'b110, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000001);
        run_test(3'b110, 1'b0, 1'b0, 1'b0, 32'h01010101, 32'h01010101);
        run_test(3'b110, 1'b0, 1'b0, 1'b0, 32'h10101010, 32'h10101010);
        run_test(3'b110, 1'b0, 1'b0, 1'b0, 32'h10101010, 32'h01010101);
        if (local_error > 0) begin
            $display("[OR FAILURE]");
            local_error = 0;
        end
        
        // ===== AND Tests =====
        $display("\n--- AND Tests ---");
        run_test(3'b111, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000000);
        run_test(3'b111, 1'b0, 1'b0, 1'b0, 32'h00000001, 32'h00000000);
        run_test(3'b111, 1'b0, 1'b0, 1'b0, 32'h00000000, 32'h00000001);
        run_test(3'b111, 1'b0, 1'b0, 1'b0, 32'h01010101, 32'h01010101);
        run_test(3'b111, 1'b0, 1'b0, 1'b0, 32'h10101010, 32'h10101010);
        run_test(3'b111, 1'b0, 1'b0, 1'b0, 32'h10101010, 32'h01010101);
        if (local_error > 0) begin
            $display("[AND FAILURE]");
            local_error = 0;
        end
        
        // Print summary (not visible to students)
        $display("\n========== Test Summary ==========");
        $display("Total tests:  %0d", test_count);
        $display("Passed:       %0d", pass_count);
        $display("Failed:       %0d", fail_count);
        $display("Pass rate:    %0d%%", (pass_count * 100) / test_count);
        
        if (fail_count == 0) begin
            $display("\nYahoo! All tests passed.");
        end else begin
            $display("\nSome tests failed.");
        end
        
        $finish;
    end

    always #5 clk = ~clk;

endmodule

`default_nettype wire