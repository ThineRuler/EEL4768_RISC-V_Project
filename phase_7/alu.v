//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/08/2026 11:18:34 AM
// Design Name: 
// Module Name: ALU_RISC_V
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`default_nettype none

module alu (
    input  wire        i_clk,
    input  wire        i_rst,

    input  wire [ 2:0] i_opsel,
    input  wire        i_sub,
    input  wire        i_unsigned,
    input  wire        i_arith,
    input  wire        i_mext,
    input  wire        i_advance,

    input  wire [31:0] i_op1,
    input  wire [31:0] i_op2,

    output wire [31:0] o_result,
    output wire        o_eq,
    output wire        o_slt,
    output wire        o_busy
);
//*********************************
//  32 BIT  2'S COMPLEMENT CLA //
//**********************************

wire [31: 0] P; //PROPOGATE
wire [31: 0] G; //GENERATE
wire [32: 0] C; //CARRY
wire [31: 0] SUM; //SUM
wire [31: 0] B_modifi; //B CHANGES TO 2'S COMPLEMENTS FOR SUBTRACTION AND ADDITION


assign P = i_op1 ^ B_modifi; //Propgate signal
assign G = i_op1 & B_modifi; //Generate signal

//2'S COMPLEMENT
assign B_modifi = i_op2 ^ {32{i_sub}};


//FIRST 4 BITS (0-3)
assign C[0] = i_sub;  // CARRY IN
assign C[1] = G[0] | (P[0] & C[0]); //CARRY 1
assign C[2] = G[1] | (P[1] & G[0]) | (P[1] & P[0] & C[0]); // CARRY 2
assign C[3] = G[2] | (P[2] & G[1]) |(P[2] & P[1] & G[0]) | (P[2] & P[1] & P[0] & C[0]); // CARRY 3
assign C[4] = G[3] | (P[3] & G[2]) | (P[3] &P[2] & G[1])| (P[3] & P[2] & P[1] & G[0]) | (P[3] & P[2] & P[1] & P[0] & C[0]); // CARRY 3

//ADD / SUB
assign SUM[0] = P[0] ^ C[0];
assign SUM[1] = P[1] ^ C[1];
assign SUM[2] = P[2] ^ C[2];
assign SUM[3] = P[3] ^ C[3];



//NEXT 4 BITS = (4-7)
assign C[5] = G[4] | (P[4] & C[4]);  // CARRY 1
assign C[6] = G[5] | (P[5] & G[4]) | (P[5] & P[4] & C[4]); // CARRY 2
assign C[7] = G[6] | (P[6] & G[5]) |(P[6] & P[5] & G[4]) | (P[6] & P[5] & P[4] & C[4]); // CARRY 3
assign C[8] = G[7] | (P[7] & G[6]) | (P[7] & P[6] & G[5])| (P[7] & P[6] & P[5] & G[4]) | (P[7] & P[6] & P[5] & P[4] & C[4]); // CARRY 4

//ADD / SUB
assign SUM[4] = P[4] ^ C[4];
assign SUM[5] = P[5] ^ C[5];
assign SUM[6] = P[6] ^ C[6];
assign SUM[7] = P[7] ^ C[7];


//NEXT 4 BITS = (8-11)
assign C[9] = G[8] | (P[8] & C[8]);  // CARRY 1
assign C[10] = G[9] | (P[9] & G[8]) | (P[9] & P[8] & C[8]); // CARRY 2
assign C[11] = G[10] | (P[10] & G[9]) |(P[10] & P[9] & G[8]) | (P[10] & P[9] & P[8] & C[8]); // CARRY 3
assign C[12] = G[11] | (P[11] & G[10]) | (P[11] & P[10] & G[9])| (P[11] & P[10] & P[9] & G[8]) | (P[11] & P[10]  & P[9] & P[8] & C[8]); // CARRY 4

//ADD / SUB
assign SUM[8] = P[8] ^ C[8];
assign SUM[9] = P[9] ^ C[9];
assign SUM[10] = P[10] ^ C[10];
assign SUM[11] = P[11] ^ C[11];


//NEXT 4 BITS = (12-15)
assign C[13] = G[12] | (P[12] & C[12]);  // CARRY 1
assign C[14] = G[13] | (P[13] & G[12]) | (P[13] & P[12] & C[12]); // CARRY 2
assign C[15] = G[14] | (P[14] & G[13]) |(P[14] & P[13] & G[12]) | (P[14] & P[13] & P[12] & C[12]); // CARRY 3
assign C[16] = G[15] | (P[15] & G[14]) | (P[15] & P[14] & G[13])| (P[15] & P[14] & P[13] & G[12]) |(P[15] & P[14] & P[13] & P[12] & C[12]); // CARRY 4

//ADD / SUB
assign SUM[12] = P[12] ^ C[12];
assign SUM[13] = P[13] ^ C[13];
assign SUM[14] = P[14] ^ C[14];
assign SUM[15] = P[15] ^ C[15];


//NEXT 4 BITS = (16-19)
assign C[17] = G[16] | (P[16] & C[16]);  // CARRY 1
assign C[18] = G[17] | (P[17] & G[16]) | (P[17] & P[16] & C[16]); // CARRY 2
assign C[19] = G[18] | (P[18] & G[17]) |(P[18] & P[17] & G[16]) | (P[18] & P[17] & P[16] & C[16]); // CARRY 3
assign C[20] = G[19] | (P[19] & G[18]) | (P[19] & P[18] & G[17])| (P[19] & P[18] & P[17] & G[16]) | (P[19] & P[18] & P[17] & P[16] & C[16]); // CARRY 3

//ADD / SUB
assign SUM[16] = P[16] ^ C[16];
assign SUM[17] = P[17] ^ C[17];
assign SUM[18] = P[18] ^ C[18];
assign SUM[19] = P[19] ^ C[19];


//NEXT 4 BITS = (20-23)
assign C[21] = G[20] | (P[20] & C[20]);  // CARRY 1
assign C[22] = G[21] | (P[21] & G[20]) | (P[21] & P[20] & C[20]); // CARRY 2
assign C[23] = G[22] | (P[22] & G[21]) |(P[22] & P[21] & G[20]) | (P[22] & P[21] & P[20] & C[20]); // CARRY 3
assign C[24] = G[23] | (P[23] & G[22]) | (P[23] & P[22] & G[21])| (P[23] & P[22] & P[21] & G[20]) | (P[23] & P[22] & P[21] & P[20] & C[20]); // CARRY 4

//ADD / SUB
assign SUM[20] = P[20] ^ C[20];
assign SUM[21] = P[21] ^ C[21];
assign SUM[22] = P[22] ^ C[22];
assign SUM[23] = P[23] ^ C[23];


//NEXT 4 BITS = (24-27)
assign C[25] = G[24] | (P[24] & C[24]);  // CARRY 1
assign C[26] = G[25] | (P[25] & G[24]) | (P[25] & P[24] & C[24]); // CARRY 2
assign C[27] = G[26] | (P[26] & G[25]) |(P[26] & P[25] & G[24]) | (P[26] & P[25] & P[24] & C[24]); // CARRY 3
assign C[28] = G[27] | (P[27] & G[26]) | (P[27] & P[26] & G[25])| (P[27] & P[26] & P[25] & G[24]) | (P[27] & P[26] & P[25] & P[24] & C[24]); // CARRY 4

//ADD / SUB
assign SUM[24] = P[24] ^ C[24];
assign SUM[25] = P[25] ^ C[25];
assign SUM[26] = P[26] ^ C[26];
assign SUM[27] = P[27] ^ C[27];

//NEXT 4 BITS = (28-31)
assign C[29] = G[28] | (P[28] & C[28]);  // CARRY 1
assign C[30] = G[29] | (P[29] & G[28]) | (P[29] & P[28] & C[28]); // CARRY 2
assign C[31] = G[30] | (P[30] & G[29]) |(P[30] & P[29] & G[28]) | (P[30] & P[29] & P[28] & C[28]); // CARRY 3
assign C[32] = G[31] | (P[31] & G[30]) | (P[31] & P[30] & G[29])| (P[31] & P[30] & P[29] & G[28]) | (P[31] & P[30] & P[29] & P[28] & C[28]); // CARRY 3

//ADD / SUB
assign SUM[28] = P[28] ^ C[28];
assign SUM[29] = P[29] ^ C[29];
assign SUM[30] = P[30] ^ C[30];
assign SUM[31] = P[31] ^ C[31];


//*********************************
//SLL, SRL, SRA - BARREL SHIFTER - SHIFT AMOUNTS ARE 5 BITS SO 5 STAGES PER SHIFT INSTRUCTION
//*********************************

//SLL

//shift 1 - 1 bit
wire [31:0] SL_1;
assign SL_1 = i_op2[0] ?  {i_op1[30:0], 1'b0} : i_op1;

//shift 2 - 2 bit
wire [31:0] SL_2;
assign SL_2 = i_op2[1] ?  {SL_1[29:0], 2'b00} : SL_1;

//shift 3 - 4 bit
wire [31:0] SL_3;
assign SL_3 = i_op2[2] ?  {SL_2[27:0], 4'b0000} : SL_2;

//shift 4 - 8 bit
wire [31:0] SL_4;
assign SL_4 = i_op2[3] ?  {SL_3[23:0], 8'b00000000} : SL_3;

//shift 5 - 16 bit
wire [31:0] SL_5;
assign SL_5 = i_op2[4] ?  {SL_4[15:0], 16'b0000000000000000} : SL_4;


//SRL

//shift 1 - 1 bit
wire [31:0] SR_1;
assign SR_1 = i_op2[0] ?  { 1'b0, i_op1[31:1]} : i_op1;

//shift 2 - 2 bit
wire [31:0] SR_2;
assign SR_2 = i_op2[1] ?  {2'b00, SR_1[31:2]} : SR_1;

//shift 3 - 4 bit
wire [31:0] SR_3;
assign SR_3 = i_op2[2] ?  { 4'b0000, SR_2[31:4]} : SR_2;

//shift 4 - 8 bit
wire [31:0] SR_4;
assign SR_4 = i_op2[3] ?  { 8'b00000000, SR_3[31:8]} : SR_3;

//shift 5 - 16 bit
wire [31:0] SR_5;
assign SR_5 = i_op2[4] ?  { 16'b0000000000000000, SR_4[31:16]} : SR_4;


//SRA
//shift 1 - 1 bit
wire [31:0] SA_1;
assign SA_1 = i_op2[0] ?  { i_op1[31], i_op1[31:1]} : i_op1;

//shift 2 - 2 bit
wire [31:0] SA_2;
assign SA_2 = i_op2[1] ?  {{2{i_op1[31]}}, SA_1[31:2]} : SA_1;

//shift 3 - 4 bit
wire [31:0] SA_3;
assign SA_3 = i_op2[2] ?  {{4{i_op1[31]}}, SA_2[31:4]} : SA_2;

//shift 4 - 8 bit
wire [31:0] SA_4;
assign SA_4 = i_op2[3] ?  {{8{i_op1[31]}}, SA_3[31:8]} : SA_3;

//shift 5 - 16 bit
wire [31:0] SA_5;
assign SA_5 = i_op2[4] ?  {{16{i_op1[31]}}, SA_4[31:16]} : SA_4;

//*********************************
// SLT / SLTU - COMPARATOR
//*********************************

//32 BIT COMPARATOR - 4 BIT COMPARATOR USED 8 TIMES TO ACHIEVE 32 BITS

//GROUP#1 (BITS 0-3)
wire EQ_A;
wire EQ1;
wire EQ2;
wire EQ3;
wire EQ4;

assign EQ1 = ~(i_op1[0] ^ i_op2[0]);
assign EQ2 = ~(i_op1[1] ^ i_op2[1]);
assign EQ3 = ~(i_op1[2] ^ i_op2[2]);
assign EQ4 = ~(i_op1[3] ^ i_op2[3]);
assign EQ_A = EQ4 & EQ3 & EQ2 & EQ1;

wire LT1;
assign LT1 =  (~i_op1[3] & i_op2[3]) | (EQ4 & ~i_op1[2] & i_op2[2]) | (EQ4 & EQ3 & ~i_op1[1] & i_op2[1]) | (EQ4 & EQ3 & EQ2 & ~i_op1[0] & i_op2[0]);

//GROUP#2 (BITS 4-7)
wire EQ_B;
wire EQ5;
wire EQ6;
wire EQ7;
wire EQ8;

assign EQ5 = ~(i_op1[4] ^ i_op2[4]);
assign EQ6 = ~(i_op1[5] ^ i_op2[5]);
assign EQ7 = ~(i_op1[6] ^ i_op2[6]);
assign EQ8 = ~(i_op1[7] ^ i_op2[7]);
assign EQ_B = EQ8 & EQ7 & EQ6 & EQ5;

wire LT2;
assign LT2 =  (~i_op1[7] & i_op2[7]) | (EQ8 & ~i_op1[6] & i_op2[6]) | (EQ8 & EQ7 & ~i_op1[5] & i_op2[5]) | (EQ8 & EQ7 & EQ6 & ~i_op1[4] & i_op2[4]);

//GROUP#3 (BITS 8-11)
wire EQ_C;
wire EQ9;
wire EQ10;
wire EQ11;
wire EQ12;

assign EQ9 = ~(i_op1[8] ^ i_op2[8]);
assign EQ10 = ~(i_op1[9] ^ i_op2[9]);
assign EQ11 = ~(i_op1[10] ^ i_op2[10]);
assign EQ12= ~(i_op1[11] ^ i_op2[11]);
assign EQ_C = EQ12 & EQ11 & EQ10 & EQ9;

wire LT3;
assign LT3 =  (~i_op1[11] & i_op2[11]) | (EQ12 & ~i_op1[10] & i_op2[10]) | (EQ12 & EQ11 & ~i_op1[9] & i_op2[9]) | (EQ12 & EQ11 & EQ10 & ~i_op1[8] & i_op2[8]);

//GROUP#4 (BITS 12-15)
wire EQ_D;
wire EQ13;
wire EQ14;
wire EQ15;
wire EQ16;

assign EQ13 = ~(i_op1[12] ^ i_op2[12]);
assign EQ14 = ~(i_op1[13] ^ i_op2[13]);
assign EQ15 = ~(i_op1[14] ^ i_op2[14]);
assign EQ16 = ~(i_op1[15] ^ i_op2[15]);
assign EQ_D = EQ16 & EQ15 & EQ14 & EQ13;

wire LT4;
assign LT4 =  (~i_op1[15] & i_op2[15]) | (EQ16 & ~i_op1[14] & i_op2[14]) | (EQ16 & EQ15 & ~i_op1[13] & i_op2[13]) | (EQ16 & EQ15 & EQ14 & ~i_op1[12] & i_op2[12]);

//GROUP#5 (BITS 16-19)
wire EQ_E;
wire EQ17;
wire EQ18;
wire EQ19;
wire EQ20;

assign EQ17 = ~(i_op1[16] ^ i_op2[16]);
assign EQ18 = ~(i_op1[17] ^ i_op2[17]);
assign EQ19 = ~(i_op1[18] ^ i_op2[18]);
assign EQ20= ~(i_op1[19] ^ i_op2[19]);
assign EQ_E = EQ20 & EQ19 & EQ18 & EQ17;

wire LT5;
assign LT5 =  (~i_op1[19] & i_op2[19]) | (EQ20 & ~i_op1[18] & i_op2[18]) | (EQ20 & EQ19 & ~i_op1[17] & i_op2[17]) | (EQ20 & EQ19 & EQ18 & ~i_op1[16] & i_op2[16]);

//GROUP#6 (BITS 20-23)
wire EQ_F;
wire EQ21;
wire EQ22;
wire EQ23;
wire EQ24;

assign EQ21 = ~(i_op1[20] ^ i_op2[20]);
assign EQ22 = ~(i_op1[21] ^ i_op2[21]);
assign EQ23 = ~(i_op1[22] ^ i_op2[22]);
assign EQ24 = ~(i_op1[23] ^ i_op2[23]);
assign EQ_F = EQ24 & EQ23 & EQ22 & EQ21;

wire LT6;
assign LT6 =  (~i_op1[23] & i_op2[23]) | (EQ24 & ~i_op1[22] & i_op2[22]) | (EQ24 & EQ23 & ~i_op1[21] & i_op2[21]) | (EQ24 & EQ23 & EQ22 & ~i_op1[20] & i_op2[20]);

//GROUP#7 (BITS 24-27)
wire EQ_G;
wire EQ25;
wire EQ26;
wire EQ27;
wire EQ28;

assign EQ25 = ~(i_op1[24] ^ i_op2[24]);
assign EQ26 = ~(i_op1[25] ^ i_op2[25]);
assign EQ27 = ~(i_op1[26] ^ i_op2[26]);
assign EQ28 = ~(i_op1[27] ^ i_op2[27]);
assign EQ_G = EQ28 & EQ27 & EQ26 & EQ25;

wire LT7;
assign LT7 =  (~i_op1[27] & i_op2[27]) | (EQ28 & ~i_op1[26] & i_op2[26]) | (EQ28 & EQ27 & ~i_op1[25] & i_op2[25]) | (EQ28 & EQ27 & EQ26 & ~i_op1[24] & i_op2[24]);

//GROUP#8 (BITS 28-31)
wire EQ_H;
wire EQ29;
wire EQ30;
wire EQ31;
wire EQ32;

assign EQ29 = ~(i_op1[28] ^ i_op2[28]);
assign EQ30 = ~(i_op1[29] ^ i_op2[29]);
assign EQ31 = ~(i_op1[30] ^ i_op2[30]);
assign EQ32 = ~(i_op1[31] ^ i_op2[31]);
assign EQ_H = EQ32 & EQ31 & EQ30 & EQ29;

wire LT8;
assign LT8 =  (~i_op1[31] & i_op2[31]) | (EQ32 & ~i_op1[30] & i_op2[30]) | (EQ32 & EQ31 & ~i_op1[29] & i_op2[29]) | (EQ32 & EQ31 & EQ30 & ~i_op1[28] & i_op2[28]);

//FINAL COMPARATOR  ANSWER
wire LT_FINAL;
assign LT_FINAL = LT8 | (EQ_H & LT7) | (EQ_H & EQ_G & LT6) | (EQ_H & EQ_G & EQ_F & LT5) | (EQ_H & EQ_G & EQ_F & EQ_E & LT4) | (EQ_H & EQ_G & EQ_F & EQ_E & EQ_D & LT3) | (EQ_H & EQ_G & EQ_F & EQ_E & EQ_D & EQ_C & LT2) | (EQ_H & EQ_G & EQ_F & EQ_E & EQ_D & EQ_C & EQ_B & LT1);

//SLTU - 4 BIT COMPARATOR USED 8 TIMES TO ACHIEVE 32 BITS
wire SLTU;
assign SLTU = LT_FINAL;

//SLT - TAKING UNSIGNED VALUE AND EVALUATING THE SIGN
wire SLT;
assign SLT = (i_op1[31] & ~i_op2[31]) | ((i_op1[31] ~^ i_op2[31]) & LT_FINAL);
//*********************************
//XOR / OR / AND
//*********************************
wire [31:0] XOR; //STORES XOR RESULT
wire [31:0] OR; //STORES OR RESULT
wire [31:0] AND; // STORES AND RESULTS

assign XOR = i_op1 ^ i_op2; //OPERATION EQUATION
assign OR = i_op1 | i_op2; //OPERATION EQUATION
assign AND = i_op1 & i_op2; //OPERATION EQUATION

//*******************
//OPERATION SELECTION
//********************
//*********************************
// RV32M MULTIPLIER
//*********************************

// MUL uses the low 32 bits, which are identical for signed and unsigned
// multiplication. MULH, MULHSU, and MULHU select operand signedness below.
wire mul_a_signed;
wire mul_b_signed;
wire [31:0] mul_a_mag;
wire [31:0] mul_b_mag;
wire mul_negative;

assign mul_a_signed = (i_opsel == 3'b001) || (i_opsel == 3'b010);
assign mul_b_signed = (i_opsel == 3'b001);

assign mul_a_mag =
    (mul_a_signed && i_op1[31]) ? (~i_op1 + 32'd1) : i_op1;
assign mul_b_mag =
    (mul_b_signed && i_op2[31]) ? (~i_op2 + 32'd1) : i_op2;

assign mul_negative =
    (mul_a_signed && i_op1[31]) ^ (mul_b_signed && i_op2[31]);

// Generate 32 aligned partial products using constant bit wiring.
// This avoids both the '*' operator and variable shift operators.
wire [63:0] mul_partial [0:31];
wire [63:0] mul_sum      [0:32];

assign mul_sum[0] = 64'b0;

genvar mul_i;
genvar mul_bit;
generate
    for (mul_i = 0; mul_i < 32; mul_i = mul_i + 1) begin : MUL_PARTIAL_PRODUCTS
        for (mul_bit = 0; mul_bit < 64; mul_bit = mul_bit + 1) begin : MUL_BITS
            if ((mul_bit >= mul_i) && (mul_bit < (mul_i + 32))) begin : SET_BIT
                assign mul_partial[mul_i][mul_bit] =
                    mul_b_mag[mul_i] & mul_a_mag[mul_bit - mul_i];
            end else begin : CLEAR_BIT
                assign mul_partial[mul_i][mul_bit] = 1'b0;
            end
        end
        assign mul_sum[mul_i + 1] =
            mul_sum[mul_i] + mul_partial[mul_i];
    end
endgenerate

wire [63:0] mul_unsigned_product;
wire [63:0] mul_product;
wire [31:0] mul_result;

assign mul_unsigned_product = mul_sum[32];
assign mul_product =
    mul_negative ? (~mul_unsigned_product + 64'd1) : mul_unsigned_product;

assign mul_result =
    (i_opsel == 3'b000) ? mul_unsigned_product[31:0] :
                          mul_product[63:32];


//*********************************
// RV32M RESTORING DIVIDER
//*********************************
localparam [1:0] DIV_IDLE = 2'b00;
localparam [1:0] DIV_RUN  = 2'b01;
localparam [1:0] DIV_DONE = 2'b10;

reg [1:0] div_state;
reg [5:0] div_count;
reg [31:0] div_quotient;
reg [32:0] div_remainder;
reg [31:0] div_divisor;
reg        div_signed;
reg        div_is_rem;
reg        div_quotient_negative;
reg        div_remainder_negative;
reg [31:0] div_result_reg;

wire [32:0] div_shifted_remainder;
wire [31:0] div_shifted_quotient;
wire [32:0] div_subtracted_remainder;
wire        div_step_take;
wire [32:0] div_next_remainder;
wire [31:0] div_next_quotient;
wire [31:0] div_corrected_quotient;
wire [31:0] div_corrected_remainder;
wire [31:0] div_abs_op1;
wire [31:0] div_abs_op2;
wire        div_request;

assign div_request = i_mext && i_opsel[2];
assign div_abs_op1 =
    (i_opsel[0] == 1'b0 && i_op1[31]) ? (~i_op1 + 32'd1) : i_op1;
assign div_abs_op2 =
    (i_opsel[0] == 1'b0 && i_op2[31]) ? (~i_op2 + 32'd1) : i_op2;

assign div_shifted_remainder = {div_remainder[31:0], div_quotient[31]};
assign div_shifted_quotient  = {div_quotient[30:0], 1'b0};
assign div_subtracted_remainder =
    div_shifted_remainder - {1'b0, div_divisor};
assign div_step_take = ~div_subtracted_remainder[32];
assign div_next_remainder =
    div_step_take ? div_subtracted_remainder : div_shifted_remainder;
assign div_next_quotient =
    {div_quotient[30:0], div_step_take};

assign div_corrected_quotient =
    div_quotient_negative ? (~div_next_quotient + 32'd1) : div_next_quotient;
assign div_corrected_remainder =
    div_remainder_negative ? (~div_next_remainder[31:0] + 32'd1) :
                             div_next_remainder[31:0];

always @(posedge i_clk) begin
    if (i_rst) begin
        div_state <= DIV_IDLE;
        div_count <= 6'd0;
        div_quotient <= 32'd0;
        div_remainder <= 33'd0;
        div_divisor <= 32'd0;
        div_signed <= 1'b0;
        div_is_rem <= 1'b0;
        div_quotient_negative <= 1'b0;
        div_remainder_negative <= 1'b0;
        div_result_reg <= 32'd0;
    end else begin
        case (div_state)
            DIV_IDLE: begin
                if (div_request) begin
                    div_signed <= ~i_opsel[0];
                    div_is_rem <= i_opsel[1];
                    div_quotient_negative <= (~i_opsel[0]) &&
                                             (i_op1[31] ^ i_op2[31]);
                    div_remainder_negative <= (~i_opsel[0]) && i_op1[31];
                    div_count <= 6'd0;
                    div_remainder <= 33'd0;

                    if (i_op2 == 32'd0) begin
                        div_result_reg <= i_opsel[1] ? i_op1 : 32'hFFFFFFFF;
                        div_state <= DIV_DONE;
                    end else if ((i_opsel == 3'b100) &&
                                 (i_op1 == 32'h80000000) &&
                                 (i_op2 == 32'hFFFFFFFF)) begin
                        div_result_reg <= 32'h80000000;
                        div_state <= DIV_DONE;
                    end else if ((i_opsel == 3'b110) &&
                                 (i_op1 == 32'h80000000) &&
                                 (i_op2 == 32'hFFFFFFFF)) begin
                        div_result_reg <= 32'd0;
                        div_state <= DIV_DONE;
                    end else begin
                        div_quotient <= div_abs_op1;
                        div_divisor <= div_abs_op2;
                        div_state <= DIV_RUN;
                    end
                end
            end

            DIV_RUN: begin
                div_remainder <= div_next_remainder;
                div_quotient <= div_next_quotient;
                div_count <= div_count + 6'd1;

                if (div_count == 6'd31) begin
                    div_result_reg <= div_is_rem ?
                        div_corrected_remainder : div_corrected_quotient;
                    div_state <= DIV_DONE;
                end
            end

            DIV_DONE: begin
                if (i_advance)
                    div_state <= DIV_IDLE;
            end

            default: begin
                div_state <= DIV_IDLE;
            end
        endcase
    end
end

wire [31:0] base_result;


assign base_result =
    (i_opsel == 3'b000) ? SUM :
    (i_opsel == 3'b001) ? SL_5 :
    ((i_opsel == 3'b010) || (i_opsel == 3'b011)) ? {{31{1'b0}}, o_slt} :
    (i_opsel == 3'b100) ? XOR :
    (i_opsel == 3'b101) ? (i_arith ? SA_5 : SR_5) :
    (i_opsel == 3'b110) ? OR :
    (i_opsel == 3'b111) ? AND : 32'b0;

// Select RV32I, multiply, or the held multi-cycle divide result.
assign o_result = i_mext ?
    (i_opsel[2] ? div_result_reg :
                  mul_result) :
    base_result;

assign o_busy = div_request && (div_state != DIV_DONE);

// Set-less-than output:
// - funct3 010 selects signed comparison.
// - funct3 011 selects unsigned comparison.
// - For branch comparisons using the ADD/SUB datapath, i_unsigned
//   selects signed or unsigned comparison.
// - Other ALU operations do not use o_slt.
assign o_slt =
    (i_opsel == 3'b010) ? SLT :
    (i_opsel == 3'b011) ? SLTU :
                          1'b0;
assign o_eq  = ~(|(i_op1 ^ i_op2));



endmodule
`default_nettype wire
