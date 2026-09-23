`timescale 1ns / 1ps

module hart #(
   
    parameter RESET_ADDR = 32'h00000000) (//starting address of the pc
    // Global clock.
    input  wire        i_clk, // PC UPDATE
    // Synchronous active-high reset.
    input  wire        i_rst, //RESET pc to RESET ADDR

    
    output wire [31:0] o_imem_raddr,// SENDS INSTRUCTION ADDRESS TO THE INSTRUCTION MEMOTY 
    input  wire [31:0] i_imem_rdata, // RECIEVES THE INSTRUCTION FROM MEMORY
    output wire [31:0] o_dmem_addr,
    output wire        o_dmem_ren,
    output wire        o_dmem_wen,
    output wire [31:0] o_dmem_wdata,
    output wire [ 3:0] o_dmem_mask,
    input  wire [31:0] i_dmem_rdata,
    output wire        o_retire_valid,
    output wire [31:0] o_retire_inst,
    output wire        o_retire_trap,
    output wire        o_retire_halt,
    output wire [ 4:0] o_retire_rs1_raddr,
    output wire [31:0] o_retire_rs1_rdata,
    output wire [ 4:0] o_retire_rs2_raddr,
    output wire [31:0] o_retire_rs2_rdata,
    output wire [ 4:0] o_retire_rd_waddr,
    output wire [31:0] o_retire_rd_wdata,
    output wire [31:0] o_retire_pc,
    output wire [31:0] o_retire_next_pc
);
    // Your implementation goes under here
    // ------------------------------------
    
    
    reg [31:0] PC;// 32 bit register that will hold the address of the instruction that is being fetched/retrieved
    assign o_imem_raddr = PC; //the instruction  adressed stored in PC is being sent to [o_imem_raddr ] 
    
    wire [31:0] nxt_instruct; // next instruxtion 
    assign nxt_instruct = PC + 32'd4; //  NEXT INSTRUCTION ADDRESS CALCULATION PC + 4 
    
    always @(posedge i_clk) begin //when clk is HIGH(1) work begins
    if(i_rst) 
    PC <= RESET_ADDR;
    else
    PC <= nxt_instruct;
    end
    
    //CONNECTING DECODER TO THE INSTRUCTION RETRIEVED
    //DECODER WIRES
       wire        r_legal;
       wire        r_halt;    
        
       wire    r_op1_sel;
       wire     r_op2_sel;     
       wire  [ 2:0] r_alu_opsel;     
       wire r_alu_sub;     
       wire  r_alu_unsigned;     
       wire r_alu_arith; 
           
       wire  r_branch;     
       wire  r_jump;     
       wire  r_branch_equal;     
       wire r_branch_unsigned;     
       wire  r_branch_invert; 
           
       wire        r_dmem_ren;     
       wire       r_dmem_wen;     
       wire [ 1:0] r_dmem_align;     
       wire     r_dmem_memb;     
       wire     r_dmem_memh;     
       wire     r_dmem_memw;     
       wire      r_dmem_memu;   
         
       wire[ 3:0] r_rd_sel;     
       wire r_pc_sel;   
      
      wire [31:0]r_immediate; 
      wire[4:0] r_rs1;            
      wire[4:0] r_rs2;             
      wire [4:0]r_rd;               
         
         //CONNECTING ALL DECODER OUTPUT TO HART
             decoder decoder_i(                        
             .i_inst(i_imem_rdata),                 
             .o_legal(r_legal),                     
             .o_halt(r_halt),                       
             .o_immediate(r_immediate),             
             .o_rs1(r_rs1),                         
             .o_rs2(r_rs2),                         
             .o_rd(r_rd),                           
             .o_op1_sel(r_op1_sel),                 
             .o_op2_sel(r_op2_sel),                 
             .o_alu_opsel(r_alu_opsel),             
             .o_alu_sub(r_alu_sub),                 
             .o_alu_unsigned(r_alu_unsigned),       
             .o_alu_arith(r_alu_arith),             
             .o_branch(r_branch),                   
             .o_jump(r_jump),                       
             .o_branch_equal(r_branch_equal),       
             .o_branch_unsigned(r_branch_unsigned), 
             .o_branch_invert(r_branch_invert),     
             .o_dmem_ren(r_dmem_ren),               
             .o_dmem_wen(r_dmem_wen),               
             .o_dmem_align(r_dmem_align),           
             .o_dmem_memb(r_dmem_memb),             
             .o_dmem_memh(r_dmem_memh),
             .o_dmem_memw(r_dmem_memw),     
             .o_dmem_memu(r_dmem_memu), 
             .o_rd_sel(r_rd_sel),    
             .o_pc_sel(r_pc_sel) 
             );
             
             //CONNECTING RF(REGISTER FILE)
             
             wire [31:0] rs1_data; 
             wire [31:0] rs2_data; 
             wire [31:0] rd_data; 
             
             //Connecting  ports to hart through decoder
             rf #(.BYPASS_EN(0)) 
             rf_connect(
             .i_clk(i_clk),
             .i_rst(i_rst),
             .o_rs1_rdata(rs1_data),
             .o_rs2_rdata(rs2_data),
             .i_rs1_raddr(r_rs1),
             .i_rs2_raddr(r_rs2),  
             .i_rd_waddr(r_rd),  
             .i_rd_wdata(rd_data)
             );
             
             //CONNECTING THE ALU FILE
             
             wire r_alu_slt;
             wire r_alu_eq;
             wire [31:0] r_result;
             wire [31:0] r_alu_op1;
             wire [31:0] r_alu_op2;
             
             assign r_alu_op1 = r_op1_sel ? PC : rs1_data; //determines if PC adress or rs1 data is put into alu_op1  
             assign r_alu_op2 = r_op2_sel ? r_immediate : rs2_data;     //determines if immediate or rs2 data is put into alu_op2       
             
             //CONNECTING PORTS
             
             alu alu_i(
              .i_opsel(r_alu_opsel),
              .i_sub(r_alu_sub),
              .i_unsigned(r_alu_unsigned),
              .o_slt( r_alu_slt),   
              .o_eq( r_alu_eq),
              .o_result(r_result),
              .i_op2(r_alu_op2),
              .i_op1(r_alu_op1),
              .i_arith(r_alu_arith)
              );
                 
             
 //STARTED NOT COMPLETE   
             
             //BRANCH AND JUMPING LOGIC
             wire[31:0] next_pc;
             wire[31:0] target_address;
             assign target_address = PC + r_immediate; // no incrementing
             
             //MUX Determining if incrementing or branch/jump instruction
             
             
             
              assign o_retire_next_pc = next_pc;
            
              
              
             
             endmodule