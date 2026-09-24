`default_nettype none
`timescale 1ns / 1ps

// A hart ("hardware thread") is one complete RISC-V CPU: it fetches an
// instruction, decodes it, executes it, and writes the result back. This one
// is single-cycle, so all four of those happen in the same clock cycle and
// exactly one instruction retires every cycle -- there are no bubbles and no
// pipeline stages.
//
// You do not write the datapath blocks again here. Instantiate the four
// modules from phase 2 (`alu`, `rf`, `decoder`, which itself contains `imm`)
// and wire them together, then add the parts phase 2 did not have: the
// program counter, the branch/jump target logic, and the memory interfaces.
//
// Remember the two small changes phase 3 needs from your phase 2 modules:
// `rf` is instantiated with BYPASS_EN = 0 (a single-cycle design writes back
// on the same edge the next read samples, so there is nothing to bypass),
// and `rf` no longer has a write enable -- gate a write by driving its write
// address to 5'd0 instead. Do not instantiate a second copy of `imm`; your
// `decoder` already contains one.
module hart #(
    // The address the program counter is initialized to on reset. The first
    // instruction to retire after `i_rst` deasserts must be the one fetched
    // from this address.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,

    // ---- Instruction memory ------------------------------------------
    // The instruction memory is external to your design, read-only, and
    // combinational: the word at `o_imem_raddr` appears on `i_imem_rdata`
    // in the same cycle, with no clock edge and no latency.
    //
    // Address of the instruction to fetch. One instruction per cycle,
    // always 4-byte aligned.
    output wire [31:0] o_imem_raddr,
    // The instruction word stored at `o_imem_raddr`.
    input  wire [31:0] i_imem_rdata,

    // ---- Data memory -------------------------------------------------
    // The data memory is also external and combinational: reads need no
    // clock edge, and writes commit on the next clock edge.
    //
    // Data address. This is **always word-aligned** -- the low two bits of
    // the computed byte address never reach memory. Which bytes inside that
    // word are touched is `o_dmem_mask`'s job, not the address's.
    output wire [31:0] o_dmem_addr,
    // Read enable. Must never be high in the same cycle as `o_dmem_wen`.
    output wire        o_dmem_ren,
    // Write enable.
    output wire        o_dmem_wen,
    // Store data. Only the byte lanes selected by `o_dmem_mask` are used;
    // the rest are ignored, so they may hold anything. For a sub-word store
    // (`sb`, `sh`), the byte(s) must be positioned in the lane(s) they are
    // being written to, not left at the bottom of the word.
    output wire [31:0] o_dmem_wdata,
    // Which of the four byte lanes of the word at `o_dmem_addr` are read or
    // written. A byte access asserts one lane, a half-word two adjacent
    // lanes, and a word all four.
    output wire [ 3:0] o_dmem_mask,
    // The full 32-bit word at `o_dmem_addr`, regardless of the mask.
    // Extracting the requested bytes and sign- or zero-extending them
    // (`lb`/`lh` vs `lbu`/`lhu`) is this module's job.
    input  wire [31:0] i_dmem_rdata,

    // ---- Retire interface --------------------------------------------
    // These outputs are not part of RV32I. They exist so a testbench can see
    // what your design actually did each cycle. Drive every one of them
    // appropriately on every cycle; all of them are checked on every
    // retiring instruction.
    //
    // An instruction retired this cycle. Because the design is
    // single-cycle, this is high every cycle after `i_rst` deasserts,
    // through the cycle `o_retire_halt` fires.
    output wire        o_retire_valid,
    // The raw instruction word that was fetched and retired this cycle.
    output wire [31:0] o_retire_inst,
    // The instruction was an illegal encoding, or a misaligned data access
    // (a half-word access at an odd address, or a word access at an address
    // that is not a multiple of four -- a byte access is never misaligned).
    // A trapping instruction has no side effects: no memory access happens,
    // `o_retire_rd_waddr` is 5'd0, and control flow is not redirected
    // (there is no trap vector in this interface, so `o_retire_next_pc` is
    // still the pc plus four).
    output wire        o_retire_trap,
    // The instruction is `ebreak`, and execution should halt. Like a trap,
    // it reads nothing, writes nothing, and touches no memory.
    output wire        o_retire_halt,
    // First source register address, and the value read from it.
    // Instructions that do not read a first source register (`lui`,
    // `auipc`, `jal`, and illegal encodings) must report 5'd0 here.
    output wire [ 4:0] o_retire_rs1_raddr,
    output wire [31:0] o_retire_rs1_rdata,
    // Second source register address, and the value read from it. Only
    // R-type, store and branch instructions read a second source register;
    // everything else must report 5'd0 here.
    output wire [ 4:0] o_retire_rs2_raddr,
    output wire [31:0] o_retire_rs2_rdata,
    // Destination register address, and the value written to it. When the
    // instruction writes no register, this address must be 5'd0 -- the same
    // convention the decoder used in phase 2, and what discards the write
    // in the register file. The address is checked on every instruction,
    // including trapping ones; the data only matters when the address is
    // nonzero.
    output wire [ 4:0] o_retire_rd_waddr,
    output wire [31:0] o_retire_rd_wdata,
    // The address this instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // The address the next instruction will be fetched from: the pc plus
    // four, or the branch or jump target when a branch is taken or a jump
    // is executed. This is what proves your branch targets, `jal`/`jalr`
    // targets, and `jalr`'s cleared low bit are right.
    output wire [31:0] o_retire_next_pc
);
    // Your implementation goes under here
    // ------------------------------------

    // ========================================================================
    // 1. Program Counter (PC)
    // ========================================================================

    reg [31:0] PC;// 32 bit register that will hold the address of the instruction that is being fetched/retrieved
    wire [31:0] nxt_instruct; // next instruction (PC + 4)
    wire [31:0] next_pc;      // next PC after branch/jump resolution
    
    assign o_imem_raddr = PC; //the instruction adressed stored in PC is being sent to [o_imem_raddr] 
    assign nxt_instruct = PC + 32'd4; // NEXT INSTRUCTION ADDRESS CALCULATION PC + 4 
    
    always @(posedge i_clk) begin //when clk is HIGH(1) work begins
        if(i_rst) 
            PC <= RESET_ADDR;
        else
            PC <= next_pc;
    end
    
    // ========================================================================
    // 2. Decoder & Immediate Generator Linking
    // ========================================================================

    wire        r_legal;
    wire        r_halt;    
     
    wire        r_op1_sel;
    wire        r_op2_sel;     
    wire [2:0]  r_alu_opsel;     
    wire        r_alu_sub;     
    wire        r_alu_unsigned;     
    wire        r_alu_arith; 
        
    wire        r_branch;     
    wire        r_jump;     
    wire        r_branch_equal;     
    wire        r_branch_unsigned;     
    wire        r_branch_invert; 
        
    wire        r_dmem_ren;     
    wire        r_dmem_wen;     
    wire [1:0]  r_dmem_align;     
    wire        r_dmem_memb;     
    wire        r_dmem_memh;     
    wire        r_dmem_memw;     
    wire        r_dmem_memu;   
      
    wire [3:0]  r_rd_sel;     
    wire        r_pc_sel;   
    
    wire [31:0] r_immediate; 
    wire [4:0]  r_rs1;            
    wire [4:0]  r_rs2;             
    wire [4:0]  r_rd;               
         
    // CONNECTING ALL DECODER OUTPUT TO HART
    decoder decoder_i(                        
        .i_inst             (i_imem_rdata),                 
        .o_legal            (r_legal),                     
        .o_halt             (r_halt),                       
        .o_immediate        (r_immediate),             
        .o_rs1              (r_rs1),                         
        .o_rs2              (r_rs2),                         
        .o_rd               (r_rd),                           
        .o_op1_sel          (r_op1_sel),                 
        .o_op2_sel          (r_op2_sel),                 
        .o_alu_opsel        (r_alu_opsel),             
        .o_alu_sub          (r_alu_sub),                 
        .o_alu_unsigned     (r_alu_unsigned),       
        .o_alu_arith        (r_alu_arith),             
        .o_branch           (r_branch),                   
        .o_jump             (r_jump),                       
        .o_branch_equal     (r_branch_equal),       
        .o_branch_unsigned  (r_branch_unsigned), 
        .o_branch_invert    (r_branch_invert),     
        .o_dmem_ren         (r_dmem_ren),               
        .o_dmem_wen         (r_dmem_wen),               
        .o_dmem_align       (r_dmem_align),           
        .o_dmem_memb        (r_dmem_memb),             
        .o_dmem_memh        (r_dmem_memh),
        .o_dmem_memw        (r_dmem_memw),     
        .o_dmem_memu        (r_dmem_memu), 
        .o_rd_sel           (r_rd_sel),    
        .o_pc_sel           (r_pc_sel) 
    );
    
    // ========================================================================
    // 3. Register File (RF) Linking
    // ========================================================================
    
    // BYPASS_EN = 0 per single-cycle specification.
    // Writes to x0 or when trapped are discarded by gating i_rd_waddr to 5'd0.
    wire [31:0] rs1_data; 
    wire [31:0] rs2_data; 
    wire [31:0] rd_data; 
    wire [4:0]  actual_rd_waddr;
    wire        trap_condition;

    assign actual_rd_waddr = trap_condition ? 5'd0 : r_rd;
    
    // Connecting  ports to hart through decoder
    rf #(
        .BYPASS_EN(0)
    ) 
    
    rf_connect(
        .i_clk          (i_clk),
        .i_rst          (i_rst),
        .o_rs1_rdata    (rs1_data),
        .o_rs2_rdata    (rs2_data),
        .i_rs1_raddr    (r_rs1),
        .i_rs2_raddr    (r_rs2),  
        .i_rd_waddr     (actual_rd_waddr),  
        .i_rd_wdata     (rd_data)
    );
    
    // ========================================================================
    // 4. ALU Linking
    // ========================================================================
    
    wire        r_alu_slt;
    wire        r_alu_eq;
    wire [31:0] r_result;
    wire [31:0] r_alu_op1;
    wire [31:0] r_alu_op2;
    
    assign r_alu_op1 = r_op1_sel ? PC : rs1_data; //determines if PC adress or rs1 data is put into alu_op1  
    assign r_alu_op2 = r_op2_sel ? r_immediate : rs2_data;     //determines if immediate or rs2 data is put into alu_op2       
    
    //CONNECTING PORTS
    
    alu alu_i(
        .i_opsel    (r_alu_opsel),
        .i_sub      (r_alu_sub),
        .i_unsigned (r_alu_unsigned),
        .o_slt      (r_alu_slt),   
        .o_eq       (r_alu_eq),
        .o_result   (r_result),
        .i_op2      (r_alu_op2),
        .i_op1      (r_alu_op1),
        .i_arith    (r_alu_arith)
    );

    // ========================================================================
    // [TEAMMATE HOOK 1]: Branch and Jump Logic
    // ========================================================================
    // Computes target addresses and whether a branch/jump is taken.
    wire [31:0] branch_target;
    wire [31:0] jump_target;
    wire        branch_comp;
    wire        branch_taken;

    assign branch_target = PC + r_immediate;
    assign jump_target   = r_pc_sel ? (r_result & ~32'd1) : (PC + r_immediate);

    assign branch_comp   = r_branch_equal ? r_alu_eq : r_alu_slt;
    assign branch_taken  = r_branch & (branch_comp ^ r_branch_invert);

    assign next_pc = r_jump       ? jump_target :
                     branch_taken ? branch_target :
                                    nxt_instruct;

    // ========================================================================
    // [TEAMMATE HOOK 2]: Data Memory Interface & Sub-word Alignment
    // ========================================================================
    // Checks alignment, forms byte mask, shifts store data, and sign/zero extends loads.
    wire [1:0] mem_byte_offset;
    assign mem_byte_offset = r_result[1:0];

    // Misalignment check:
    // Half-word access requires 2-byte alignment (LSB must be 0).
    // Word access requires 4-byte alignment (LSBs must be 00).
    wire misaligned_access;
    assign misaligned_access = (r_dmem_ren | r_dmem_wen) & (
        (r_dmem_memh & mem_byte_offset[0]) |
        (r_dmem_memw & (mem_byte_offset != 2'b00))
    );

    assign trap_condition = (!r_legal) | misaligned_access;

    // Word-aligned address output to memory
    assign o_dmem_addr = {r_result[31:2], 2'b00};

    // Suppress memory access on trap or halt
    assign o_dmem_ren  = r_dmem_ren & (!trap_condition) & (!r_halt);
    assign o_dmem_wen  = r_dmem_wen & (!trap_condition) & (!r_halt);

    // Byte lane mask generation
    assign o_dmem_mask = r_dmem_memw ? 4'b1111 :
                         r_dmem_memh ? (mem_byte_offset[1] ? 4'b1100 : 4'b0011) :
                         r_dmem_memb ? (
                             (mem_byte_offset == 2'b00) ? 4'b0001 :
                             (mem_byte_offset == 2'b01) ? 4'b0010 :
                             (mem_byte_offset == 2'b10) ? 4'b0100 : 4'b1000
                         ) : 4'b0000;

    // Store data positioning into active byte lanes
    assign o_dmem_wdata = r_dmem_memw ? rs2_data :
                          r_dmem_memh ? (mem_byte_offset[1] ? {rs2_data[15:0], 16'b0} : {16'b0, rs2_data[15:0]}) :
                          r_dmem_memb ? (
                              (mem_byte_offset == 2'b00) ? {24'b0, rs2_data[7:0]} :
                              (mem_byte_offset == 2'b01) ? {16'b0, rs2_data[7:0], 8'b0} :
                              (mem_byte_offset == 2'b10) ? {8'b0, rs2_data[7:0], 16'b0} :
                                                           {rs2_data[7:0], 24'b0}
                          ) : rs2_data;

    // Load data byte lane extraction and sign/zero extension
    wire [ 7:0] raw_byte;
    wire [15:0] raw_half;
    wire [31:0] mem_rdata_ext;

    assign raw_byte = (mem_byte_offset == 2'b00) ? i_dmem_rdata[ 7: 0] :
                      (mem_byte_offset == 2'b01) ? i_dmem_rdata[15: 8] :
                      (mem_byte_offset == 2'b10) ? i_dmem_rdata[23:16] :
                                                   i_dmem_rdata[31:24];

    assign raw_half = mem_byte_offset[1] ? i_dmem_rdata[31:16] : i_dmem_rdata[15:0];

    assign mem_rdata_ext = r_dmem_memw ? i_dmem_rdata :
                           r_dmem_memh ? (r_dmem_memu ? {16'b0, raw_half} : {{16{raw_half[15]}}, raw_half}) :
                           r_dmem_memb ? (r_dmem_memu ? {24'b0, raw_byte} : {{24{raw_byte[7]}}, raw_byte}) :
                                         32'd0;

    // ========================================================================
    // [TEAMMATE HOOK 3]: Writeback Multiplexer
    // ========================================================================
    // Selects destination data based on one-hot r_rd_sel:
    // [0] = ALU result
    // [1] = immediate (LUI)
    // [2] = PC + 4 (JAL / JALR)
    // [3] = memory load data
    assign rd_data = r_rd_sel[0] ? r_result :
                     r_rd_sel[1] ? r_immediate :
                     r_rd_sel[2] ? nxt_instruct :
                     r_rd_sel[3] ? mem_rdata_ext :
                                   32'd0;

    // ========================================================================
    // [TEAMMATE HOOK 4]: Retire Interface
    // ========================================================================
    // Drives cycle-by-cycle retire information for the testbench.
    wire [6:0] inst_opcode;
    assign inst_opcode = i_imem_rdata[6:0];

    // rs1 is read by OP, OP-IMM, LOAD, STORE, BRANCH, JALR (not LUI, AUIPC, JAL, or illegal)
    wire rs1_is_read;
    assign rs1_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0010011) | // OP-IMM
        (inst_opcode == 7'b0000011) | // LOAD
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011) | // BRANCH
        (inst_opcode == 7'b1100111)   // JALR
    );

    // rs2 is read only by OP, STORE, and BRANCH
    wire rs2_is_read;
    assign rs2_is_read = r_legal & (
        (inst_opcode == 7'b0110011) | // OP
        (inst_opcode == 7'b0100011) | // STORE
        (inst_opcode == 7'b1100011)   // BRANCH
    );

    assign o_retire_valid     = ~i_rst;
    assign o_retire_inst      = i_imem_rdata;
    assign o_retire_trap      = trap_condition;
    assign o_retire_halt      = r_halt;
    assign o_retire_rs1_raddr = rs1_is_read ? r_rs1 : 5'd0;
    assign o_retire_rs1_rdata = rs1_data;
    assign o_retire_rs2_raddr = rs2_is_read ? r_rs2 : 5'd0;
    assign o_retire_rs2_rdata = rs2_data;
    assign o_retire_rd_waddr  = actual_rd_waddr;
    assign o_retire_rd_wdata  = rd_data;
    assign o_retire_pc        = PC;
    assign o_retire_next_pc   = next_pc;

endmodule

`default_nettype wire