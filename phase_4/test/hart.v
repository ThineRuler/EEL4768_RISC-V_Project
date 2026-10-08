`default_nettype none

module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00400000,
    // When set, pipeline forwarding optimizations are enabled.
    parameter FWD_EN = 1,
    // When set, register file bypassing is enabled.
    parameter BYPASS_EN = 1
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available on the same cycle.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002003`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
    // The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    output wire [31:0] o_retire_dmem_addr,
    output wire [ 3:0] o_retire_dmem_mask,
    output wire        o_retire_dmem_ren,
    output wire        o_retire_dmem_wen,
    output wire [31:0] o_retire_dmem_rdata,
    output wire [31:0] o_retire_dmem_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);

// Write your code here.

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
         else if(stall) // when stall =1 then the PC doesnt advance 
            PC <= PC;
        else
            PC <= next_pc;
    end

    // ========================================================================
    // IF/ID Pipeline Register
    // ========================================================================
    
    reg [31:0] IF_ID_PC;
    reg [31:0] IF_ID_instruct;
    reg [31:0] IF_ID_next_pc;
    reg        IF_ID_valid;

    //TODO: Take into account stalls and flushes when implementing the IF/ID pipeline register.
    always @(posedge i_clk) begin
        if (i_rst) begin
            IF_ID_PC <= 32'd0;
            IF_ID_instruct <= 32'h00000013; // NOP instruction
            IF_ID_next_pc <= 32'd0;
            IF_ID_valid <= 1'b0;
            end else if (stall) begin //stall logic
            IF_ID_PC <= IF_ID_PC;
            IF_ID_instruct <= IF_ID_instruct;
            IF_ID_next_pc <= IF_ID_next_pc;
            IF_ID_valid <= IF_ID_valid;
        end else begin
            IF_ID_PC <= PC;
            IF_ID_instruct <= i_imem_rdata;
            IF_ID_next_pc <= nxt_instruct;
            IF_ID_valid <= 1'b1; // Assuming instruction is valid when fetched
        end
    end
    
    // ========================================================================
    // 2. Decoder & Immediate Generator Linking
    // ========================================================================

    // Initializes all the decoder output wires to their default states
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
        .i_inst             (IF_ID_instruct),                 
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
    
    // Writes to x0 or when trapped are discarded by gating i_rd_waddr to 5'd0.
    wire [31:0] rs1_data; 
    wire [31:0] rs2_data; 
    wire [31:0] rd_data; 
    wire [4:0]  actual_rd_waddr;
    wire        trap_condition;

    // Declaration needs to happen before the rf module is instantiated, otherwise the compiler will throw an error.
    reg        MEM_WB_trap;
    reg [4:0]  MEM_WB_rd;

    // WB sends the data back to the register file
    assign actual_rd_waddr = MEM_WB_trap ? 5'd0 : MEM_WB_rd;
    
    rf #(
        .BYPASS_EN(BYPASS_EN)
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

    // Drives cycle-by-cycle retire information for the testbench.
    wire [6:0] inst_opcode;
    assign inst_opcode = IF_ID_instruct[6:0];

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
    
    // ========================================================================
    // STALL/HAZARD DETECTION SIGNAL 
    // ========================================================================

    wire stall;
    //checks if the instruction in the ID stage depends on the load instruction  in the EX stage
    assign stall = 
        ID_EX_valid && ID_EX_dmem_ren && (ID_EX_rd != 5'd0) && 
        (
            (rs1_is_read && (r_rs1 == ID_EX_rd)) || //does rs1 read and is it in the same register the load is going to write
            (rs2_is_read && (r_rs2 == ID_EX_rd)) //does rs2 read and is it in the register the load is going to write
        );
        //the "||" mean does either register depend on the load


    // ========================================================================
    // ID/EX Pipeline Registers
    // ========================================================================

    // Instruction and PC tracking
    reg [31:0] ID_EX_PC;
    reg [31:0] ID_EX_next_pc;
    reg [31:0] ID_EX_instruct;
    reg        ID_EX_valid;

    // Input register addresses and data
    reg [4:0] ID_EX_rs1;
    reg [4:0] ID_EX_rs2;
    reg [4:0] ID_EX_rd;
    reg [31:0] ID_EX_immediate;

    // Output register data from the register file
    reg [31:0] ID_EX_rs1_data;
    reg [31:0] ID_EX_rs2_data;

    // Execution control signals
    reg        ID_EX_op1_sel;
    reg        ID_EX_op2_sel;
    reg [2:0]  ID_EX_alu_opsel;
    reg        ID_EX_alu_sub;
    reg        ID_EX_alu_unsigned;
    reg        ID_EX_alu_arith;

    // Branch and jump control signals
    reg        ID_EX_branch;
    reg        ID_EX_jump;
    reg        ID_EX_branch_equal;
    reg        ID_EX_branch_unsigned;
    reg        ID_EX_branch_invert;

    // Data memory control signals
    reg        ID_EX_dmem_ren;
    reg        ID_EX_dmem_wen;
    reg [1:0]  ID_EX_dmem_align;
    reg        ID_EX_dmem_memb;
    reg        ID_EX_dmem_memh;
    reg        ID_EX_dmem_memw;
    reg        ID_EX_dmem_memu;

    // Writeback control signals and retire tracking
    reg [3:0]  ID_EX_rd_sel;
    reg        ID_EX_pc_sel;
    reg        ID_EX_legal;
    reg        ID_EX_halt;
    reg        ID_EX_rs1_is_read;
    reg        ID_EX_rs2_is_read;
    
    always @(posedge i_clk) begin
        if (i_rst) begin
            ID_EX_PC <= 32'd0;
            ID_EX_next_pc <= 32'd0;
            ID_EX_instruct <= 32'h00000013; // NOP instruction
            ID_EX_valid <= 1'b0;

            ID_EX_rs1 <= 5'd0;
            ID_EX_rs2 <= 5'd0;
            ID_EX_rd <= 5'd0;
            ID_EX_immediate <= 32'd0;

            ID_EX_rs1_data <= 32'd0;
            ID_EX_rs2_data <= 32'd0;

            // Clear all control signals to default values
            ID_EX_op1_sel <= 1'b0;
            ID_EX_op2_sel <= 1'b0;
            ID_EX_alu_opsel <= 3'd0;
            ID_EX_alu_sub <= 1'b0;
            ID_EX_alu_unsigned <= 1'b0;
            ID_EX_alu_arith <= 1'b0;

            ID_EX_branch <= 1'b0;
            ID_EX_jump <= 1'b0;
            ID_EX_branch_equal <= 1'b0;
            ID_EX_branch_unsigned <= 1'b0;
            ID_EX_branch_invert <= 1'b0;

            ID_EX_dmem_ren <= 1'b0;
            ID_EX_dmem_wen <= 1'b0;
            ID_EX_dmem_align <= 2'd0;
            ID_EX_dmem_memb <= 1'b0;
            ID_EX_dmem_memh <= 1'b0;
            ID_EX_dmem_memw <= 1'b0;
            ID_EX_dmem_memu <= 1'b0;

            ID_EX_rd_sel <= 4'd0;
            ID_EX_pc_sel <= 1'b0;
            ID_EX_legal <= 1'b0;
            ID_EX_halt <= 1'b0;
            ID_EX_rs1_is_read <= 1'b0;
            ID_EX_rs2_is_read <= 1'b0;
            
            end else if(stall) begin
        ID_EX_PC <= 32'd0;
        ID_EX_next_pc <= 32'd0;
        ID_EX_instruct <= 32'h00000013;
        ID_EX_valid <= 1'b0;

        ID_EX_rs1 <= 5'd0;
        ID_EX_rs2 <= 5'd0;
        ID_EX_rd <= 5'd0;
        ID_EX_immediate <= 32'd0;

        ID_EX_rs1_data <= 32'd0;
        ID_EX_rs2_data <= 32'd0;

        ID_EX_op1_sel <= 1'b0;
        ID_EX_op2_sel <= 1'b0;
        ID_EX_alu_opsel <= 3'd0;
        ID_EX_alu_sub <= 1'b0;
        ID_EX_alu_unsigned <= 1'b0;
        ID_EX_alu_arith <= 1'b0;

        ID_EX_branch <= 1'b0;
        ID_EX_jump <= 1'b0;
        ID_EX_branch_equal <= 1'b0;
        ID_EX_branch_unsigned <= 1'b0;
        ID_EX_branch_invert <= 1'b0;

        ID_EX_dmem_ren <= 1'b0;
        ID_EX_dmem_wen <= 1'b0;
        ID_EX_dmem_align <= 2'd0;
        ID_EX_dmem_memb <= 1'b0;
        ID_EX_dmem_memh <= 1'b0;
        ID_EX_dmem_memw <= 1'b0;
        ID_EX_dmem_memu <= 1'b0;

        ID_EX_rd_sel <= 4'd0;
        ID_EX_pc_sel <= 1'b0;
        ID_EX_legal <= 1'b0;
        ID_EX_halt <= 1'b0;
        ID_EX_rs1_is_read <= 1'b0;
        ID_EX_rs2_is_read <= 1'b0;

        end else begin
            //Normal latching of the Decoder & RF outputs
            ID_EX_PC <= IF_ID_PC;
            ID_EX_next_pc <= IF_ID_next_pc;
            ID_EX_instruct <= IF_ID_instruct;
            ID_EX_valid <= IF_ID_valid;

            ID_EX_rs1 <= r_rs1;
            ID_EX_rs2 <= r_rs2;
            ID_EX_rd <= r_rd;
            ID_EX_immediate <= r_immediate;

            ID_EX_rs1_data <= rs1_data;
            ID_EX_rs2_data <= rs2_data;

            ID_EX_op1_sel <= r_op1_sel;
            ID_EX_op2_sel <= r_op2_sel;
            ID_EX_alu_opsel <= r_alu_opsel;
            ID_EX_alu_sub <= r_alu_sub;
            ID_EX_alu_unsigned <= r_alu_unsigned;
            ID_EX_alu_arith <= r_alu_arith;

            ID_EX_branch <= r_branch;
            ID_EX_jump <= r_jump;
            ID_EX_branch_equal <= r_branch_equal;
            ID_EX_branch_unsigned <= r_branch_unsigned;
            ID_EX_branch_invert <= r_branch_invert;

            ID_EX_dmem_ren <= r_dmem_ren;
            ID_EX_dmem_wen <= r_dmem_wen;
            ID_EX_dmem_align <= r_dmem_align;
            ID_EX_dmem_memb <= r_dmem_memb;
            ID_EX_dmem_memh <= r_dmem_memh;
            ID_EX_dmem_memw <= r_dmem_memw;
            ID_EX_dmem_memu <= r_dmem_memu;

            ID_EX_rd_sel <= r_rd_sel;
            ID_EX_pc_sel <= r_pc_sel;
            ID_EX_legal <= r_legal;
            ID_EX_halt <= r_halt;
            ID_EX_rs1_is_read <= rs1_is_read;
            ID_EX_rs2_is_read <= rs2_is_read;
        end
    end

    // ========================================================================
// 4. ALU Linking
// ========================================================================

wire        r_alu_slt;
wire        r_alu_eq;
wire [31:0] r_result;
wire [31:0] r_alu_op1;
wire [31:0] r_alu_op2;

// Forwarding signals
wire [31:0] r_forward_rs1;
wire [31:0] r_forward_rs2;
wire [31:0] r_exmem_fwd_data;
wire [31:0] r_memwb_fwd_data;
wire [31:0] r_store_data;

// ========================================================================
// EX/MEM Forwarding Data
// ========================================================================
// Select the value that would eventually be written to the register file.
// Loads are intentionally excluded here because their data is not ready
// until the MEM/WB stage.

assign r_exmem_fwd_data =
    EX_MEM_rd_sel[0] ? EX_MEM_result :
    EX_MEM_rd_sel[1] ? EX_MEM_immediate :
    EX_MEM_rd_sel[2] ? EX_MEM_next_pc :
                       32'd0;

// ========================================================================
// Forwarding to ALU operand 1 (rs1)
// ========================================================================

// Priority:
// 1. EX/MEM
// 2. MEM/WB
// 3. Register-file value

assign r_forward_rs1 =
    (FWD_EN &&
     ID_EX_rs1 != 5'd0 &&
     EX_MEM_valid &&
     EX_MEM_legal &&
     !EX_MEM_halt &&
     EX_MEM_rd != 5'd0 &&
     (EX_MEM_rd_sel[0] ||
      EX_MEM_rd_sel[1] ||
      EX_MEM_rd_sel[2]) &&
     (EX_MEM_rd == ID_EX_rs1))
    ? r_exmem_fwd_data :

    (FWD_EN &&
     ID_EX_rs1 != 5'd0 &&
     MEM_WB_valid &&
     !MEM_WB_trap &&
     !MEM_WB_halt &&
     MEM_WB_rd != 5'd0 &&
     (MEM_WB_rd_sel[0] ||
      MEM_WB_rd_sel[1] ||
      MEM_WB_rd_sel[2] ||
      MEM_WB_rd_sel[3]) &&
     (MEM_WB_rd == ID_EX_rs1))
    ? r_memwb_fwd_data :

    ID_EX_rs1_data;

// ALU operand 1
assign r_alu_op1 =
    ID_EX_op1_sel ? ID_EX_PC : r_forward_rs1;


// ========================================================================
// Forwarding to ALU operand 2 (rs2)
// ========================================================================

assign r_forward_rs2 =
    (FWD_EN &&
     ID_EX_rs2 != 5'd0 &&
     EX_MEM_valid &&
     EX_MEM_legal &&
     !EX_MEM_halt &&
     EX_MEM_rd != 5'd0 &&
     (EX_MEM_rd_sel[0] ||
      EX_MEM_rd_sel[1] ||
      EX_MEM_rd_sel[2]) &&
     (EX_MEM_rd == ID_EX_rs2))
    ? r_exmem_fwd_data :

    (FWD_EN &&
     ID_EX_rs2 != 5'd0 &&
     MEM_WB_valid &&
     !MEM_WB_trap &&
     !MEM_WB_halt &&
     MEM_WB_rd != 5'd0 &&
     (MEM_WB_rd_sel[0] ||
      MEM_WB_rd_sel[1] ||
      MEM_WB_rd_sel[2] ||
      MEM_WB_rd_sel[3]) &&
     (MEM_WB_rd == ID_EX_rs2))
    ? r_memwb_fwd_data :

    ID_EX_rs2_data;

// ALU operand 2
assign r_alu_op2 =
    ID_EX_op2_sel ? ID_EX_immediate : r_forward_rs2;

    //CONNECTING PORTS
    
    alu alu_i(
        .i_opsel    (ID_EX_alu_opsel),
        .i_sub      (ID_EX_alu_sub),
        .i_unsigned (ID_EX_alu_unsigned),
        .o_slt      (r_alu_slt),   
        .o_eq       (r_alu_eq),
        .o_result   (r_result),
        .i_op1      (r_alu_op1),
        .i_op2      (r_alu_op2),
        .i_arith    (ID_EX_alu_arith)
    );

    // ========================================================================
    // Branch and Jump Logic
    // ========================================================================
    // Computes target addresses and whether a branch/jump is taken.
    wire [31:0] branch_target;
    wire [31:0] jump_target;
    wire        branch_comp;
    wire        branch_taken;


    // Since r_result, r_alu_eq, and r_alu_slt are all computed in the EX stage, 
    // we can use them directly to compute branch/jump targets and whether a branch is taken.
    assign branch_target = ID_EX_PC + ID_EX_immediate;
    assign jump_target   = ID_EX_pc_sel ? (r_result & ~32'd1) : (ID_EX_PC + ID_EX_immediate);

    assign branch_comp   = ID_EX_branch_equal ? r_alu_eq : r_alu_slt;
    assign branch_taken  = ID_EX_branch & (branch_comp ^ ID_EX_branch_invert);

    wire [31:0] EX_next_pc;
    assign next_pc = (ID_EX_jump | branch_taken) ? EX_next_pc : nxt_instruct;
    assign EX_next_pc = ID_EX_jump   ? jump_target :
                        branch_taken ? branch_target :
                                       ID_EX_next_pc;

    // ========================================================================
    // EX/MEM Pipeline Registers
    // ========================================================================

    // Instruction and PC tracking
    reg [31:0] EX_MEM_PC;
    reg [31:0] EX_MEM_next_pc;
    reg [31:0] EX_MEM_instruct;
    reg        EX_MEM_valid;

    // Execution results and data
    reg [31:0] EX_MEM_result;
    reg [31:0] EX_MEM_rs2_data;
    reg [4:0]  EX_MEM_rd;
    reg [31:0] EX_MEM_immediate;

    // Memory control signals
    reg        EX_MEM_dmem_ren;
    reg        EX_MEM_dmem_wen;
    reg [1:0]  EX_MEM_dmem_align;
    reg        EX_MEM_dmem_memb;
    reg        EX_MEM_dmem_memh;
    reg        EX_MEM_dmem_memw;
    reg        EX_MEM_dmem_memu;

    // Writeback control signals and retire tracking
    reg [3:0]  EX_MEM_rd_sel;
    reg        EX_MEM_pc_sel;
    reg        EX_MEM_legal;
    reg        EX_MEM_halt;
    reg [4:0]  EX_MEM_rs1;
    reg [4:0]  EX_MEM_rs2;
    reg [31:0] EX_MEM_rs1_data;
    reg        EX_MEM_rs1_is_read;
    reg        EX_MEM_rs2_is_read;

    always @(posedge i_clk) begin
        if (i_rst) begin
            EX_MEM_PC <= 32'd0;
            EX_MEM_next_pc <= 32'd0;
            EX_MEM_instruct <= 32'h00000013; // NOP instruction
            EX_MEM_valid <= 1'b0;

            EX_MEM_result <= 32'd0;
            EX_MEM_rs2_data <= 32'd0;
            EX_MEM_rd <= 5'd0;
            EX_MEM_immediate <= 32'd0;

            // Clear all control signals to default values
            EX_MEM_dmem_ren <= 1'b0;
            EX_MEM_dmem_wen <= 1'b0;
            EX_MEM_dmem_align <= 2'd0;
            EX_MEM_dmem_memb <= 1'b0;
            EX_MEM_dmem_memh <= 1'b0;
            EX_MEM_dmem_memw <= 1'b0;
            EX_MEM_dmem_memu <= 1'b0;

            EX_MEM_rd_sel <= 4'd0;
            EX_MEM_pc_sel <= 1'b0;
            EX_MEM_legal <= 1'b0;
            EX_MEM_halt <= 1'b0;
            EX_MEM_rs1 <= 5'd0;
            EX_MEM_rs2 <= 5'd0;
            EX_MEM_rs1_data <= 32'd0;
            EX_MEM_rs1_is_read <= 1'b0;
            EX_MEM_rs2_is_read <= 1'b0;
        end else begin
            EX_MEM_PC <= ID_EX_PC;
            EX_MEM_next_pc <= EX_next_pc;
            EX_MEM_instruct <= ID_EX_instruct;
            EX_MEM_valid <= ID_EX_valid;

            EX_MEM_result <= r_result;
            EX_MEM_rs2_data <= r_forward_rs2;
            EX_MEM_rd <= ID_EX_rd;
            EX_MEM_immediate <= ID_EX_immediate;

            EX_MEM_dmem_ren <= ID_EX_dmem_ren;
            EX_MEM_dmem_wen <= ID_EX_dmem_wen;
            EX_MEM_dmem_align <= ID_EX_dmem_align;
            EX_MEM_dmem_memb <= ID_EX_dmem_memb;
            EX_MEM_dmem_memh <= ID_EX_dmem_memh;
            EX_MEM_dmem_memw <= ID_EX_dmem_memw;
            EX_MEM_dmem_memu <= ID_EX_dmem_memu;

            EX_MEM_rd_sel <= ID_EX_rd_sel;
            EX_MEM_pc_sel <= ID_EX_pc_sel;
            EX_MEM_legal <= ID_EX_legal;
            EX_MEM_halt <= ID_EX_halt;
            EX_MEM_rs1 <= ID_EX_rs1;
            EX_MEM_rs2 <= ID_EX_rs2;
            EX_MEM_rs1_data <= r_forward_rs1;
            EX_MEM_rs1_is_read <= ID_EX_rs1_is_read;
            EX_MEM_rs2_is_read <= ID_EX_rs2_is_read;
        end
    end

    // ========================================================================
    // Data Memory Interface & Sub-word Alignment
    // ========================================================================
    // Checks alignment, forms byte mask, shifts store data, and sign/zero extends loads.
    wire [1:0] mem_byte_offset;
    assign mem_byte_offset = EX_MEM_result[1:0];

    // Misalignment check:
    // Half-word access requires 2-byte alignment (LSB must be 0).
    // Word access requires 4-byte alignment (LSBs must be 00).
    wire misaligned_access;
    assign misaligned_access = (EX_MEM_dmem_ren | EX_MEM_dmem_wen) & (
        (EX_MEM_dmem_memh & mem_byte_offset[0]) |
        (EX_MEM_dmem_memw & (mem_byte_offset != 2'b00))
    );

    assign trap_condition = (!EX_MEM_legal) | misaligned_access;

    // Word-aligned address output to memory
    assign o_dmem_addr = {EX_MEM_result[31:2], 2'b00};

    // Suppress memory access on trap or halt
    assign o_dmem_ren  = EX_MEM_dmem_ren & (!trap_condition) & (!EX_MEM_halt);
    assign o_dmem_wen  = EX_MEM_dmem_wen & (!trap_condition) & (!EX_MEM_halt);

    // Byte lane mask generation
    assign o_dmem_mask = EX_MEM_dmem_memw ? 4'b1111 :
                         EX_MEM_dmem_memh ? (mem_byte_offset[1] ? 4'b1100 : 4'b0011) :
                         EX_MEM_dmem_memb ? (
                             (mem_byte_offset == 2'b00) ? 4'b0001 :
                             (mem_byte_offset == 2'b01) ? 4'b0010 :
                             (mem_byte_offset == 2'b10) ? 4'b0100 : 4'b1000
                         ) : 4'b0000;

    // Store data positioning into active byte lanes
    assign o_dmem_wdata = EX_MEM_dmem_memw ? r_store_data :
                      EX_MEM_dmem_memh ? (mem_byte_offset[1] ? {r_store_data[15:0], 16'b0} : {16'b0, r_store_data[15:0]}) :
                          EX_MEM_dmem_memb ? (
                              (mem_byte_offset == 2'b00) ? {24'b0, r_store_data[7:0]} :
                              (mem_byte_offset == 2'b01) ? {16'b0, r_store_data[7:0], 8'b0} :
                              (mem_byte_offset == 2'b10) ? {8'b0, r_store_data[7:0], 16'b0} :
                                                           {r_store_data[7:0], 24'b0}
                          ) : r_store_data;

    // Load data byte lane extraction and sign/zero extension
    wire [ 7:0] raw_byte;
    wire [15:0] raw_half;
    wire [31:0] mem_rdata_ext;

    assign raw_byte = (mem_byte_offset == 2'b00) ? i_dmem_rdata[ 7: 0] :
                      (mem_byte_offset == 2'b01) ? i_dmem_rdata[15: 8] :
                      (mem_byte_offset == 2'b10) ? i_dmem_rdata[23:16] :
                                                   i_dmem_rdata[31:24];

    assign raw_half = mem_byte_offset[1] ? i_dmem_rdata[31:16] : i_dmem_rdata[15:0];

    assign mem_rdata_ext = EX_MEM_dmem_memw ? i_dmem_rdata :
                           EX_MEM_dmem_memh ? (EX_MEM_dmem_memu ? {16'b0, raw_half} : {{16{raw_half[15]}}, raw_half}) :
                           EX_MEM_dmem_memb ? (EX_MEM_dmem_memu ? {24'b0, raw_byte} : {{24{raw_byte[7]}}, raw_byte}) :
                                         32'd0;

    // ========================================================================
    // MEM/WB Pipeline Registers
    // ========================================================================

    // Instruction and PC tracking
    reg [31:0] MEM_WB_PC;
    reg [31:0] MEM_WB_next_pc;
    reg [31:0] MEM_WB_instruct;
    reg        MEM_WB_valid;

    // Writeback data 
    reg [31:0] MEM_WB_result;
    reg [31:0] MEM_WB_mem_rdata;
    reg [31:0] MEM_WB_immediate;
    reg [3:0]  MEM_WB_rd_sel;

    // Status
    reg        MEM_WB_halt;

    // Source registers for retirement
    reg [4:0]  MEM_WB_rs1;
    reg [4:0]  MEM_WB_rs2;
    reg [31:0] MEM_WB_rs1_data;
    reg [31:0] MEM_WB_rs2_data;
    reg        MEM_WB_rs1_is_read;
    reg        MEM_WB_rs2_is_read;

    // Memory operation control signals for retirement
    reg [31:0] MEM_WB_dmem_addr;
    reg        MEM_WB_dmem_ren;
    reg        MEM_WB_dmem_wen;
    reg [3:0]  MEM_WB_dmem_mask;
    reg [31:0] MEM_WB_dmem_rdata;
    reg [31:0] MEM_WB_dmem_wdata;

    always @(posedge i_clk) begin
        if (i_rst) begin
            MEM_WB_PC <= 32'd0;
            MEM_WB_next_pc <= 32'd0;
            MEM_WB_instruct <= 32'h00000013; // NOP instruction
            MEM_WB_valid <= 1'b0;

            MEM_WB_result <= 32'd0;
            MEM_WB_mem_rdata <= 32'd0;
            MEM_WB_rd <= 5'd0;
            MEM_WB_immediate <= 32'd0;
            MEM_WB_rd_sel <= 4'd0;

            MEM_WB_trap <= 1'b0;
            MEM_WB_halt <= 1'b0;

            MEM_WB_rs1 <= 5'd0;
            MEM_WB_rs2 <= 5'd0;
            MEM_WB_rs1_data <= 32'd0;
            MEM_WB_rs2_data <= 32'd0;
            MEM_WB_rs1_is_read <= 1'b0;
            MEM_WB_rs2_is_read <= 1'b0;

            MEM_WB_dmem_addr <= 32'd0;
            MEM_WB_dmem_ren <= 1'b0;
            MEM_WB_dmem_wen <= 1'b0;
            MEM_WB_dmem_mask <= 4'd0;
            MEM_WB_dmem_rdata <= 32'd0;
            MEM_WB_dmem_wdata <= 32'd0;
        end else begin
            // Latch values from EX/MEM stage to MEM/WB stage
            MEM_WB_PC <= EX_MEM_PC;
            MEM_WB_next_pc <= EX_MEM_next_pc;
            MEM_WB_instruct <= EX_MEM_instruct;
            MEM_WB_valid <= EX_MEM_valid;

            MEM_WB_result <= EX_MEM_result;
            MEM_WB_mem_rdata <= mem_rdata_ext; // Load data from memory
            MEM_WB_rd <= EX_MEM_rd;
            MEM_WB_immediate <= EX_MEM_immediate;
            MEM_WB_rd_sel <= EX_MEM_rd_sel;

            // Propagate trap and halt conditions
            MEM_WB_trap <= trap_condition; // Captured in MEM
            MEM_WB_halt <= EX_MEM_halt;

            // Propagate source register information for retirement
            MEM_WB_rs1 <= EX_MEM_rs1;
            MEM_WB_rs2 <= EX_MEM_rs2;
            MEM_WB_rs1_data <= EX_MEM_rs1_data;
            MEM_WB_rs2_data <= EX_MEM_rs2_data;
            MEM_WB_rs1_is_read <= EX_MEM_rs1_is_read;
            MEM_WB_rs2_is_read <= EX_MEM_rs2_is_read;

            // Latch memory operation control signals for retirement
            MEM_WB_dmem_addr <= o_dmem_addr;
            MEM_WB_dmem_ren <= o_dmem_ren;
            MEM_WB_dmem_wen <= o_dmem_wen;
            MEM_WB_dmem_mask <= o_dmem_mask;
            MEM_WB_dmem_rdata <= i_dmem_rdata;
            MEM_WB_dmem_wdata <= o_dmem_wdata;
        end
    end

    // ========================================================================
// Writeback Multiplexer
// ========================================================================

// Selects destination data based on one-hot r_rd_sel:
// [0] = ALU result
// [1] = immediate (LUI)
// [2] = PC + 4 (JAL / JALR)
// [3] = memory load data

assign rd_data = MEM_WB_rd_sel[0] ? MEM_WB_result :
                 MEM_WB_rd_sel[1] ? MEM_WB_immediate :
                 MEM_WB_rd_sel[2] ? MEM_WB_next_pc :
                 MEM_WB_rd_sel[3] ? MEM_WB_mem_rdata :
                                    32'd0;

// MEM/WB forwarding uses the same final value that is written back.
assign r_memwb_fwd_data = rd_data;


// ========================================================================
// MEM -> MEM Store Data Forwarding
// ========================================================================

assign r_store_data =
    (FWD_EN &&
     EX_MEM_rs2 != 5'd0 &&
     MEM_WB_valid &&
     !MEM_WB_trap &&
     !MEM_WB_halt &&
     MEM_WB_rd != 5'd0 &&
     (MEM_WB_rd_sel[0] ||
      MEM_WB_rd_sel[1] ||
      MEM_WB_rd_sel[2] ||
      MEM_WB_rd_sel[3]) &&
     (MEM_WB_rd == EX_MEM_rs2))
    ? r_memwb_fwd_data :
    EX_MEM_rs2_data;

    // ========================================================================
    // Retire Interface
    // ========================================================================

    assign o_retire_valid     = MEM_WB_valid;
    assign o_retire_inst      = MEM_WB_instruct;
    assign o_retire_trap      = MEM_WB_trap;
    assign o_retire_halt      = MEM_WB_halt;
    assign o_retire_rs1_raddr = MEM_WB_rs1_is_read ? MEM_WB_rs1 : 5'd0;
    assign o_retire_rs1_rdata = MEM_WB_rs1_data;
    assign o_retire_rs2_raddr = MEM_WB_rs2_is_read ? MEM_WB_rs2 : 5'd0;
    assign o_retire_rs2_rdata = MEM_WB_rs2_data;
    assign o_retire_rd_waddr  = MEM_WB_trap ? 5'd0 : MEM_WB_rd;
    assign o_retire_rd_wdata  = rd_data;
    assign o_retire_pc        = MEM_WB_PC;
    assign o_retire_next_pc   = MEM_WB_next_pc;

    // Retire data memory interface signals
    assign o_retire_dmem_addr  = MEM_WB_dmem_addr;
    assign o_retire_dmem_mask  = MEM_WB_dmem_mask;
    assign o_retire_dmem_ren   = MEM_WB_dmem_ren;
    assign o_retire_dmem_wen   = MEM_WB_dmem_wen;
    assign o_retire_dmem_rdata = MEM_WB_dmem_rdata;
    assign o_retire_dmem_wdata = MEM_WB_dmem_wdata;

endmodule
