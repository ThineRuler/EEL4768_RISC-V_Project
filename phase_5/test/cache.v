`default_nettype none

// A set associative cache. `hart` instantiates two of them, one as the
// instruction cache and one as the data cache, and passes the same five
// parameters to both. The replacement, write, and prefetch rules are in
// `documentation/phase_5.pdf`.
module cache #(
    // Replacement policy: 0 = LRU, 1 = PLRU.
    parameter EVICT_POLICY = 0,
    // Lines per set: 1 = direct mapped, the number of lines = fully
    // associative.
    parameter NUM_WAYS     = 1,
    // Data capacity in KB. Tags, valid bits and dirty bits are not counted.
    parameter CACHE_SIZE   = 16,
    // Bytes per line.
    parameter BLOCK_SIZE   = 128,
    // Next-line prefetch on a load miss: 1 = on, 0 = off.
    parameter PREFETCH_EN  = 0
) (
    // Global clock.
    input  wire                    i_clk,
    // Synchronous active-high reset.
    input  wire                    i_rst,
    // CPU side: the phase 4 data memory port, plus `o_ready` to stall the
    // pipeline while the cache waits on main memory. How the cache tells the
    // pipeline to stall is up to you, so you can change these ports.
    input  wire                    i_ren,
    input  wire                    i_wen,
    input  wire [31:0]             i_addr,
    input  wire [ 3:0]             i_mask,
    input  wire [31:0]             i_wdata,
    output wire                    o_ready,
    output wire [31:0]             o_rdata,
    // Memory side: one whole line (BLOCK_SIZE bytes) per transfer.
    //
    // Line read request, high for one cycle per request.
    output wire                    o_mem_rd,
    // Line-aligned address of the line to read.
    output wire [31:0]             o_mem_rd_addr,
    // High for one cycle when a requested line arrives. Memory answers
    // requests in order, after a delay set by the testbench: wait for this
    // signal instead of counting cycles.
    input  wire                    i_mem_valid,
    // The whole line, valid while `i_mem_valid` is high.
    input  wire [BLOCK_SIZE*8-1:0] i_mem_rd_data,
    // Line write, high for one cycle per write. Memory applies it on that
    // clock edge.
    output wire                    o_mem_wr,
    // Line-aligned address of the write.
    output wire [31:0]             o_mem_wr_addr,
    // Write data, in place within the line.
    output wire [BLOCK_SIZE*8-1:0] o_mem_wr_data,
    // One bit per byte: all ones for a dirty write-back, only the stored
    // bytes for a write-through.
    output wire [BLOCK_SIZE-1:0]   o_mem_wr_be
);

// ============================================================================
// EEL 4768: Computer Architecture - Phase 5 Cache Design
// Implementation: Parts 1 and 2 (Direct-Mapped, Multi-Way, LRU & PLRU)
//
// OVERVIEW AND ARCHITECTURE:
// --------------------------
// This module implements a parameterizable L1 cache supporting:
//  - Direct-Mapped (NUM_WAYS = 1)
//  - Set-Associative (NUM_WAYS = 2, 4, 8, etc.)
//  - Fully-Associative (NUM_WAYS = TOTAL_BLOCKS, NUM_SETS = 1)
//  - Replacement Policies:
//      * EVICT_POLICY = 0: True LRU (Least Recently Used) via MRU->LRU order list
//      * EVICT_POLICY = 1: PLRU (Pseudo-LRU) via binary decision tree
//      * Rule (Section 4.3 & 4.11): The lowest-numbered invalid way is ALWAYS
//        used before evicting any valid line.
//  - Write Policy (Section 4.4):
//      * Store Hit  : Write-back (update cache line, mark dirty, no memory access)
//      * Store Miss : Write-through, no-allocate (write straight to main memory
//                     with byte enables; line is NOT brought into the cache)
//  - Eviction & Miss Policy (Section 4.4.1 & 4.7):
//      * Clean Eviction : Request new line from memory immediately.
//      * Dirty Eviction : Write full dirty block back to main memory first
//                         (all byte enables high), then request new line.
//      * CPU Handshake  : o_ready stalls the pipeline during memory wait and
//                         releases once data is ready.
//
// ADDRESS DECOMPOSITION (Section 4.5):
// ------------------------------------
// A 32-bit byte address is partitioned as follows:
//  +---------------------------+-------------------+----------------------+
//  | Tag (TAG_BITS)            | Set (SET_BITS)    | Byte Offset (OFFSET) |
//  +---------------------------+-------------------+----------------------+
//  - Byte Offset : OFFSET_BITS = $clog2(BLOCK_SIZE)
//  - Set Index   : SET_BITS    = $clog2(NUM_SETS) where NUM_SETS = (CACHE_SIZE*1024)/(BLOCK_SIZE*NUM_WAYS)
//  - Tag         : TAG_BITS    = 32 - SET_BITS - OFFSET_BITS
//
// When NUM_SETS == 1 (Fully Associative), SET_BITS is 0, so the address splits
// purely into Tag and Byte Offset.
// ============================================================================

    // ------------------------------------------------------------------------
    // 1. Geometry Calculations & Local Parameters
    // ------------------------------------------------------------------------
    localparam TOTAL_BYTES     = CACHE_SIZE * 1024;
    localparam TOTAL_BLOCKS    = TOTAL_BYTES / BLOCK_SIZE;
    localparam NUM_SETS        = TOTAL_BLOCKS / NUM_WAYS;
    localparam [31:0] BLOCK_SIZE_BYTES = BLOCK_SIZE;

    // Address field widths
    localparam OFFSET_BITS     = $clog2(BLOCK_SIZE);
    localparam SET_BITS_RAW    = $clog2(NUM_SETS);
    // Guard against zero-width index wires when NUM_SETS == 1 (Fully Associative)
    localparam SET_BITS        = (SET_BITS_RAW > 0) ? SET_BITS_RAW : 1;
    localparam TAG_BITS        = 32 - SET_BITS_RAW - OFFSET_BITS;

    // Word indexing within a block (each RV32 word is 4 bytes = 32 bits)
    localparam WORD_BITS       = (OFFSET_BITS > 2) ? (OFFSET_BITS - 2) : 1;

    // Way indexing
    localparam WAY_BITS_RAW    = $clog2(NUM_WAYS);
    localparam WAY_BITS        = (WAY_BITS_RAW > 0) ? WAY_BITS_RAW : 1;
    localparam PENULT_WAY      = (NUM_WAYS > 1) ? (NUM_WAYS - 2) : 0;

    // PLRU tree nodes per set: an N-way binary tree requires N - 1 decision bits
    localparam PLRU_NODES      = (NUM_WAYS > 1) ? (NUM_WAYS - 1) : 1;
    localparam NODE_BITS_RAW   = $clog2(PLRU_NODES);
    localparam NODE_BITS       = (NODE_BITS_RAW > 1) ? NODE_BITS_RAW : 2;

    // ------------------------------------------------------------------------
    // 2. Address Slicing
    // ------------------------------------------------------------------------
    // Byte offset within the block
    wire [OFFSET_BITS-1:0] req_offset = i_addr[OFFSET_BITS-1:0];

    // Word index within the block (bits [OFFSET_BITS-1:2])
    wire [WORD_BITS-1:0]   req_word   = (OFFSET_BITS > 2) ? req_offset[OFFSET_BITS-1:2] : 1'b0;

    // Tag from high address bits
    wire [TAG_BITS-1:0]    req_tag    = i_addr[31 : 32 - TAG_BITS];

    // Set index from middle address bits (constant 0 when Fully Associative)
    wire [SET_BITS-1:0]    req_set;
    generate
        if (NUM_SETS > 1) begin : gen_req_set
            assign req_set = i_addr[OFFSET_BITS + SET_BITS_RAW - 1 : OFFSET_BITS];
        end else begin : gen_req_set_fa
            assign req_set = 1'b0;
        end
    endgenerate

    // ------------------------------------------------------------------------
    // 3. Cache Storage Structures
    // ------------------------------------------------------------------------
    // For each set and each way:
    //  - valid : 1 bit indicating the line holds valid data
    //  - dirty : 1 bit indicating the line has been modified by a store
    //  - tags  : TAG_BITS tag memory
    //  - mem   : BLOCK_SIZE * 8 bits data storage
    reg                    valid [0:NUM_SETS-1][0:NUM_WAYS-1];
    reg                    dirty [0:NUM_SETS-1][0:NUM_WAYS-1];
    reg [TAG_BITS-1:0]     tags  [0:NUM_SETS-1][0:NUM_WAYS-1];
    reg [BLOCK_SIZE*8-1:0] mem   [0:NUM_SETS-1][0:NUM_WAYS-1];

    // ------------------------------------------------------------------------
    // 4. Parallel Multi-Way Tag Comparison & Hit Logic
    // ------------------------------------------------------------------------
    // Check all ways in parallel: hit occurs if valid bit is set and tag matches.
    wire [NUM_WAYS-1:0] way_hit;
    genvar gw;
    generate
        for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : gen_way_hit
            assign way_hit[gw] = valid[req_set][gw] && (tags[req_set][gw] == req_tag);
        end
    endgenerate

    wire hit = |way_hit;

    // Priority encoder to identify hit_way (only one way can hit since tags are unique)
    wire [WAY_BITS-1:0] hit_way;
    generate
        if (NUM_WAYS == 1) begin : gen_hit_way_dm
            assign hit_way = 1'b0;
        end else begin : gen_hit_way_mw
            genvar gb_idx, gw_idx;
            for (gb_idx = 0; gb_idx < WAY_BITS; gb_idx = gb_idx + 1) begin : gen_hit_bit
                wire [NUM_WAYS-1:0] bit_matches;
                for (gw_idx = 0; gw_idx < NUM_WAYS; gw_idx = gw_idx + 1) begin : gen_hit_cand
                    localparam [WAY_BITS-1:0] GW_VAL = gw_idx;
                    localparam integer BIT_VAL = GW_VAL[gb_idx];
                    assign bit_matches[gw_idx] = way_hit[gw_idx] && (BIT_VAL == 1);
                end
                assign hit_way[gb_idx] = |bit_matches;
            end
        end
    endgenerate

    // Read word extraction from hit way:
    // Each word is 32 bits = 2^5 bits, so bit offset is {req_word, 5'b00000}.
    wire [BLOCK_SIZE*8-1:0]  hit_line_data    = mem[req_set][hit_way];
    wire [OFFSET_BITS+3-1:0] hit_word_bit_idx = {req_word, 5'b00000};
    wire [31:0]              hit_word         = hit_line_data[hit_word_bit_idx +: 32];

    // ------------------------------------------------------------------------
    // 5. Victim Selection Logic (on a Miss)
    // ------------------------------------------------------------------------
    // Section 4.3 & 4.11 Requirement:
    // "The lowest-numbered invalid way is always used before evicting a valid one."
    wire [NUM_WAYS-1:0] way_invalid;
    genvar giw;
    generate
        for (giw = 0; giw < NUM_WAYS; giw = giw + 1) begin : gen_invalid_bits
            assign way_invalid[giw] = ~valid[req_set][giw];
        end
    endgenerate

    wire has_invalid = |way_invalid;

    // Find the first (lowest-numbered) invalid way
    wire [WAY_BITS-1:0] lowest_invalid_way;
    generate
        if (NUM_WAYS == 1) begin : gen_inv_dm
            assign lowest_invalid_way = 1'b0;
        end else begin : gen_inv_mw
            wire [NUM_WAYS-1:0] first_invalid;
            assign first_invalid[0] = way_invalid[0];
            for (giw = 1; giw < NUM_WAYS; giw = giw + 1) begin : gen_first_inv
                assign first_invalid[giw] = way_invalid[giw] && (~|way_invalid[giw-1:0]);
            end

            genvar gb_i, gw_i;
            for (gb_i = 0; gb_i < WAY_BITS; gb_i = gb_i + 1) begin : gen_inv_bit
                wire [NUM_WAYS-1:0] bit_cands;
                for (gw_i = 0; gw_i < NUM_WAYS; gw_i = gw_i + 1) begin : gen_inv_cand
                    localparam [WAY_BITS-1:0] GW_VAL = gw_i;
                    localparam integer B_VAL = GW_VAL[gb_i];
                    assign bit_cands[gw_i] = first_invalid[gw_i] && (B_VAL == 1);
                end
                assign lowest_invalid_way[gb_i] = |bit_cands;
            end
        end
    endgenerate

    // Replacement Policy 0: True LRU (Least Recently Used)
    // Maintains an order list order[s][0..NUM_WAYS-1] per set:
    //  - order[s][0] is the Most Recently Used (MRU) way.
    //  - order[s][NUM_WAYS-1] is the Least Recently Used (LRU) way.
    reg [WAY_BITS-1:0] order [0:NUM_SETS-1][0:NUM_WAYS-1];
    wire [WAY_BITS-1:0] lru_victim = (NUM_WAYS > 1) ? order[req_set][NUM_WAYS-1] : 1'b0;

    // Replacement Policy 1: PLRU (Pseudo-LRU) Binary Decision Tree
    // Section 4.3.2 & Figure 1:
    //  - Bit value 0 points LEFT; bit value 1 points RIGHT.
    //  - On a miss: Traverses tree without updating to locate victim leaf.
    //  - On hit or fill: Tree nodes along access path flip to point away from accessed way.
    reg [PLRU_NODES-1:0] plru_tree [0:NUM_SETS-1];
    wire [WAY_BITS-1:0]  plru_victim;
    wire [NODE_BITS-1:0] trav_node [0:WAY_BITS-1];

    genvar gd;
    generate
        if (NUM_WAYS <= 1) begin : gen_plru_dm
            assign plru_victim = 1'b0;
        end else begin : gen_plru_mw
            assign trav_node[0] = {NODE_BITS{1'b0}};
            assign plru_victim[WAY_BITS-1] = plru_tree[req_set][0];

            for (gd = 1; gd < WAY_BITS; gd = gd + 1) begin : gen_trav
                wire bit_val = plru_tree[req_set][trav_node[gd-1]];
                assign trav_node[gd] = bit_val ?
                    ((trav_node[gd-1] << 1) + {{(NODE_BITS-2){1'b0}}, 2'd2}) :
                    ((trav_node[gd-1] << 1) + {{(NODE_BITS-1){1'b0}}, 1'd1});
                assign plru_victim[WAY_BITS-1-gd] = plru_tree[req_set][trav_node[gd]];
            end
        end
    endgenerate

    // Overall Victim Selection:
    // Invalid way always takes precedence over replacement policies!
    wire [WAY_BITS-1:0] victim_way = has_invalid ? lowest_invalid_way :
                                     (EVICT_POLICY == 0) ? lru_victim : plru_victim;

    // Check if the chosen victim line is valid and dirty (requires write-back)
    wire victim_is_dirty = valid[req_set][victim_way] && dirty[req_set][victim_way];

    // ------------------------------------------------------------------------
    // 6. Next Replacement State Computation (LRU & PLRU)
    // ------------------------------------------------------------------------
    // Both policies update when an access finishes:
    //  1. Read Hit or Store Hit in STATE_IDLE (access to hit_way)
    //  2. Line Fill in STATE_WAIT_DATA upon i_mem_valid (access to latched_victim_way)
    wire [SET_BITS-1:0] update_set;
    wire [WAY_BITS-1:0] update_way;
    wire                access_update;

    // LRU next order computation:
    // On access to update_way, update_way moves to slot 0 (MRU).
    // Slots that were ahead of update_way shift right by 1; slots behind it stay.
    wire [WAY_BITS-1:0] next_order [0:NUM_SETS-1][0:NUM_WAYS-1];
    genvar s_g, w_g, p_g;
    generate
        for (s_g = 0; s_g < NUM_SETS; s_g = s_g + 1) begin : gen_set_lru
            if (NUM_WAYS > 1) begin : gen_mw_lru
                assign next_order[s_g][0] = update_way;
                for (w_g = 1; w_g < NUM_WAYS; w_g = w_g + 1) begin : gen_lru_slot
                    wire [w_g-1:0] prefix_matches;
                    for (p_g = 0; p_g < w_g; p_g = p_g + 1) begin : gen_pfx
                        assign prefix_matches[p_g] = (order[s_g][p_g] == update_way);
                    end
                    wire found_in_pfx = |prefix_matches;
                    assign next_order[s_g][w_g] = found_in_pfx ? order[s_g][w_g] : order[s_g][w_g-1];
                end
            end else begin : gen_dm_lru
                assign next_order[s_g][0] = 1'b0;
            end
        end
    endgenerate

    // PLRU next tree computation:
    // Identify nodes on path to update_way and flip each to point AWAY from update_way.
    wire [NODE_BITS-1:0] upd_node [0:WAY_BITS-1];
    generate
        if (NUM_WAYS > 1) begin : gen_upd_nodes
            assign upd_node[0] = {NODE_BITS{1'b0}};
            for (gd = 1; gd < WAY_BITS; gd = gd + 1) begin : gen_upd_idx
                assign upd_node[gd] = update_way[WAY_BITS-gd] ?
                    ((upd_node[gd-1] << 1) + {{(NODE_BITS-2){1'b0}}, 2'd2}) :
                    ((upd_node[gd-1] << 1) + {{(NODE_BITS-1){1'b0}}, 1'd1});
            end
        end
    endgenerate

    wire [PLRU_NODES-1:0] next_tree [0:NUM_SETS-1];
    genvar n_g;
    generate
        for (s_g = 0; s_g < NUM_SETS; s_g = s_g + 1) begin : gen_set_plru
            if (NUM_WAYS > 1) begin : gen_mw_tree
                for (n_g = 0; n_g < PLRU_NODES; n_g = n_g + 1) begin : gen_node
                    localparam [NODE_BITS-1:0] NODE_IDX = n_g;
                    wire [WAY_BITS-1:0] node_matches;
                    for (gd = 0; gd < WAY_BITS; gd = gd + 1) begin : gen_match
                        assign node_matches[gd] = (upd_node[gd] == NODE_IDX);
                    end
                    wire is_on_path = |node_matches;

                    wire [WAY_BITS-1:0] new_bit_cand;
                    for (gd = 0; gd < WAY_BITS; gd = gd + 1) begin : gen_cand
                        assign new_bit_cand[gd] = node_matches[gd] && (~update_way[WAY_BITS-1-gd]);
                    end
                    assign next_tree[s_g][n_g] = is_on_path ? (|new_bit_cand) : plru_tree[s_g][n_g];
                end
            end else begin : gen_dm_tree
                assign next_tree[s_g] = 1'b0;
            end
        end
    endgenerate

    // ------------------------------------------------------------------------
    // 7. Write Data & Byte Enable Generators
    // ------------------------------------------------------------------------
    // Store Hit (Write-Back):
    // Merges store bytes into the hit way's block using i_mask and i_wdata.
    wire [BLOCK_SIZE*8-1:0] store_hit_updated_line;
    genvar gb;
    generate
        for (gb = 0; gb < BLOCK_SIZE; gb = gb + 1) begin : gen_hit_bytes
            localparam [WORD_BITS-1:0] GB_WORD = gb / 4;
            localparam [1:0]           GB_LANE = gb % 4;
            wire byte_en = (req_word == GB_WORD) && i_mask[GB_LANE];
            assign store_hit_updated_line[gb*8 +: 8] = byte_en ? i_wdata[GB_LANE*8 +: 8] :
                                                                  hit_line_data[gb*8 +: 8];
        end
    endgenerate

    // Store Miss (Write-Through, No-Allocate):
    // Places store bytes at the appropriate byte positions within a BLOCK_SIZE line,
    // and sets byte enables only for the modified bytes.
    wire [BLOCK_SIZE*8-1:0] store_miss_mem_wr_data;
    wire [BLOCK_SIZE-1:0]   store_miss_mem_wr_be;
    generate
        for (gb = 0; gb < BLOCK_SIZE; gb = gb + 1) begin : gen_sm_bytes
            localparam [WORD_BITS-1:0] GB_WORD = gb / 4;
            localparam [1:0]           GB_LANE = gb % 4;
            wire byte_en = (req_word == GB_WORD) && i_mask[GB_LANE];
            assign store_miss_mem_wr_be[gb] = byte_en;
            assign store_miss_mem_wr_data[gb*8 +: 8] = byte_en ? i_wdata[GB_LANE*8 +: 8] : 8'h00;
        end
    endgenerate

    // ------------------------------------------------------------------------
    // 8. FSM States & Miss Registers
    // ------------------------------------------------------------------------
    localparam STATE_IDLE      = 2'd0; // Servicing hits or detecting misses
    localparam STATE_DIRTY_WB  = 2'd1; // Writing back dirty victim block to main memory
    localparam STATE_READ_REQ  = 2'd2; // Pulsing memory read request for missed line
    localparam STATE_WAIT_DATA = 2'd3; // Waiting for i_mem_valid from memory

    reg [1:0] state;

    // Registers to latch miss context across memory stall cycles
    reg [31:0]             latched_addr;
    reg [WAY_BITS-1:0]     latched_victim_way;
    reg [TAG_BITS-1:0]     latched_victim_tag;
    reg [BLOCK_SIZE*8-1:0] latched_victim_data;

    // ------------------------------------------------------------------------
// Prefetch context
// ------------------------------------------------------------------------
reg                  prefetch_pending;
reg                  prefetch_issued;
reg [31:0]           prefetch_addr;
reg [SET_BITS-1:0]   prefetch_set;
reg [WAY_BITS-1:0]   prefetch_victim_way;
reg [TAG_BITS-1:0]   prefetch_tag;
reg [TAG_BITS-1:0]   prefetch_victim_tag;
reg [BLOCK_SIZE*8-1:0] prefetch_victim_data;
reg                  prefetch_victim_dirty;
reg                  prefetch_wb_issued;

// ------------------------------------------------------------------------
// Next-line prefetch address
// ------------------------------------------------------------------------
wire [31:0] latched_block_addr =
    {latched_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

wire [31:0] next_line_addr =
    latched_block_addr + BLOCK_SIZE_BYTES;

// Current CPU request's next-line address.
wire [31:0] current_next_line_addr =
    {i_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}} + BLOCK_SIZE_BYTES;
wire [TAG_BITS-1:0] current_next_line_tag =
    current_next_line_addr[31 : 32 - TAG_BITS];
wire [SET_BITS-1:0] current_next_line_set;
generate
    if (NUM_SETS > 1) begin : gen_current_next_set
        assign current_next_line_set =
            current_next_line_addr[OFFSET_BITS + SET_BITS_RAW - 1 : OFFSET_BITS];
    end else begin : gen_current_next_set_fa
        assign current_next_line_set = 1'b0;
    end
endgenerate

    // ------------------------------------------------------------------------
// Prefetch address decomposition
// ------------------------------------------------------------------------
wire [TAG_BITS-1:0] prefetch_req_tag =
    next_line_addr[31 : 32 - TAG_BITS];

wire [SET_BITS-1:0] prefetch_req_set;

generate
    if (NUM_SETS > 1) begin : gen_prefetch_set
        assign prefetch_req_set =
            next_line_addr[OFFSET_BITS + SET_BITS_RAW - 1 : OFFSET_BITS];
    end else begin : gen_prefetch_set_fa
        assign prefetch_req_set = 1'b0;
    end
endgenerate

// ------------------------------------------------------------------------
// Check whether the next line is already in the cache
// ------------------------------------------------------------------------
wire [NUM_WAYS-1:0] prefetch_way_hit;

genvar pw;
generate
    for (pw = 0; pw < NUM_WAYS; pw = pw + 1) begin : gen_prefetch_way_hit
        assign prefetch_way_hit[pw] =
            valid[current_next_line_set][pw] &&
            (tags[current_next_line_set][pw] == current_next_line_tag);
    end
endgenerate

// ------------------------------------------------------------------------
// Prefetch invalid-way detection
// ------------------------------------------------------------------------
wire [NUM_WAYS-1:0] prefetch_invalid_way;

genvar pi;
generate
    for (pi = 0; pi < NUM_WAYS; pi = pi + 1) begin : gen_prefetch_invalid
        assign prefetch_invalid_way[pi] =
            !valid[current_next_line_set][pi];
    end
endgenerate

wire prefetch_has_invalid = |prefetch_invalid_way;

// Lowest invalid way for the prefetch target set.
wire [WAY_BITS-1:0] prefetch_lowest_invalid_way;
generate
    if (NUM_WAYS == 1) begin : gen_prefetch_inv_dm
        assign prefetch_lowest_invalid_way = 1'b0;
    end else begin : gen_prefetch_inv_mw
        wire [NUM_WAYS-1:0] prefetch_first_invalid;
        assign prefetch_first_invalid[0] = prefetch_invalid_way[0];
        for (pi = 1; pi < NUM_WAYS; pi = pi + 1) begin : gen_prefetch_first_inv
            assign prefetch_first_invalid[pi] =
                prefetch_invalid_way[pi] && (~|prefetch_invalid_way[pi-1:0]);
        end
        genvar pgb_i, pgw_i;
        for (pgb_i = 0; pgb_i < WAY_BITS; pgb_i = pgb_i + 1) begin : gen_prefetch_inv_bit
            wire [NUM_WAYS-1:0] prefetch_bit_cands;
            for (pgw_i = 0; pgw_i < NUM_WAYS; pgw_i = pgw_i + 1) begin : gen_prefetch_inv_cand
                localparam [WAY_BITS-1:0] PGW_VAL = pgw_i;
                localparam integer PIV_VAL = PGW_VAL[pgb_i];
                assign prefetch_bit_cands[pgw_i] =
                    prefetch_first_invalid[pgw_i] && (PIV_VAL == 1);
            end
            assign prefetch_lowest_invalid_way[pgb_i] = |prefetch_bit_cands;
        end
    end
endgenerate

wire prefetch_hit = |prefetch_way_hit;

    wire [WAY_BITS-1:0] prefetch_lru_victim =
        (NUM_WAYS > 1) ? order[current_next_line_set][NUM_WAYS-1] : 1'b0;

    wire [WAY_BITS-1:0] prefetch_plru_victim;
    wire [NODE_BITS-1:0] prefetch_trav_node [0:WAY_BITS-1];

    genvar ptd;
    generate
        if (NUM_WAYS <= 1) begin : gen_prefetch_plru_dm
            assign prefetch_plru_victim = 1'b0;
        end else begin : gen_prefetch_plru_mw
            assign prefetch_trav_node[0] = {NODE_BITS{1'b0}};
            assign prefetch_plru_victim[WAY_BITS-1] = plru_tree[current_next_line_set][0];
            for (ptd = 1; ptd < WAY_BITS; ptd = ptd + 1) begin : gen_prefetch_trav
                wire pbit = plru_tree[current_next_line_set][prefetch_trav_node[ptd-1]];
                assign prefetch_trav_node[ptd] = pbit ?
                    ((prefetch_trav_node[ptd-1] << 1) + {{(NODE_BITS-2){1'b0}}, 2'd2}) :
                    ((prefetch_trav_node[ptd-1] << 1) + {{(NODE_BITS-1){1'b0}}, 1'd1});
                assign prefetch_plru_victim[WAY_BITS-1-ptd] =
                    plru_tree[current_next_line_set][prefetch_trav_node[ptd]];
            end
        end
    endgenerate

    wire [WAY_BITS-1:0] prefetch_victim_way_now =
        prefetch_has_invalid ? prefetch_lowest_invalid_way :
        (EVICT_POLICY == 0) ? prefetch_lru_victim : prefetch_plru_victim;

    // Sliced fields from latched_addr
    wire [OFFSET_BITS-1:0] latched_offset = latched_addr[OFFSET_BITS-1:0];
    wire [WORD_BITS-1:0]   latched_word   = (OFFSET_BITS > 2) ? latched_offset[OFFSET_BITS-1:2] : 1'b0;
    wire [TAG_BITS-1:0]    latched_tag    = latched_addr[31 : 32 - TAG_BITS];

    wire [SET_BITS-1:0]    latched_set;
    generate
        if (NUM_SETS > 1) begin : gen_latched_set
            assign latched_set = latched_addr[OFFSET_BITS + SET_BITS_RAW - 1 : OFFSET_BITS];
        end else begin : gen_latched_set_fa
            assign latched_set = 1'b0;
        end
    endgenerate

    // Reconstruct memory block address of the evicted dirty line
    wire [31:0] latched_evict_addr;
    generate
        if (NUM_SETS > 1) begin : gen_latched_evict_addr
            assign latched_evict_addr = {latched_victim_tag, latched_set, {OFFSET_BITS{1'b0}}};
        end else begin : gen_latched_evict_addr_fa
            assign latched_evict_addr = {latched_victim_tag, {OFFSET_BITS{1'b0}}};
        end
    endgenerate

    // Extract word requested by CPU from memory read response
    wire [OFFSET_BITS+3-1:0] latched_word_bit_idx = {latched_word, 5'b00000};
    wire [31:0]              mem_word             = i_mem_rd_data[latched_word_bit_idx +: 32];

    // A prefetch response has priority over a CPU access in the same cycle.
    wire prefetch_response = (state == STATE_IDLE) && prefetch_pending && prefetch_issued && i_mem_valid;
wire prefetch_block_cpu = (state == STATE_IDLE) && prefetch_pending && !prefetch_response &&
                          ((i_ren && !hit) || (i_wen && !hit));

    assign access_update = prefetch_response ||
                           (state == STATE_IDLE && (i_ren || i_wen) && hit) ||
                           (state == STATE_WAIT_DATA && i_mem_valid);
    assign update_set    = prefetch_response ? prefetch_set :
                           (state == STATE_WAIT_DATA) ? latched_set : req_set;
    assign update_way    = prefetch_response ? prefetch_victim_way :
                           (state == STATE_WAIT_DATA) ? latched_victim_way : hit_way;

    // ------------------------------------------------------------------------
    // 9. CPU & Memory Interface Output Assignments
    // ------------------------------------------------------------------------
    // CPU Interface:
    //  - o_ready : High when ready to accept/complete access.
    //              * In STATE_IDLE: high if no access, if hit, or if store miss (write-through no stall)
    //              * In STATE_WAIT_DATA: high on the cycle i_mem_valid arrives
    //              * Otherwise: low (stalls the pipeline)
    assign o_ready = (state == STATE_IDLE) ?
                        (prefetch_response || prefetch_block_cpu ? 1'b0 : (!i_ren || hit || i_wen)) :
                     (state == STATE_WAIT_DATA) ? i_mem_valid :
                     1'b0;

    //  - o_rdata : Multiplexed word returned to CPU.
    //              * In STATE_WAIT_DATA: word extracted directly from arriving memory line
    //              * In STATE_IDLE: word extracted from the hit way
    assign o_rdata = (state == STATE_WAIT_DATA) ? mem_word : hit_word;

    // Memory Line Read Interface:
    //  - o_mem_rd : 1-cycle pulse requesting a line from memory.
    //               * Clean miss: issued directly in STATE_IDLE
    //               * Dirty miss: issued in STATE_READ_REQ (after dirty write-back completes)
    assign o_mem_rd =
        (state == STATE_IDLE && !prefetch_response && !prefetch_block_cpu && i_ren && !hit && !victim_is_dirty) ||
        (state == STATE_READ_REQ) ||
        (state == STATE_WAIT_DATA && prefetch_pending && !prefetch_issued);

    //  - o_mem_rd_addr : Line-aligned address of requested block
    assign o_mem_rd_addr =
        (state == STATE_WAIT_DATA && prefetch_pending && !prefetch_issued) ? prefetch_addr :
        (state == STATE_READ_REQ) ?
            {latched_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}} :
            {i_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

    // Memory Line Write Interface:
    //  - o_mem_wr : 1-cycle pulse to write a line to memory.
    //               * Dirty write-back in STATE_DIRTY_WB
    //               * Store miss write-through in STATE_IDLE
    assign o_mem_wr = (state == STATE_DIRTY_WB) ||
                      (state == STATE_WAIT_DATA && prefetch_pending && prefetch_issued &&
                       prefetch_victim_dirty && !prefetch_wb_issued) ||
                      (state == STATE_IDLE && !prefetch_response && !prefetch_block_cpu && i_wen && !hit);

    //  - o_mem_wr_addr : Target line address
    assign o_mem_wr_addr = (state == STATE_DIRTY_WB) ?
                               latched_evict_addr :
                           (state == STATE_WAIT_DATA && prefetch_pending && prefetch_issued &&
                            prefetch_victim_dirty && !prefetch_wb_issued) ?
                               {prefetch_victim_tag, prefetch_set, {OFFSET_BITS{1'b0}}} :
                               {i_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

    //  - o_mem_wr_data : Block data written to memory
    assign o_mem_wr_data = (state == STATE_DIRTY_WB) ?
                               latched_victim_data :
                           (state == STATE_WAIT_DATA && prefetch_pending && prefetch_issued &&
                            prefetch_victim_dirty && !prefetch_wb_issued) ?
                               prefetch_victim_data :
                               store_miss_mem_wr_data;

    //  - o_mem_wr_be : One bit per byte. All ones for dirty write-back,
    //                  or selective bytes for write-through.
    assign o_mem_wr_be = (state == STATE_DIRTY_WB) ?
                             {BLOCK_SIZE{1'b1}} :
                         (state == STATE_WAIT_DATA && prefetch_pending && prefetch_issued &&
                          prefetch_victim_dirty && !prefetch_wb_issued) ?
                             {BLOCK_SIZE{1'b1}} :
                             store_miss_mem_wr_be;


    // ------------------------------------------------------------------------
    // 10. Sequential Logic (Synchronous Single-Clock Reset)
    // ------------------------------------------------------------------------
    integer s_idx, w_idx;

    always @(posedge i_clk) begin
        if (i_rst) begin
            // Synchronous Reset: invalidate all lines, clear dirty bits and FSM
            state                <= STATE_IDLE;
            latched_addr         <= 32'd0;
            latched_victim_way   <= {WAY_BITS{1'b0}};
            latched_victim_tag   <= {TAG_BITS{1'b0}};
            latched_victim_data  <= {(BLOCK_SIZE*8){1'b0}};
            prefetch_pending     <= 1'b0;
            prefetch_issued      <= 1'b0;
            prefetch_addr        <= 32'd0;
            prefetch_set         <= {SET_BITS{1'b0}};
            prefetch_victim_way  <= {WAY_BITS{1'b0}};
            prefetch_tag         <= {TAG_BITS{1'b0}};
            prefetch_victim_tag  <= {TAG_BITS{1'b0}};
            prefetch_victim_data <= {(BLOCK_SIZE*8){1'b0}};
            prefetch_victim_dirty <= 1'b0;
            prefetch_wb_issued   <= 1'b0;

            for (s_idx = 0; s_idx < NUM_SETS; s_idx = s_idx + 1) begin
                for (w_idx = 0; w_idx < NUM_WAYS; w_idx = w_idx + 1) begin
                    valid[s_idx][w_idx] <= 1'b0;
                    dirty[s_idx][w_idx] <= 1'b0;
                    tags[s_idx][w_idx]  <= {TAG_BITS{1'b0}};
                    order[s_idx][w_idx] <= w_idx[WAY_BITS-1:0];
                end
                plru_tree[s_idx] <= {PLRU_NODES{1'b0}};
            end
        end else begin
            // ----------------------------------------------------------------
            // Replacement Policy Updates on Access (Hit or Fill)
            // ----------------------------------------------------------------
            if (access_update) begin
                if (EVICT_POLICY == 0) begin
                    for (w_idx = 0; w_idx < NUM_WAYS; w_idx = w_idx + 1) begin
                        order[update_set][w_idx] <= next_order[update_set][w_idx];
                    end
                end else begin
                    plru_tree[update_set] <= next_tree[update_set];
                end
            end

            // ----------------------------------------------------------------
            // Main Cache Controller FSM
            // ----------------------------------------------------------------
            case (state)
                STATE_IDLE: begin
                    if (prefetch_response) begin
                        // Prefetch response has priority over the CPU for this cycle.
                        // If the selected victim became dirty while the request was
                        // outstanding, discard the prefetched line rather than losing data.
                        if (!(valid[prefetch_set][prefetch_victim_way] &&
                              dirty[prefetch_set][prefetch_victim_way])) begin
                            mem[prefetch_set][prefetch_victim_way]   <= i_mem_rd_data;
                            tags[prefetch_set][prefetch_victim_way]  <= prefetch_tag;
                            valid[prefetch_set][prefetch_victim_way] <= 1'b1;
                            dirty[prefetch_set][prefetch_victim_way] <= 1'b0;
                        end
                        prefetch_pending <= 1'b0;
                        prefetch_issued  <= 1'b0;
                        prefetch_victim_dirty <= 1'b0;
                        prefetch_wb_issued <= 1'b0;
                        state <= STATE_IDLE;
                    end else if (i_wen && hit) begin
                        // Store Hit: Write-back policy
                        // Update cache line, mark as dirty, no main memory write.
                        mem[req_set][hit_way]   <= store_hit_updated_line;
                        dirty[req_set][hit_way] <= 1'b1;
                        state                   <= STATE_IDLE;
                    end else if (i_wen && !hit) begin
                        // Store Miss: Write-through, no-allocate
                        // Directly forwarded to memory via o_mem_wr; cache unchanged.
                        state <= STATE_IDLE;
                    end else if (i_ren && !hit) begin
                        // Read Miss: Must allocate a block in the set
                        latched_addr       <= i_addr;
                        latched_victim_way <= victim_way;

                        if (PREFETCH_EN && !prefetch_hit) begin
                            prefetch_pending    <= 1'b1;
                            prefetch_issued     <= 1'b0;
                            prefetch_addr       <= current_next_line_addr;
                            prefetch_set        <= current_next_line_set;
                            prefetch_victim_way <= prefetch_victim_way_now;
                            prefetch_tag        <= current_next_line_tag;
                            prefetch_victim_tag <= tags[current_next_line_set][prefetch_victim_way_now];
                            prefetch_victim_data <= mem[current_next_line_set][prefetch_victim_way_now];
                            prefetch_victim_dirty <=
                                valid[current_next_line_set][prefetch_victim_way_now] &&
                                dirty[current_next_line_set][prefetch_victim_way_now];
                            prefetch_wb_issued <= 1'b0;
                        end else begin
                            prefetch_pending    <= 1'b0;
                            prefetch_issued     <= 1'b0;
                            prefetch_victim_dirty <= 1'b0;
                            prefetch_wb_issued <= 1'b0;
                        end

                        if (victim_is_dirty) begin
                            // Dirty line must be written back to memory first
                            latched_victim_tag  <= tags[req_set][victim_way];
                            latched_victim_data <= mem[req_set][victim_way];
                            state               <= STATE_DIRTY_WB;
                        end else begin
                            // Clean line: o_mem_rd is asserted this cycle
                            state <= STATE_WAIT_DATA;
                        end
                    end else begin
                        // Read hit or Idle: no change
                        state <= STATE_IDLE;
                    end
                end

                STATE_DIRTY_WB: begin
                    // One-cycle write pulse has been delivered; now issue read request
                    state <= STATE_READ_REQ;
                end

                STATE_READ_REQ: begin
                    // One-cycle read pulse has been delivered; now await memory line
                    state <= STATE_WAIT_DATA;
                end

                STATE_WAIT_DATA: begin
                    // The prefetch read is issued exactly one cycle after the
                    // demand read request. It is a one-cycle pulse.
                    if (prefetch_pending && !prefetch_issued) begin
                        prefetch_issued <= 1'b1;
                    end else if (prefetch_pending && prefetch_issued &&
                                 prefetch_victim_dirty && !prefetch_wb_issued) begin
                        // The prefetch read was launched in the previous cycle.
                        // Write its dirty victim back before the prefetch response
                        // installs the new line.
                        prefetch_wb_issued <= 1'b1;
                    end

                    if (i_mem_valid) begin
                        // Requested line has arrived from main memory!
                        // Store line, set valid, clear dirty, set tag
                        mem[latched_set][latched_victim_way]   <= i_mem_rd_data;
                        tags[latched_set][latched_victim_way]  <= latched_tag;
                        valid[latched_set][latched_victim_way] <= 1'b1;
                        dirty[latched_set][latched_victim_way] <= 1'b0;

                        // For a fully-associative cache, the demand fill and the
                        // prefetch target share the same set. If they selected the
                        // same way, redirect the prefetch to another way.
                        if (prefetch_pending && (prefetch_set == latched_set) &&
                            (prefetch_victim_way == latched_victim_way) && (NUM_WAYS > 1)) begin
                            if (prefetch_invalid_way[0] && (latched_victim_way != 0))
                                prefetch_victim_way <= 0;
                            else
                                prefetch_victim_way <= (order[latched_set][NUM_WAYS-1] == latched_victim_way) ?
                                                       order[latched_set][PENULT_WAY] :
                                                       order[latched_set][NUM_WAYS-1];
                        end

                        // Return to IDLE for next instruction
                        state <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
