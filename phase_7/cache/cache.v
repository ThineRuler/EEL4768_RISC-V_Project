`default_nettype none

// Non-blocking set-associative L1 cache (Hit-Under-Miss + 4-entry Miss/Prefetch Queue).
// Preserves Phase 5 replacement (LRU / PLRU, lowest-invalid-first) and write policy
// (write-back on store hit, write-through no-allocate on store miss), while:
//  - Serving read/store hits immediately under in-flight misses (Hit-Under-Miss)
//  - Overlapping multiple misses + non-blocking next-line prefetch via memq
//  - Exposing o_miss_accepted and o_fill_* for non-blocking OoO LSU wakeup
module cache #(
    parameter EVICT_POLICY = 0,
    parameter NUM_WAYS     = 1,
    parameter CACHE_SIZE   = 16,
    parameter BLOCK_SIZE   = 128,
    parameter PREFETCH_EN  = 0
) (
    input  wire                    i_clk,
    input  wire                    i_rst,

    input  wire                    i_ren,
    input  wire                    i_wen,
    input  wire [31:0]             i_addr,
    input  wire [ 3:0]             i_mask,
    input  wire [31:0]             i_wdata,
    output wire                    o_ready,
    output wire [31:0]             o_rdata,

    // Non-blocking MSHR / fill notification ports for OoO LSU
    output wire                    o_miss_accepted,
    output wire                    o_fill_valid,
    output wire [31:0]             o_fill_addr,
    output wire [BLOCK_SIZE*8-1:0] o_fill_data,

    output wire                    o_mem_rd,
    output wire [31:0]             o_mem_rd_addr,
    input  wire                    i_mem_ready,
    input  wire                    i_mem_valid,
    input  wire [BLOCK_SIZE*8-1:0] i_mem_rd_data,
    output wire                    o_mem_wr,
    output wire [31:0]             o_mem_wr_addr,
    output wire [BLOCK_SIZE*8-1:0] o_mem_wr_data,
    output wire [BLOCK_SIZE-1:0]   o_mem_wr_be
);

    // ------------------------------------------------------------------------
    // 1. Geometry Calculations & Local Parameters
    // ------------------------------------------------------------------------
    localparam TOTAL_BYTES     = CACHE_SIZE * 1024;
    localparam TOTAL_BLOCKS    = TOTAL_BYTES / BLOCK_SIZE;
    localparam NUM_SETS        = TOTAL_BLOCKS / NUM_WAYS;
    localparam [31:0] BLOCK_SIZE_BYTES = BLOCK_SIZE;

    localparam OFFSET_BITS     = $clog2(BLOCK_SIZE);
    localparam SET_BITS_RAW    = $clog2(NUM_SETS);
    localparam SET_BITS        = (SET_BITS_RAW > 0) ? SET_BITS_RAW : 1;
    localparam TAG_BITS        = 32 - SET_BITS_RAW - OFFSET_BITS;

    localparam WORD_BITS       = (OFFSET_BITS > 2) ? (OFFSET_BITS - 2) : 1;

    localparam WAY_BITS_RAW    = $clog2(NUM_WAYS);
    localparam WAY_BITS        = (WAY_BITS_RAW > 0) ? WAY_BITS_RAW : 1;

    localparam PLRU_NODES      = (NUM_WAYS > 1) ? (NUM_WAYS - 1) : 1;
    localparam NODE_BITS_RAW   = $clog2(PLRU_NODES);
    localparam NODE_BITS       = (NODE_BITS_RAW > 1) ? NODE_BITS_RAW : 2;

    localparam MEMQ_SIZE       = 4;

    // ------------------------------------------------------------------------
    // 2. Address Slicing (Current CPU Request)
    // ------------------------------------------------------------------------
    wire [OFFSET_BITS-1:0] req_offset   = i_addr[OFFSET_BITS-1:0];
    wire [WORD_BITS-1:0]   req_word     = (OFFSET_BITS > 2) ? req_offset[OFFSET_BITS-1:2] : 1'b0;
    wire [TAG_BITS-1:0]    req_tag      = i_addr[31 : 32 - TAG_BITS];
    wire [31:0]            req_blk_addr = {i_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

    wire [SET_BITS-1:0] req_set;
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
    reg                    valid [0:NUM_SETS-1][0:NUM_WAYS-1];
    reg                    dirty [0:NUM_SETS-1][0:NUM_WAYS-1];
    reg [TAG_BITS-1:0]     tags  [0:NUM_SETS-1][0:NUM_WAYS-1];
    reg [BLOCK_SIZE*8-1:0] mem   [0:NUM_SETS-1][0:NUM_WAYS-1];

    reg [WAY_BITS-1:0]     order     [0:NUM_SETS-1][0:NUM_WAYS-1];
    reg [PLRU_NODES-1:0]   plru_tree [0:NUM_SETS-1];

    // ------------------------------------------------------------------------
    // 4. Parallel Multi-Way Tag Comparison & Hit Logic
    // ------------------------------------------------------------------------
    wire [NUM_WAYS-1:0] way_hit;
    genvar gw;
    generate
        for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : gen_way_hit
            assign way_hit[gw] = valid[req_set][gw] && (tags[req_set][gw] == req_tag);
        end
    endgenerate

    wire hit = |way_hit;

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

    wire [BLOCK_SIZE*8-1:0]  hit_line_data    = mem[req_set][hit_way];
    wire [OFFSET_BITS+3-1:0] hit_word_bit_idx = {req_word, 5'b00000};
    wire [31:0]              hit_word         = hit_line_data[hit_word_bit_idx +: 32];

    // ------------------------------------------------------------------------
    // 5. In-Flight Memory Read FIFO (MSHR + Prefetch Queue, 4 entries)
    // ------------------------------------------------------------------------
    reg        memq_valid   [0:MEMQ_SIZE-1];
    reg [31:0] memq_addr    [0:MEMQ_SIZE-1];
    reg [ 1:0] memq_head, memq_tail;
    reg [ 2:0] memq_count;

    // Pending 1-entry next-line prefetch request (scheduled whenever mem_rd is idle)
    reg        pref_req_pending;
    reg [31:0] pref_req_addr;

    wire [MEMQ_SIZE-1:0] req_memq_match;
    genvar mq;
    generate
        for (mq = 0; mq < MEMQ_SIZE; mq = mq + 1) begin : gen_req_memq
            assign req_memq_match[mq] = memq_valid[mq] && (memq_addr[mq] == req_blk_addr);
        end
    endgenerate
    wire req_in_memq = |req_memq_match;

    // ------------------------------------------------------------------------
    // 6. Fill Handling (When i_mem_valid Arrives for memq[memq_head])
    // ------------------------------------------------------------------------
    wire        fill_now  = i_mem_valid && (memq_count != 3'd0);
    wire [31:0] fill_addr = memq_addr[memq_head];
    wire [TAG_BITS-1:0] fill_tag = fill_addr[31 : 32 - TAG_BITS];
    wire [SET_BITS-1:0] fill_set;
    generate
        if (NUM_SETS > 1) begin : gen_fill_set
            assign fill_set = fill_addr[OFFSET_BITS + SET_BITS_RAW - 1 : OFFSET_BITS];
        end else begin : gen_fill_set_fa
            assign fill_set = 1'b0;
        end
    endgenerate

    // Check if fill_addr already lives in cache (guard against duplicate install)
    wire [NUM_WAYS-1:0] fill_dup_hit;
    generate
        for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : gen_fill_dup
            assign fill_dup_hit[gw] = valid[fill_set][gw] && (tags[fill_set][gw] == fill_tag);
        end
    endgenerate
    wire fill_already_cached = |fill_dup_hit;

    // Fill-time victim selection on fill_set
    wire [NUM_WAYS-1:0] fill_way_invalid;
    genvar giw;
    generate
        for (giw = 0; giw < NUM_WAYS; giw = giw + 1) begin : gen_fill_inv_bits
            assign fill_way_invalid[giw] = ~valid[fill_set][giw];
        end
    endgenerate
    wire fill_has_invalid = |fill_way_invalid;

    wire [WAY_BITS-1:0] fill_lowest_invalid_way;
    generate
        if (NUM_WAYS == 1) begin : gen_fill_inv_dm
            assign fill_lowest_invalid_way = 1'b0;
        end else begin : gen_fill_inv_mw
            wire [NUM_WAYS-1:0] fill_first_invalid;
            assign fill_first_invalid[0] = fill_way_invalid[0];
            for (giw = 1; giw < NUM_WAYS; giw = giw + 1) begin : gen_fill_first_inv
                assign fill_first_invalid[giw] = fill_way_invalid[giw] && (~|fill_way_invalid[giw-1:0]);
            end

            genvar gb_i, gw_i;
            for (gb_i = 0; gb_i < WAY_BITS; gb_i = gb_i + 1) begin : gen_fill_inv_bit
                wire [NUM_WAYS-1:0] bit_cands;
                for (gw_i = 0; gw_i < NUM_WAYS; gw_i = gw_i + 1) begin : gen_fill_inv_cand
                    localparam [WAY_BITS-1:0] GW_VAL = gw_i;
                    localparam integer B_VAL = GW_VAL[gb_i];
                    assign bit_cands[gw_i] = fill_first_invalid[gw_i] && (B_VAL == 1);
                end
                assign fill_lowest_invalid_way[gb_i] = |bit_cands;
            end
        end
    endgenerate

    wire [WAY_BITS-1:0]  fill_lru_victim = (NUM_WAYS > 1) ? order[fill_set][NUM_WAYS-1] : 1'b0;
    wire [WAY_BITS-1:0]  fill_plru_victim;
    wire [NODE_BITS-1:0] fill_trav_node [0:WAY_BITS-1];

    genvar gd;
    generate
        if (NUM_WAYS <= 1) begin : gen_fill_plru_dm
            assign fill_plru_victim = 1'b0;
        end else begin : gen_fill_plru_mw
            assign fill_trav_node[0] = {NODE_BITS{1'b0}};
            assign fill_plru_victim[WAY_BITS-1] = plru_tree[fill_set][0];
            for (gd = 1; gd < WAY_BITS; gd = gd + 1) begin : gen_fill_trav
                wire bit_val = plru_tree[fill_set][fill_trav_node[gd-1]];
                assign fill_trav_node[gd] = bit_val ?
                    ((fill_trav_node[gd-1] << 1) + {{(NODE_BITS-2){1'b0}}, 2'd2}) :
                    ((fill_trav_node[gd-1] << 1) + {{(NODE_BITS-1){1'b0}}, 1'd1});
                assign fill_plru_victim[WAY_BITS-1-gd] = plru_tree[fill_set][fill_trav_node[gd]];
            end
        end
    endgenerate

    wire [WAY_BITS-1:0] fill_victim_way =
        fill_has_invalid    ? fill_lowest_invalid_way :
        (EVICT_POLICY == 0) ? fill_lru_victim : fill_plru_victim;

    wire fill_install_en   = fill_now && !fill_already_cached;
    wire fill_victim_dirty = fill_install_en && valid[fill_set][fill_victim_way] && dirty[fill_set][fill_victim_way];
    wire [TAG_BITS-1:0] fill_victim_tag = tags[fill_set][fill_victim_way];
    wire [BLOCK_SIZE*8-1:0] fill_victim_data = mem[fill_set][fill_victim_way];

    wire [31:0] fill_evict_addr;
    generate
        if (NUM_SETS > 1) begin : gen_fill_evict_addr
            assign fill_evict_addr = {fill_victim_tag, fill_set, {OFFSET_BITS{1'b0}}};
        end else begin : gen_fill_evict_addr_fa
            assign fill_evict_addr = {fill_victim_tag, {OFFSET_BITS{1'b0}}};
        end
    endgenerate

    wire [31:0] fill_word = i_mem_rd_data[hit_word_bit_idx +: 32];
    wire        fill_matches_req = fill_now && (fill_addr == req_blk_addr);

    // ------------------------------------------------------------------------
    // 7. Next-Line Prefetch Eligibility Check
    // ------------------------------------------------------------------------
    wire [31:0] next_line_addr = req_blk_addr + BLOCK_SIZE_BYTES;
    wire [TAG_BITS-1:0] next_line_tag = next_line_addr[31 : 32 - TAG_BITS];
    wire [SET_BITS-1:0] next_line_set;
    generate
        if (NUM_SETS > 1) begin : gen_next_set
            assign next_line_set = next_line_addr[OFFSET_BITS + SET_BITS_RAW - 1 : OFFSET_BITS];
        end else begin : gen_next_set_fa
            assign next_line_set = 1'b0;
        end
    endgenerate

    wire [NUM_WAYS-1:0] next_line_cache_hit;
    generate
        for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : gen_nl_cache
            assign next_line_cache_hit[gw] = valid[next_line_set][gw] && (tags[next_line_set][gw] == next_line_tag);
        end
    endgenerate
    wire [MEMQ_SIZE-1:0] next_line_memq_hit;
    generate
        for (mq = 0; mq < MEMQ_SIZE; mq = mq + 1) begin : gen_nl_memq
            assign next_line_memq_hit[mq] = memq_valid[mq] && (memq_addr[mq] == next_line_addr);
        end
    endgenerate
    wire next_line_present = (|next_line_cache_hit) || (|next_line_memq_hit);

    // Check if pending prefetch address is still absent from cache & memq
    wire [TAG_BITS-1:0] pref_tag = pref_req_addr[31 : 32 - TAG_BITS];
    wire [SET_BITS-1:0] pref_set;
    generate
        if (NUM_SETS > 1) begin : gen_pref_set
            assign pref_set = pref_req_addr[OFFSET_BITS + SET_BITS_RAW - 1 : OFFSET_BITS];
        end else begin : gen_pref_set_fa
            assign pref_set = 1'b0;
        end
    endgenerate
    wire [NUM_WAYS-1:0] pref_cache_hit;
    generate
        for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : gen_pref_cache
            assign pref_cache_hit[gw] = valid[pref_set][gw] && (tags[pref_set][gw] == pref_tag);
        end
    endgenerate
    wire [MEMQ_SIZE-1:0] pref_memq_hit;
    generate
        for (mq = 0; mq < MEMQ_SIZE; mq = mq + 1) begin : gen_pref_memq
            assign pref_memq_hit[mq] = memq_valid[mq] && (memq_addr[mq] == pref_req_addr);
        end
    endgenerate
    wire pref_still_needed = pref_req_pending && !(|pref_cache_hit) && !(|pref_memq_hit);

    // ------------------------------------------------------------------------
    // 8. Replacement State Update Calculations (LRU & PLRU)
    // ------------------------------------------------------------------------
    // Fill update takes priority on the rare cycle both fill and CPU hit occur in same set.
    wire        cpu_hit_ready  = ((i_ren && hit) || (i_wen && hit && !(fill_install_en && (req_set == fill_set))));
    wire        access_update  = fill_install_en || cpu_hit_ready;
    wire [SET_BITS-1:0] update_set = fill_install_en ? fill_set        : req_set;
    wire [WAY_BITS-1:0] update_way = fill_install_en ? fill_victim_way : hit_way;

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
    // 9. Write Data & Byte Enable Generators
    // ------------------------------------------------------------------------
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
    // 10. CPU & Memory Interface Signals
    // ------------------------------------------------------------------------
    wire memq_has_space   = (memq_count < 3'd4) && i_mem_ready;
    wire demand_miss_new  = i_ren && !hit && !req_in_memq && memq_has_space;
    wire pref_issue_now   = !demand_miss_new && PREFETCH_EN && pref_still_needed && memq_has_space;

    wire store_hit_ready  = i_wen && hit  && !(fill_install_en && (req_set == fill_set));
    wire store_miss_ready = i_wen && !hit && !req_in_memq && !fill_victim_dirty && i_mem_ready;

    assign o_ready = (!i_ren && !i_wen) ? 1'b1 :
                     (i_ren && hit)     ? 1'b1 :
                     (i_ren && fill_matches_req) ? 1'b1 :
                     store_hit_ready    ? 1'b1 :
                     store_miss_ready   ? 1'b1 :
                                          1'b0;

    assign o_rdata = (i_ren && fill_matches_req && !hit) ? fill_word : hit_word;

    assign o_miss_accepted = i_ren && !hit && !fill_matches_req && (req_in_memq || memq_has_space);
    assign o_fill_valid    = fill_now;
    assign o_fill_addr     = fill_addr;
    assign o_fill_data     = i_mem_rd_data;

    assign o_mem_rd      = demand_miss_new || pref_issue_now;
    assign o_mem_rd_addr = demand_miss_new ? req_blk_addr : pref_req_addr;

    assign o_mem_wr      = fill_victim_dirty || store_miss_ready;
    assign o_mem_wr_addr = fill_victim_dirty ? fill_evict_addr  : req_blk_addr;
    assign o_mem_wr_data = fill_victim_dirty ? fill_victim_data : store_miss_mem_wr_data;
    assign o_mem_wr_be   = fill_victim_dirty ? {BLOCK_SIZE{1'b1}} : store_miss_mem_wr_be;

    // ------------------------------------------------------------------------
    // 11. Sequential Logic
    // ------------------------------------------------------------------------
    integer s_idx, w_idx, mq_idx;
    wire memq_push = o_mem_rd;
    wire memq_pop  = fill_now;

    always @(posedge i_clk) begin
        if (i_rst) begin
            memq_head        <= 2'd0;
            memq_tail        <= 2'd0;
            memq_count       <= 3'd0;
            pref_req_pending <= 1'b0;
            pref_req_addr    <= 32'd0;

            for (mq_idx = 0; mq_idx < MEMQ_SIZE; mq_idx = mq_idx + 1) begin
                memq_valid[mq_idx] <= 1'b0;
                memq_addr[mq_idx]  <= 32'd0;
            end

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
            // Replacement policy update
            if (access_update) begin
                if (EVICT_POLICY == 0) begin
                    for (w_idx = 0; w_idx < NUM_WAYS; w_idx = w_idx + 1) begin
                        order[update_set][w_idx] <= next_order[update_set][w_idx];
                    end
                end else begin
                    plru_tree[update_set] <= next_tree[update_set];
                end
            end

            // Store hit update
            if (store_hit_ready) begin
                mem[req_set][hit_way]   <= store_hit_updated_line;
                dirty[req_set][hit_way] <= 1'b1;
            end

            // Line fill install from memory response
            if (fill_install_en) begin
                mem[fill_set][fill_victim_way]   <= i_mem_rd_data;
                tags[fill_set][fill_victim_way]  <= fill_tag;
                valid[fill_set][fill_victim_way] <= 1'b1;
                dirty[fill_set][fill_victim_way] <= 1'b0;
            end

            // Memory Read Queue (MSHR) push/pop
            if (memq_pop) begin
                memq_valid[memq_head] <= 1'b0;
                memq_head <= memq_head + 2'd1;
            end
            if (memq_push) begin
                memq_valid[memq_tail] <= 1'b1;
                memq_addr[memq_tail]  <= o_mem_rd_addr;
                memq_tail <= memq_tail + 2'd1;
            end
            memq_count <= memq_count + (memq_push ? 3'd1 : 3'd0) - (memq_pop ? 3'd1 : 3'd0);

            // Pending next-line prefetch scheduling
            if (demand_miss_new && PREFETCH_EN && !next_line_present) begin
                pref_req_pending <= 1'b1;
                pref_req_addr    <= next_line_addr;
            end else if (pref_issue_now || (pref_req_pending && !pref_still_needed)) begin
                pref_req_pending <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
