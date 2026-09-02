module order_book (
    input  wire        clk,
    input  wire        rst,

    input  wire        packet_valid,
    input  wire [7:0]  msg_type,
    input  wire [7:0]  symbol_id,
    input  wire [31:0] timestamp,
    input  wire [31:0] order_id,
    input  wire        side,
    input  wire [31:0] price,
    input  wire [23:0] quantity,

    output reg          op_done,
    output reg          op_success,
    output reg [5:0]     last_slot
);

    localparam NUM_SLOTS  = 64;
    localparam CHUNK_SIZE = 8;                      // slots examined per cycle
    localparam NUM_CHUNKS = NUM_SLOTS / CHUNK_SIZE;  // 8 cycles to sweep the whole table
    // v3: replaced a serial 8-deep per-chunk compare with a balanced TREE
    // reduction (8 leaves -> 4 -> 2 -> 1) -- fixed the LOGIC-depth bottleneck.
    // v3.2: split "read a chunk's slots" and "compare that chunk's data"
    // into two overlapped pipeline stages, so chunk_idx's read-select only
    // had to reach a plain register each cycle.
    // v4: moved the order table off flat register arrays onto per-position
    // memory banks (ram_style=block), matching the single-address-per-cycle
    // idiom real BRAM needs. Real synthesis showed 0 Block RAM Tiles used
    // (Vivado judged an 8-deep bank too shallow to be worth a 36Kb tile and
    // kept it as flip-flops) -- so this alone didn't move timing much, and
    // the *same class* of problem resurfaced on a different signal:
    // reg_symbol_id[5] (a plain compare-input register) was found fanning
    // out to 179 loads across best_price_reg/best_timestamp_reg's CE pins,
    // net-delay dominated (12.4ns total, mostly routing) -- because ALL
    // ~94 bits of the "current best crossing candidate" registers
    // (best_price+best_timestamp+best_quantity+cross_slot) share ONE
    // update-enable signal, and that enable is computed by the SAME
    // tree-reduce-then-compare logic that also has to reach every one of
    // those scattered bits, all within one cycle.
    // v4.1: applies the same fix that already worked twice (v3.2's
    // read/compare split) one stage deeper. S_SCAN became a 3-stage
    // overlapped pipeline -- Stage A reads a chunk's slots, Stage B
    // tree-reduces the PREVIOUS chunk Stage A already read, Stage C merges
    // the chunk BEFORE THAT which Stage B already tree-reduced -- so the
    // wide "should I update the running best" decision is a REGISTERED,
    // already-computed value (p_*) by the time it's applied, giving that
    // enable's fan-out a full clock period to route instead of competing
    // with the logic that computes it. Explicit reg_dv/p_dv valid flags
    // track what each stage actually has ready (rather than inferring it
    // from chunk_idx alone) so the pipeline fills and drains correctly
    // without silently double-processing or skipping a chunk -- the
    // S_FILL state from v3.2 is gone; priming is now just the pipeline's
    // natural first two "bubble" cycles. This moved WNS from -2.715ns to
    // -0.334ns -- but real synthesis then showed the SAME pattern one
    // level deeper still: reg_timestamp[6], a Stage A output register, was
    // fanning out to 91 loads across p_cross_price/p_cross_ts/
    // p_cross_quantity's DATA pins, because Stage B's leaf-build AND its
    // full 3-level tree-reduce (8->4->2->1) were still one combinational
    // block computed and committed in a single cycle.
    // v4.2 (this version): splits Stage B itself the same way -- Stage B1
    // now does leaf-build + tree Stage1 (8->4) only, committing a
    // registered 4-wide partial result (m_*); Stage B2 finishes the
    // reduction (tree Stage2 4->2, Stage3 2->1) reading m_* and committing
    // to p_* exactly as before. S_SCAN is now a 4-stage overlapped
    // pipeline (A / B1 / B2 / C), gated by a new m_dv valid flag chained
    // in alongside reg_dv/p_dv. Costs 1 more cycle overall (~15 total vs
    // v4.1's ~14).

    localparam MSG_ADD     = 8'h01;
    localparam MSG_CANCEL  = 8'h02;
    localparam MSG_EXECUTE = 8'h03;

    localparam S_IDLE  = 2'd0;
    localparam S_CLEAR = 2'd1; // walk all NUM_CHUNKS rows clearing mem_valid after reset
    localparam S_SCAN  = 2'd2;
    localparam S_APPLY = 2'd3;

    reg [1:0] state;
    reg [3:0] chunk_idx;     // next chunk to read, 0..NUM_CHUNKS
    reg [3:0] reg_chunk_idx; // which chunk reg_* (Stage A's output) holds
    reg [2:0] clear_row;     // 0..NUM_CHUNKS-1, walked once at start-of-day by S_CLEAR

    // v4: the order table as CHUNK_SIZE independent per-position memory
    // banks, each NUM_CHUNKS deep. mem_X[k][row] holds the field for
    // original slot number (row*CHUNK_SIZE + k). ram_style=block asks
    // Vivado to use real Block RAM tiles; a real run showed it declined
    // (0 tiles used, banks this shallow aren't worth a 36Kb tile) and kept
    // these as ordinary registers -- functionally identical to before,
    // just organized this way so the RTL is ready if these ever grow deep
    // enough for the tool to actually take the hint.
    (* ram_style = "block" *) reg              mem_valid    [0:CHUNK_SIZE-1][0:NUM_CHUNKS-1];
    (* ram_style = "block" *) reg [31:0]       mem_order_id [0:CHUNK_SIZE-1][0:NUM_CHUNKS-1];
    (* ram_style = "block" *) reg [7:0]        mem_symbol_id[0:CHUNK_SIZE-1][0:NUM_CHUNKS-1];
    (* ram_style = "block" *) reg              mem_side     [0:CHUNK_SIZE-1][0:NUM_CHUNKS-1];
    (* ram_style = "block" *) reg [31:0]       mem_price    [0:CHUNK_SIZE-1][0:NUM_CHUNKS-1];
    (* ram_style = "block" *) reg [23:0]       mem_quantity [0:CHUNK_SIZE-1][0:NUM_CHUNKS-1];
    (* ram_style = "block" *) reg [31:0]       mem_timestamp[0:CHUNK_SIZE-1][0:NUM_CHUNKS-1]; // resting order's arrival timestamp, for price-TIME priority

    // ---- Stage A output: this chunk's raw slots, one cycle after being read ----
    reg              reg_valid     [0:CHUNK_SIZE-1];
    reg [31:0]        reg_order_id [0:CHUNK_SIZE-1];
    reg [7:0]         reg_symbol_id[0:CHUNK_SIZE-1];
    reg               reg_side     [0:CHUNK_SIZE-1];
    reg [31:0]        reg_price    [0:CHUNK_SIZE-1];
    reg [23:0]        reg_quantity [0:CHUNK_SIZE-1];
    reg [31:0]        reg_timestamp[0:CHUNK_SIZE-1];
    reg               reg_dv; // reg_* holds a chunk Stage B hasn't tree-reduced yet

    // ---- Stage B output: one chunk's TREE-REDUCED result, one cycle after
    //      being computed -- this is what makes the merge-enable in Stage C
    //      a plain registered value instead of something computed fresh
    //      the same cycle it has to fan out to ~94 scattered CE pins. ----
    reg        p_free_found;
    reg [5:0]  p_free_idx;
    reg        p_match_found;
    reg [5:0]  p_match_idx;
    reg [23:0] p_match_quantity;
    reg        p_cross_found;
    reg [31:0] p_cross_price;
    reg [31:0] p_cross_ts;
    reg [23:0] p_cross_quantity;
    reg [5:0]  p_cross_idx;
    reg        p_dv; // p_* holds a chunk Stage C hasn't merged yet

    // ---- Stage B1 output (v4.2): one chunk's leaf-build + tree Stage1
    //      (8->4) result, a REGISTERED 4-wide partial reduction. This is
    //      what lets Stage B2 finish the reduction from a plain register
    //      instead of from logic computed the same cycle it has to reach
    //      p_cross_price/p_cross_ts/p_cross_quantity's data pins. ----
    reg        m_free_found [0:3];
    reg [5:0]  m_free_idx   [0:3];
    reg        m_match_found[0:3];
    reg [5:0]  m_match_idx  [0:3];
    reg [23:0] m_match_quantity[0:3];
    reg        m_cross_found[0:3];
    reg [31:0] m_cross_price[0:3];
    reg [31:0] m_cross_ts   [0:3];
    reg [23:0] m_cross_quantity[0:3];
    reg [5:0]  m_cross_idx  [0:3];
    reg        m_dv; // m_* holds a chunk Stage B2 hasn't finished reducing yet

    // Request latched at the start of an op and held for the whole multi-cycle scan
    reg [7:0]  r_msg_type;
    reg [7:0]  r_symbol_id;
    reg [31:0] r_order_id;
    reg        r_side;
    reg [31:0] r_price;
    reg [23:0] r_quantity;
    reg [31:0] r_timestamp;

    // Running scan results, carried across the whole scan. match_quantity/
    // best_quantity cache the winning candidate's CURRENT quantity as found
    // during the scan -- S_APPLY can't combinationally re-read a slot's
    // quantity the way a flat array would allow, since a real memory read
    // is synchronous.
    reg [5:0]  free_slot;
    reg        free_found;
    reg [5:0]  match_slot;
    reg        match_found;
    reg [23:0] match_quantity;
    reg [5:0]  cross_slot;
    reg        cross_found;
    reg [31:0] best_price;
    reg [31:0] best_timestamp;
    reg [23:0] best_quantity;

    // Blocking-assignment scratch copies of the above (non-blocking
    // assignments don't update within the same cycle, so a "running result"
    // built up procedurally within one clock edge needs a blocking-assignment
    // scratch version, committed to the real registers via non-blocking
    // assignment at the end of the cycle).
    reg [5:0]  v_free_slot;
    reg        v_free_found;
    reg [5:0]  v_match_slot;
    reg        v_match_found;
    reg [23:0] v_match_quantity;
    reg [5:0]  v_cross_slot;
    reg        v_cross_found;
    reg [31:0] v_best_price;
    reg [31:0] v_best_timestamp;
    reg [23:0] v_best_quantity;

    // Same blocking-scratch idea, for the pipeline's own bookkeeping: what
    // reg_dv/m_dv/p_dv/chunk_idx will become after this cycle's non-blocking
    // commits, needed THIS cycle to decide whether the pipeline has fully
    // drained (nothing left in Stage A, B1, B2, or C) and it's safe to move
    // on to S_APPLY.
    reg        b_reg_dv;
    reg        b_m_dv;
    reg        b_p_dv;
    reg [3:0]  b_chunk_idx;

    // Per-chunk leaf/tree scratch arrays (CHUNK_SIZE=8 wide; only the first
    // `count` entries are meaningful as the tree reduction progresses).
    reg        c_free_found [0:CHUNK_SIZE-1];
    reg [5:0]  c_free_idx   [0:CHUNK_SIZE-1];
    reg        c_match_found[0:CHUNK_SIZE-1];
    reg [5:0]  c_match_idx  [0:CHUNK_SIZE-1];
    reg [23:0] c_match_quantity[0:CHUNK_SIZE-1];
    reg        c_cross_found[0:CHUNK_SIZE-1];
    reg [31:0] c_cross_price[0:CHUNK_SIZE-1];
    reg [31:0] c_cross_ts   [0:CHUNK_SIZE-1];
    reg [23:0] c_cross_quantity[0:CHUNK_SIZE-1];
    reg [5:0]  c_cross_idx  [0:CHUNK_SIZE-1];

    // v4: unified write-port staging for the table memories. S_APPLY
    // (blocking-)sets these; a single write-demux loop after the state
    // case block commits them to whichever ONE bank actually owns
    // wr_slot -- exactly one write per bank per cycle, matching a real
    // BRAM write port.
    reg        wr_valid_en;
    reg        wr_valid_val;
    reg        wr_quantity_en;
    reg [23:0] wr_quantity_val;
    reg        wr_full_en;   // new-order write: also commits order_id/symbol_id/side/price/timestamp from r_*
    reg [5:0]  wr_slot;

    integer i;    // reset loop
    integer k;    // leaf-building / fill / read-ahead / write-demux loop (0..CHUNK_SIZE-1)
    integer j;    // tree-level loop
    integer idx;  // original slot index of the chunk currently being tree-reduced (reg_chunk_idx-based)

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_CLEAR;
            clear_row  <= 0;
            chunk_idx  <= 0;
            op_done    <= 0;
            op_success <= 0;
            last_slot  <= 0;
        end else begin
            op_done        <= 0; // default: pulse low unless S_APPLY fires this cycle
            wr_valid_en    = 1'b0;
            wr_quantity_en = 1'b0;
            wr_full_en     = 1'b0;
            wr_slot        = 6'd0;

            case (state)

                S_CLEAR: begin
                    // A real memory port can only take one address per cycle,
                    // so clearing all 64 "valid" bits takes NUM_CHUNKS cycles:
                    // each cycle clears row `clear_row` across all CHUNK_SIZE
                    // banks at once (CHUNK_SIZE *separate* memories, each
                    // getting exactly one write this cycle -- legal). Only
                    // `valid` needs clearing; every other field is don't-care
                    // whenever valid=0, same as the old design.
                    for (k = 0; k < CHUNK_SIZE; k = k + 1) begin
                        mem_valid[k][clear_row] <= 1'b0;
                    end
                    if (clear_row == NUM_CHUNKS - 1) begin
                        state <= S_IDLE;
                    end else begin
                        clear_row <= clear_row + 1;
                    end
                end

                S_IDLE: begin
                    if (packet_valid) begin
                        r_msg_type  <= msg_type;
                        r_symbol_id <= symbol_id;
                        r_order_id  <= order_id;
                        r_side      <= side;
                        r_price     <= price;
                        r_quantity  <= quantity;
                        r_timestamp <= timestamp;

                        free_found     <= 0;
                        free_slot      <= 0;
                        match_found    <= 0;
                        match_slot     <= 0;
                        match_quantity <= 0;
                        cross_found    <= 0;
                        cross_slot     <= 0;
                        best_price     <= side ? 32'h00000000 : 32'hFFFFFFFF;
                        best_timestamp <= 0;
                        best_quantity  <= 0;

                        chunk_idx <= 0;
                        reg_dv    <= 0;
                        m_dv      <= 0;
                        p_dv      <= 0;
                        state     <= S_SCAN;
                    end
                end

                S_SCAN: begin
                    // ---- Stage C: merge whatever Stage B tree-reduced LAST
                    //      cycle (p_*) into the running result. p_* is a
                    //      plain registered value here -- its update-enable
                    //      only has to reach the merge registers' CE pins,
                    //      never compete with tree-reduce logic in the same
                    //      cycle the way it did before. ----
                    if (p_dv) begin
                        v_free_slot      = free_slot;
                        v_free_found     = free_found;
                        v_match_slot     = match_slot;
                        v_match_found    = match_found;
                        v_match_quantity = match_quantity;
                        v_cross_slot     = cross_slot;
                        v_cross_found    = cross_found;
                        v_best_price     = best_price;
                        v_best_timestamp = best_timestamp;
                        v_best_quantity  = best_quantity;

                        if (!v_free_found && p_free_found) begin
                            v_free_found = 1'b1;
                            v_free_slot  = p_free_idx;
                        end
                        if (!v_match_found && p_match_found) begin
                            v_match_found    = 1'b1;
                            v_match_slot     = p_match_idx;
                            v_match_quantity = p_match_quantity;
                        end
                        if (!v_cross_found && p_cross_found) begin
                            v_cross_found    = 1'b1;
                            v_best_price     = p_cross_price;
                            v_best_timestamp = p_cross_ts;
                            v_best_quantity  = p_cross_quantity;
                            v_cross_slot     = p_cross_idx;
                        end else if (v_cross_found && p_cross_found) begin
                            if ((r_side == 1'b0) ?
                                ((p_cross_price < v_best_price) ||
                                 (p_cross_price == v_best_price && p_cross_ts < v_best_timestamp)) :
                                ((p_cross_price > v_best_price) ||
                                 (p_cross_price == v_best_price && p_cross_ts < v_best_timestamp))) begin
                                v_best_price     = p_cross_price;
                                v_best_timestamp = p_cross_ts;
                                v_best_quantity  = p_cross_quantity;
                                v_cross_slot     = p_cross_idx;
                            end
                        end

                        free_slot      <= v_free_slot;
                        free_found     <= v_free_found;
                        match_slot     <= v_match_slot;
                        match_found    <= v_match_found;
                        match_quantity <= v_match_quantity;
                        cross_slot     <= v_cross_slot;
                        cross_found    <= v_cross_found;
                        best_price     <= v_best_price;
                        best_timestamp <= v_best_timestamp;
                        best_quantity  <= v_best_quantity;
                    end

                    // ---- Stage B2: finish tree-reducing the PREVIOUS
                    //      chunk's Stage B1 output (m_*, a registered
                    //      4-wide partial reduction) down to a single best
                    //      candidate for Stage C, via tree Stage2 (4->2) +
                    //      Stage3 (2->1). Runs BEFORE Stage B1 below so it
                    //      finishes reading m_* and committing to p_*
                    //      before Stage B1 overwrites the shared c_*
                    //      scratch arrays for its own (unrelated)
                    //      leaf-build + Stage1 computation this same
                    //      cycle. ----
                    if (m_dv) begin
                        // Stage 2: tree-reduce 4 -> 2 (reads m_*, scratch in c_*)
                        for (j = 0; j < 2; j = j + 1) begin
                            if (m_free_found[2*j]) begin
                                c_free_found[j] = m_free_found[2*j];
                                c_free_idx[j]   = m_free_idx[2*j];
                            end else begin
                                c_free_found[j] = m_free_found[2*j+1];
                                c_free_idx[j]   = m_free_idx[2*j+1];
                            end
                            if (m_match_found[2*j]) begin
                                c_match_found[j]    = m_match_found[2*j];
                                c_match_idx[j]      = m_match_idx[2*j];
                                c_match_quantity[j] = m_match_quantity[2*j];
                            end else begin
                                c_match_found[j]    = m_match_found[2*j+1];
                                c_match_idx[j]      = m_match_idx[2*j+1];
                                c_match_quantity[j] = m_match_quantity[2*j+1];
                            end
                            if (!m_cross_found[2*j]) begin
                                c_cross_found[j]    = m_cross_found[2*j+1];
                                c_cross_price[j]    = m_cross_price[2*j+1];
                                c_cross_ts[j]       = m_cross_ts[2*j+1];
                                c_cross_quantity[j] = m_cross_quantity[2*j+1];
                                c_cross_idx[j]      = m_cross_idx[2*j+1];
                            end else if (!m_cross_found[2*j+1]) begin
                                c_cross_found[j]    = m_cross_found[2*j];
                                c_cross_price[j]    = m_cross_price[2*j];
                                c_cross_ts[j]       = m_cross_ts[2*j];
                                c_cross_quantity[j] = m_cross_quantity[2*j];
                                c_cross_idx[j]      = m_cross_idx[2*j];
                            end else begin
                                c_cross_found[j] = 1'b1;
                                if ((r_side == 1'b0) ?
                                    ((m_cross_price[2*j+1] < m_cross_price[2*j]) ||
                                     (m_cross_price[2*j+1] == m_cross_price[2*j] && m_cross_ts[2*j+1] < m_cross_ts[2*j])) :
                                    ((m_cross_price[2*j+1] > m_cross_price[2*j]) ||
                                     (m_cross_price[2*j+1] == m_cross_price[2*j] && m_cross_ts[2*j+1] < m_cross_ts[2*j]))) begin
                                    c_cross_price[j]    = m_cross_price[2*j+1];
                                    c_cross_ts[j]       = m_cross_ts[2*j+1];
                                    c_cross_quantity[j] = m_cross_quantity[2*j+1];
                                    c_cross_idx[j]      = m_cross_idx[2*j+1];
                                end else begin
                                    c_cross_price[j]    = m_cross_price[2*j];
                                    c_cross_ts[j]       = m_cross_ts[2*j];
                                    c_cross_quantity[j] = m_cross_quantity[2*j];
                                    c_cross_idx[j]      = m_cross_idx[2*j];
                                end
                            end
                        end

                        // Stage 3: tree-reduce 2 -> 1 (this chunk's final result, into c_*[0])
                        if (c_free_found[0]) begin
                            c_free_found[0] = c_free_found[0];
                            c_free_idx[0]   = c_free_idx[0];
                        end else begin
                            c_free_found[0] = c_free_found[1];
                            c_free_idx[0]   = c_free_idx[1];
                        end
                        if (c_match_found[0]) begin
                            c_match_found[0]    = c_match_found[0];
                            c_match_idx[0]      = c_match_idx[0];
                            c_match_quantity[0] = c_match_quantity[0];
                        end else begin
                            c_match_found[0]    = c_match_found[1];
                            c_match_idx[0]      = c_match_idx[1];
                            c_match_quantity[0] = c_match_quantity[1];
                        end
                        if (!c_cross_found[0]) begin
                            c_cross_found[0]    = c_cross_found[1];
                            c_cross_price[0]    = c_cross_price[1];
                            c_cross_ts[0]       = c_cross_ts[1];
                            c_cross_quantity[0] = c_cross_quantity[1];
                            c_cross_idx[0]      = c_cross_idx[1];
                        end else if (!c_cross_found[1]) begin
                            c_cross_found[0]    = c_cross_found[0];
                            c_cross_price[0]    = c_cross_price[0];
                            c_cross_ts[0]       = c_cross_ts[0];
                            c_cross_quantity[0] = c_cross_quantity[0];
                            c_cross_idx[0]      = c_cross_idx[0];
                        end else begin
                            c_cross_found[0] = 1'b1;
                            if ((r_side == 1'b0) ?
                                ((c_cross_price[1] < c_cross_price[0]) ||
                                 (c_cross_price[1] == c_cross_price[0] && c_cross_ts[1] < c_cross_ts[0])) :
                                ((c_cross_price[1] > c_cross_price[0]) ||
                                 (c_cross_price[1] == c_cross_price[0] && c_cross_ts[1] < c_cross_ts[0]))) begin
                                c_cross_price[0]    = c_cross_price[1];
                                c_cross_ts[0]       = c_cross_ts[1];
                                c_cross_quantity[0] = c_cross_quantity[1];
                                c_cross_idx[0]      = c_cross_idx[1];
                            end
                            // else: position 0 already holds the better candidate
                        end

                        p_free_found     <= c_free_found[0];
                        p_free_idx       <= c_free_idx[0];
                        p_match_found    <= c_match_found[0];
                        p_match_idx      <= c_match_idx[0];
                        p_match_quantity <= c_match_quantity[0];
                        p_cross_found    <= c_cross_found[0];
                        p_cross_price    <= c_cross_price[0];
                        p_cross_ts       <= c_cross_ts[0];
                        p_cross_quantity <= c_cross_quantity[0];
                        p_cross_idx      <= c_cross_idx[0];
                        b_p_dv = 1'b1;
                    end else begin
                        b_p_dv = 1'b0;
                    end
                    p_dv <= b_p_dv;

                    // ---- Stage B1: leaf-build + tree Stage1 (8->4) on
                    //      whatever Stage A read LAST cycle (reg_*),
                    //      committing the 4-wide partial result into m_*
                    //      for Stage B2 next cycle. Overwrites c_* AFTER
                    //      Stage B2 above already finished using it this
                    //      cycle. ----
                    if (reg_dv) begin
                        // Stage 0: build leaves
                        for (k = 0; k < CHUNK_SIZE; k = k + 1) begin
                            idx = reg_chunk_idx * CHUNK_SIZE + k;

                            c_free_found[k] = !reg_valid[k];
                            c_free_idx[k]   = idx[5:0];

                            c_match_found[k]    = reg_valid[k] && (reg_order_id[k] == r_order_id);
                            c_match_idx[k]      = idx[5:0];
                            c_match_quantity[k] = reg_quantity[k];

                            if (reg_valid[k] &&
                                (reg_symbol_id[k] == r_symbol_id) &&
                                (reg_side[k] != r_side) &&
                                ((r_side == 1'b0) ? (reg_price[k] <= r_price) : (reg_price[k] >= r_price))) begin
                                c_cross_found[k] = 1'b1;
                            end else begin
                                c_cross_found[k] = 1'b0;
                            end
                            c_cross_price[k]    = reg_price[k];
                            c_cross_ts[k]       = reg_timestamp[k];
                            c_cross_quantity[k] = reg_quantity[k];
                            c_cross_idx[k]      = idx[5:0];
                        end

                        // Stage 1: tree-reduce 8 -> 4
                        for (j = 0; j < 4; j = j + 1) begin
                            if (c_free_found[2*j]) begin
                                c_free_found[j] = c_free_found[2*j];
                                c_free_idx[j]   = c_free_idx[2*j];
                            end else begin
                                c_free_found[j] = c_free_found[2*j+1];
                                c_free_idx[j]   = c_free_idx[2*j+1];
                            end
                            if (c_match_found[2*j]) begin
                                c_match_found[j]    = c_match_found[2*j];
                                c_match_idx[j]      = c_match_idx[2*j];
                                c_match_quantity[j] = c_match_quantity[2*j];
                            end else begin
                                c_match_found[j]    = c_match_found[2*j+1];
                                c_match_idx[j]      = c_match_idx[2*j+1];
                                c_match_quantity[j] = c_match_quantity[2*j+1];
                            end
                            if (!c_cross_found[2*j]) begin
                                c_cross_found[j]    = c_cross_found[2*j+1];
                                c_cross_price[j]    = c_cross_price[2*j+1];
                                c_cross_ts[j]       = c_cross_ts[2*j+1];
                                c_cross_quantity[j] = c_cross_quantity[2*j+1];
                                c_cross_idx[j]      = c_cross_idx[2*j+1];
                            end else if (!c_cross_found[2*j+1]) begin
                                c_cross_found[j]    = c_cross_found[2*j];
                                c_cross_price[j]    = c_cross_price[2*j];
                                c_cross_ts[j]       = c_cross_ts[2*j];
                                c_cross_quantity[j] = c_cross_quantity[2*j];
                                c_cross_idx[j]      = c_cross_idx[2*j];
                            end else begin
                                c_cross_found[j] = 1'b1;
                                if ((r_side == 1'b0) ?
                                    ((c_cross_price[2*j+1] < c_cross_price[2*j]) ||
                                     (c_cross_price[2*j+1] == c_cross_price[2*j] && c_cross_ts[2*j+1] < c_cross_ts[2*j])) :
                                    ((c_cross_price[2*j+1] > c_cross_price[2*j]) ||
                                     (c_cross_price[2*j+1] == c_cross_price[2*j] && c_cross_ts[2*j+1] < c_cross_ts[2*j]))) begin
                                    c_cross_price[j]    = c_cross_price[2*j+1];
                                    c_cross_ts[j]       = c_cross_ts[2*j+1];
                                    c_cross_quantity[j] = c_cross_quantity[2*j+1];
                                    c_cross_idx[j]      = c_cross_idx[2*j+1];
                                end else begin
                                    c_cross_price[j]    = c_cross_price[2*j];
                                    c_cross_ts[j]       = c_cross_ts[2*j];
                                    c_cross_quantity[j] = c_cross_quantity[2*j];
                                    c_cross_idx[j]      = c_cross_idx[2*j];
                                end
                            end
                        end

                        for (k = 0; k < 4; k = k + 1) begin
                            m_free_found[k]     <= c_free_found[k];
                            m_free_idx[k]       <= c_free_idx[k];
                            m_match_found[k]    <= c_match_found[k];
                            m_match_idx[k]      <= c_match_idx[k];
                            m_match_quantity[k] <= c_match_quantity[k];
                            m_cross_found[k]    <= c_cross_found[k];
                            m_cross_price[k]    <= c_cross_price[k];
                            m_cross_ts[k]       <= c_cross_ts[k];
                            m_cross_quantity[k] <= c_cross_quantity[k];
                            m_cross_idx[k]      <= c_cross_idx[k];
                        end
                        b_m_dv = 1'b1;
                    end else begin
                        b_m_dv = 1'b0;
                    end
                    m_dv <= b_m_dv;

                    // ---- Stage A: read the next chunk's slots ----
                    if (chunk_idx < NUM_CHUNKS) begin
                        for (k = 0; k < CHUNK_SIZE; k = k + 1) begin
                            reg_valid[k]     <= mem_valid[k][chunk_idx];
                            reg_order_id[k]  <= mem_order_id[k][chunk_idx];
                            reg_symbol_id[k] <= mem_symbol_id[k][chunk_idx];
                            reg_side[k]      <= mem_side[k][chunk_idx];
                            reg_price[k]     <= mem_price[k][chunk_idx];
                            reg_quantity[k]  <= mem_quantity[k][chunk_idx];
                            reg_timestamp[k] <= mem_timestamp[k][chunk_idx];
                        end
                        reg_chunk_idx <= chunk_idx;
                        b_reg_dv    = 1'b1;
                        b_chunk_idx = chunk_idx + 1;
                    end else begin
                        b_reg_dv    = 1'b0;
                        b_chunk_idx = chunk_idx;
                    end
                    reg_dv    <= b_reg_dv;
                    chunk_idx <= b_chunk_idx;

                    // ---- Done once nothing will be pending in any stage
                    //      next cycle (b_* are this cycle's freshly computed
                    //      "next" values, so this correctly waits for the
                    //      LAST chunk to actually finish Stage C, not just
                    //      Stage A). ----
                    if (b_chunk_idx >= NUM_CHUNKS && !b_reg_dv && !b_m_dv && !b_p_dv) begin
                        state <= S_APPLY;
                    end
                end

                S_APPLY: begin
                    // Stage the write (wr_*) instead of indexing straight into
                    // a flat array -- the actual memory write happens in the
                    // demux loop after this case block, one bank at a time.
                    case (r_msg_type)

                        MSG_ADD: begin
                            if (cross_found) begin
                                wr_slot = cross_slot;
                                if (r_quantity >= best_quantity) begin
                                    wr_valid_en  = 1'b1;
                                    wr_valid_val = 1'b0;
                                end else begin
                                    wr_quantity_en  = 1'b1;
                                    wr_quantity_val = best_quantity - r_quantity;
                                end
                                last_slot  <= cross_slot;
                                op_success <= 1;
                            end else if (free_found) begin
                                wr_slot         = free_slot;
                                wr_full_en      = 1'b1; // commits order_id/symbol_id/side/price/timestamp from r_*
                                wr_valid_en     = 1'b1;
                                wr_valid_val    = 1'b1;
                                wr_quantity_en  = 1'b1;
                                wr_quantity_val = r_quantity;
                                last_slot  <= free_slot;
                                op_success <= 1;
                            end else begin
                                op_success <= 0;
                            end
                        end

                        MSG_CANCEL: begin
                            if (match_found) begin
                                wr_slot      = match_slot;
                                wr_valid_en  = 1'b1;
                                wr_valid_val = 1'b0;
                                last_slot  <= match_slot;
                                op_success <= 1;
                            end else begin
                                op_success <= 0; // order not found
                            end
                        end

                        MSG_EXECUTE: begin
                            if (match_found) begin
                                wr_slot = match_slot;
                                if (r_quantity >= match_quantity) begin
                                    wr_valid_en  = 1'b1;
                                    wr_valid_val = 1'b0; // fully filled
                                end else begin
                                    wr_quantity_en  = 1'b1;
                                    wr_quantity_val = match_quantity - r_quantity;
                                end
                                last_slot  <= match_slot;
                                op_success <= 1;
                            end else begin
                                op_success <= 0;
                            end
                        end

                        default: begin
                            op_success <= 0;
                        end

                    endcase

                    op_done <= 1;
                    state   <= S_IDLE;
                end

            endcase

            // ---- v4 unified write-port demux: fires only when S_APPLY set
            //      one of the wr_*_en flags above this same cycle. wr_slot's
            //      low 3 bits pick which ONE of the CHUNK_SIZE banks owns
            //      this write; its high 3 bits are that bank's row address.
            //      Each bank gets at most one write this cycle -- matching a
            //      real single-write-port memory exactly. ----
            for (k = 0; k < CHUNK_SIZE; k = k + 1) begin
                if (wr_slot[2:0] == k) begin
                    if (wr_valid_en)    mem_valid[k][wr_slot[5:3]]    <= wr_valid_val;
                    if (wr_quantity_en) mem_quantity[k][wr_slot[5:3]] <= wr_quantity_val;
                    if (wr_full_en) begin
                        mem_order_id[k][wr_slot[5:3]]  <= r_order_id;
                        mem_symbol_id[k][wr_slot[5:3]] <= r_symbol_id;
                        mem_side[k][wr_slot[5:3]]      <= r_side;
                        mem_price[k][wr_slot[5:3]]     <= r_price;
                        mem_timestamp[k][wr_slot[5:3]] <= r_timestamp;
                    end
                end
            end

            // Defensive check only: should never fire given the 18-cycle margin
            // (packet_parser needs 18 bytes/packet; this scan takes ~14 cycles,
            // plus an 8-cycle S_CLEAR that only ever runs once at start-of-day),
            // but a future change to either side could silently violate it otherwise.
            // synthesis translate_off
            if (packet_valid && state != S_IDLE) begin
                $display("WARNING @ t=%0t: order_book got packet_valid while busy (state=%0d) -- operation dropped!", $time, state);
            end
            // synthesis translate_on
        end
    end

endmodule
