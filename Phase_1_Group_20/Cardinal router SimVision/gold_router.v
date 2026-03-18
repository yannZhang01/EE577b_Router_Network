module gold_router #(
    parameter PACKET_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     reset,
    input  wire                     polarity,
    input  wire [3:0]               local_x,
    input  wire [3:0]               local_y,

    // North side external interface
    input  wire                     n_si,
    output wire                     n_ri,
    input  wire [PACKET_WIDTH-1:0]  n_di,
    output wire                     n_so,
    input  wire                     n_ro,
    output wire [PACKET_WIDTH-1:0]  n_do,

    // South side external interface
    input  wire                     s_si,
    output wire                     s_ri,
    input  wire [PACKET_WIDTH-1:0]  s_di,
    output wire                     s_so,
    input  wire                     s_ro,
    output wire [PACKET_WIDTH-1:0]  s_do,

    // East side external interface
    input  wire                     e_si,
    output wire                     e_ri,
    input  wire [PACKET_WIDTH-1:0]  e_di,
    output wire                     e_so,
    input  wire                     e_ro,
    output wire [PACKET_WIDTH-1:0]  e_do,

    // West side external interface
    input  wire                     w_si,
    output wire                     w_ri,
    input  wire [PACKET_WIDTH-1:0]  w_di,
    output wire                     w_so,
    input  wire                     w_ro,
    output wire [PACKET_WIDTH-1:0]  w_do,

    // Local PE/NIC external interface
    input  wire                     pe_si,
    output wire                     pe_ri,
    input  wire [PACKET_WIDTH-1:0]  pe_di,
    output wire                     pe_so,
    input  wire                     pe_ro,
    output wire [PACKET_WIDTH-1:0]  pe_do
);

    // Port encoding used by request, grant, and buffer index logic.
    localparam PORT_N  = 3'd0;
    localparam PORT_S  = 3'd1;
    localparam PORT_E  = 3'd2;
    localparam PORT_W  = 3'd3;
    localparam PORT_PE = 3'd4;

    // Direction encoding: 0 for positive direction (E or S), 1 for negative direction (W or N).
    localparam DIRX_E = 1'b0;
    localparam DIRX_W = 1'b1;
    localparam DIRY_S = 1'b0;
    localparam DIRY_N = 1'b1;

    // Single-slot input and output buffers for every (port, VC) pair.
    // Index mapping is handled by buf_idx(port, vc).
    reg  [PACKET_WIDTH-1:0] in_buf_data  [0:9];
    reg                     in_buf_valid [0:9];
    reg  [PACKET_WIDTH-1:0] out_buf_data  [0:9];
    reg                     out_buf_valid [0:9];

    // Per-output, per-VC round-robin pointers.
    // E and W each arbitrate between 2 requesters, so 1-bit pointers are enough.
    // N, S, and PE each arbitrate between 4 requesters, so 2-bit pointers are used.
    reg e_rr_ptr_vc0;
    reg e_rr_ptr_vc1;
    reg w_rr_ptr_vc0;
    reg w_rr_ptr_vc1;
    reg [1:0] n_rr_ptr_vc0;
    reg [1:0] n_rr_ptr_vc1;
    reg [1:0] s_rr_ptr_vc0;
    reg [1:0] s_rr_ptr_vc1;
    reg [1:0] pe_rr_ptr_vc0;
    reg [1:0] pe_rr_ptr_vc1;

    // The router processes one VC internally while the other VC is used on external links.
    wire internal_vc;
    wire external_vc;

    integer i;

    // Map a (port, vc) pair into the flat buffer arrays.
    // Layout example: N.vc0, N.vc1, S.vc0, S.vc1, ...
    function integer buf_idx;
        input integer port;
        input integer vc;
        begin
            buf_idx = (port << 1) + vc;
        end
    endfunction

    // Header field extractors.
    function get_vc;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            get_vc = pkt[63];
        end
    endfunction

    function get_dirx;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            get_dirx = pkt[62];
        end
    endfunction

    function get_diry;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            get_diry = pkt[61];
        end
    endfunction

    function [3:0] get_hops_x;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            get_hops_x = pkt[60:57];
        end
    endfunction

    function [3:0] get_hops_y;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            get_hops_y = pkt[56:53];
        end
    endfunction

    // Header field update helpers.
    function [PACKET_WIDTH-1:0] set_vc;
        input [PACKET_WIDTH-1:0] pkt;
        input                    vc;
        begin
            set_vc       = pkt;
            set_vc[63]   = vc;
        end
    endfunction

    function [PACKET_WIDTH-1:0] set_hops_x;
        input [PACKET_WIDTH-1:0] pkt;
        input [3:0]              hops_x;
        begin
            set_hops_x        = pkt;
            set_hops_x[60:57] = hops_x;
        end
    endfunction

    function [PACKET_WIDTH-1:0] set_hops_y;
        input [PACKET_WIDTH-1:0] pkt;
        input [3:0]              hops_y;
        begin
            set_hops_y        = pkt;
            set_hops_y[56:53] = hops_y;
        end
    endfunction

    // Dateline crossing checks used for VC updates.
    function is_cross_x;
        input dirx;
        input [3:0] x;
        begin
            is_cross_x = ((dirx == DIRX_E) && (x == 4'd3)) ||
                         ((dirx == DIRX_W) && (x == 4'd0));
        end
    endfunction

    function is_cross_y;
        input diry;
        input [3:0] y;
        begin
            is_cross_y = ((diry == DIRY_S) && (y == 4'd3)) ||
                         ((diry == DIRY_N) && (y == 4'd0));
        end
    endfunction

    // Build the packet image that will be written into an output buffer after one X hop.
    function [PACKET_WIDTH-1:0] build_x_packet;
        input [PACKET_WIDTH-1:0] pkt;
        input                    next_vc;
        reg   [PACKET_WIDTH-1:0] temp_pkt;
        reg   [3:0]              next_hops_x;
        begin
            next_hops_x    = get_hops_x(pkt) - 4'd1;
            temp_pkt       = set_hops_x(pkt, next_hops_x);
            build_x_packet = set_vc(temp_pkt, next_vc);
        end
    endfunction

    // Build the packet image that will be written into an output buffer after one Y hop.
    function [PACKET_WIDTH-1:0] build_y_packet;
        input [PACKET_WIDTH-1:0] pkt;
        input                    next_vc;
        reg   [PACKET_WIDTH-1:0] temp_pkt;
        reg   [3:0]              next_hops_y;
        begin
            next_hops_y    = get_hops_y(pkt) - 4'd1;
            temp_pkt       = set_hops_y(pkt, next_hops_y);
            build_y_packet = set_vc(temp_pkt, next_vc);
        end
    endfunction

    // Two-request round-robin pick function.
    // Return value 0/1 selects a requester, and 7 means no valid requester.
    function [2:0] rr2_pick;
        input ptr;
        input req0;
        input req1;
        begin
            rr2_pick = 3'd7;
            if (ptr == 1'b0) begin
                if (req0)
                    rr2_pick = 3'd0;
                else if (req1)
                    rr2_pick = 3'd1;
            end else begin
                if (req1)
                    rr2_pick = 3'd1;
                else if (req0)
                    rr2_pick = 3'd0;
            end
        end
    endfunction

    // Four-request round-robin pick function.
    // Return value 0/1/2/3 selects a requester, and 7 means no valid requester.
    function [2:0] rr4_pick;
        input [1:0] ptr;
        input req0;
        input req1;
        input req2;
        input req3;
        begin
            rr4_pick = 3'd7;
            case (ptr)
                2'd0: begin
                    if (req0)
                        rr4_pick = 3'd0;
                    else if (req1)
                        rr4_pick = 3'd1;
                    else if (req2)
                        rr4_pick = 3'd2;
                    else if (req3)
                        rr4_pick = 3'd3;
                end
                2'd1: begin
                    if (req1)
                        rr4_pick = 3'd1;
                    else if (req2)
                        rr4_pick = 3'd2;
                    else if (req3)
                        rr4_pick = 3'd3;
                    else if (req0)
                        rr4_pick = 3'd0;
                end
                2'd2: begin
                    if (req2)
                        rr4_pick = 3'd2;
                    else if (req3)
                        rr4_pick = 3'd3;
                    else if (req0)
                        rr4_pick = 3'd0;
                    else if (req1)
                        rr4_pick = 3'd1;
                end
                default: begin
                    if (req3)
                        rr4_pick = 3'd3;
                    else if (req0)
                        rr4_pick = 3'd0;
                    else if (req1)
                        rr4_pick = 3'd1;
                    else if (req2)
                        rr4_pick = 3'd2;
                end
            endcase
        end
    endfunction

    // VC scheduling: one VC is processed inside the router while the other VC is used on links.
    assign internal_vc = polarity;
    assign external_vc = ~polarity;

    // Input ready is asserted when the external VC input slot for that port is empty.
    assign n_ri  = ~in_buf_valid[buf_idx(PORT_N,  external_vc)];
    assign s_ri  = ~in_buf_valid[buf_idx(PORT_S,  external_vc)];
    assign e_ri  = ~in_buf_valid[buf_idx(PORT_E,  external_vc)];
    assign w_ri  = ~in_buf_valid[buf_idx(PORT_W,  external_vc)];
    assign pe_ri = ~in_buf_valid[buf_idx(PORT_PE, external_vc)];

    // A transmit fire means the current external VC can leave this router on that port.
    wire n_tx_fire;
    wire s_tx_fire;
    wire e_tx_fire;
    wire w_tx_fire;
    wire pe_tx_fire;

    assign n_tx_fire  = out_buf_valid[buf_idx(PORT_N,  external_vc)] & n_ro;
    assign s_tx_fire  = out_buf_valid[buf_idx(PORT_S,  external_vc)] & s_ro;
    assign e_tx_fire  = out_buf_valid[buf_idx(PORT_E,  external_vc)] & e_ro;
    assign w_tx_fire  = out_buf_valid[buf_idx(PORT_W,  external_vc)] & w_ro;
    assign pe_tx_fire = out_buf_valid[buf_idx(PORT_PE, external_vc)] & pe_ro;

    // Send and data outputs are driven directly from the selected external VC output slot.
    assign n_so  = n_tx_fire;
    assign s_so  = s_tx_fire;
    assign e_so  = e_tx_fire;
    assign w_so  = w_tx_fire;
    assign pe_so = pe_tx_fire;

    assign n_do  = out_buf_data[buf_idx(PORT_N,  external_vc)];
    assign s_do  = out_buf_data[buf_idx(PORT_S,  external_vc)];
    assign e_do  = out_buf_data[buf_idx(PORT_E,  external_vc)];
    assign w_do  = out_buf_data[buf_idx(PORT_W,  external_vc)];
    assign pe_do = out_buf_data[buf_idx(PORT_PE, external_vc)];

    // Output slot availability for each (port, vc) pair.
    // A slot can accept a new packet if it is currently empty or if it is being transmitted this cycle.
    wire n_buf_can_vc0;
    wire n_buf_can_vc1;
    wire s_buf_can_vc0;
    wire s_buf_can_vc1;
    wire e_buf_can_vc0;
    wire e_buf_can_vc1;
    wire w_buf_can_vc0;
    wire w_buf_can_vc1;
    wire pe_buf_can_vc0;
    wire pe_buf_can_vc1;

    assign n_buf_can_vc0  = ~out_buf_valid[buf_idx(PORT_N,  0)] || ((external_vc == 1'b0) && n_tx_fire);
    assign n_buf_can_vc1  = ~out_buf_valid[buf_idx(PORT_N,  1)] || ((external_vc == 1'b1) && n_tx_fire);
    assign s_buf_can_vc0  = ~out_buf_valid[buf_idx(PORT_S,  0)] || ((external_vc == 1'b0) && s_tx_fire);
    assign s_buf_can_vc1  = ~out_buf_valid[buf_idx(PORT_S,  1)] || ((external_vc == 1'b1) && s_tx_fire);
    assign e_buf_can_vc0  = ~out_buf_valid[buf_idx(PORT_E,  0)] || ((external_vc == 1'b0) && e_tx_fire);
    assign e_buf_can_vc1  = ~out_buf_valid[buf_idx(PORT_E,  1)] || ((external_vc == 1'b1) && e_tx_fire);
    assign w_buf_can_vc0  = ~out_buf_valid[buf_idx(PORT_W,  0)] || ((external_vc == 1'b0) && w_tx_fire);
    assign w_buf_can_vc1  = ~out_buf_valid[buf_idx(PORT_W,  1)] || ((external_vc == 1'b1) && w_tx_fire);
    assign pe_buf_can_vc0 = ~out_buf_valid[buf_idx(PORT_PE, 0)] || ((external_vc == 1'b0) && pe_tx_fire);
    assign pe_buf_can_vc1 = ~out_buf_valid[buf_idx(PORT_PE, 1)] || ((external_vc == 1'b1) && pe_tx_fire);

    // Per-input request bundle.
    // Each active input can request one output port and one destination VC per cycle.
    reg                    req_valid_n;
    reg  [2:0]             req_out_n;
    reg                    req_out_vc_n;
    reg  [PACKET_WIDTH-1:0] req_pkt_n;

    reg                    req_valid_s;
    reg  [2:0]             req_out_s;
    reg                    req_out_vc_s;
    reg  [PACKET_WIDTH-1:0] req_pkt_s;

    reg                    req_valid_e;
    reg  [2:0]             req_out_e;
    reg                    req_out_vc_e;
    reg  [PACKET_WIDTH-1:0] req_pkt_e;

    reg                    req_valid_w;
    reg  [2:0]             req_out_w;
    reg                    req_out_vc_w;
    reg  [PACKET_WIDTH-1:0] req_pkt_w;

    reg                    req_valid_pe;
    reg  [2:0]             req_out_pe;
    reg                    req_out_vc_pe;
    reg  [PACKET_WIDTH-1:0] req_pkt_pe;

    // Current round-robin pointer snapshot for the VC being processed this cycle.
    reg cur_vc_e_ptr;
    reg cur_vc_w_ptr;
    reg [1:0] cur_vc_n_ptr;
    reg [1:0] cur_vc_s_ptr;
    reg [1:0] cur_vc_pe_ptr;

    // Raw arbiter pick indices.
    reg [2:0] e_pick_idx;
    reg [2:0] n_pick_idx;
    reg [2:0] s_pick_idx;
    reg [2:0] pe_pick_idx;

    // Final grant bundles for each output port.
    // These describe which input wins, which VC is targeted, which packet image is written,
    // and whether the output's round-robin pointer should advance.
    reg                    e_grant_valid;
    reg  [2:0]             e_grant_src;
    reg                    e_grant_out_vc;
    reg  [PACKET_WIDTH-1:0] e_grant_pkt;
    reg                    e_advance_ptr;
    reg                    e_next_ptr;

    reg                    w_grant_valid;
    reg  [2:0]             w_grant_src;
    reg                    w_grant_out_vc;
    reg  [PACKET_WIDTH-1:0] w_grant_pkt;
    reg                    w_advance_ptr;
    reg                    w_next_ptr;

    reg                    n_grant_valid;
    reg  [2:0]             n_grant_src;
    reg                    n_grant_out_vc;
    reg  [PACKET_WIDTH-1:0] n_grant_pkt;
    reg                    n_advance_ptr;
    reg  [1:0]             n_next_ptr;

    reg                    s_grant_valid;
    reg  [2:0]             s_grant_src;
    reg                    s_grant_out_vc;
    reg  [PACKET_WIDTH-1:0] s_grant_pkt;
    reg                    s_advance_ptr;
    reg  [1:0]             s_next_ptr;

    reg                    pe_grant_valid;
    reg  [2:0]             pe_grant_src;
    reg                    pe_grant_out_vc;
    reg  [PACKET_WIDTH-1:0] pe_grant_pkt;
    reg                    pe_advance_ptr;
    reg  [1:0]             pe_next_ptr;

    // Per-output legal request wires after routing and output-slot availability checks.
    wire req_e_from_e;
    wire req_e_from_pe;
    wire req_w_from_w;
    wire req_w_from_pe;
    wire req_n_from_n;
    wire req_n_from_e;
    wire req_n_from_w;
    wire req_n_from_pe;
    wire req_s_from_s;
    wire req_s_from_e;
    wire req_s_from_w;
    wire req_s_from_pe;
    wire req_pe_from_n;
    wire req_pe_from_s;
    wire req_pe_from_e;
    wire req_pe_from_w;

    // Multi-request flags used to decide when to advance a round-robin pointer.
    wire e_multi_req;
    wire w_multi_req;
    wire n_multi_req;
    wire s_multi_req;
    wire pe_multi_req;

    // Select the round-robin pointer that matches the VC processed in this cycle.
    always @(*) begin
        if (internal_vc == 1'b0) begin
            cur_vc_e_ptr  = e_rr_ptr_vc0;
            cur_vc_w_ptr  = w_rr_ptr_vc0;
            cur_vc_n_ptr  = n_rr_ptr_vc0;
            cur_vc_s_ptr  = s_rr_ptr_vc0;
            cur_vc_pe_ptr = pe_rr_ptr_vc0;
        end else begin
            cur_vc_e_ptr  = e_rr_ptr_vc1;
            cur_vc_w_ptr  = w_rr_ptr_vc1;
            cur_vc_n_ptr  = n_rr_ptr_vc1;
            cur_vc_s_ptr  = s_rr_ptr_vc1;
            cur_vc_pe_ptr = pe_rr_ptr_vc1;
        end
    end

    // Per-input route computation.
    // Each active input examines its current packet, computes the next output port,
    // computes the next VC, and builds the packet image that will be written into the output buffer.
    always @(*) begin
        req_valid_n  = 1'b0;
        req_out_n    = PORT_N;
        req_out_vc_n = 1'b0;
        req_pkt_n    = {PACKET_WIDTH{1'b0}};

        req_valid_s  = 1'b0;
        req_out_s    = PORT_S;
        req_out_vc_s = 1'b0;
        req_pkt_s    = {PACKET_WIDTH{1'b0}};

        req_valid_e  = 1'b0;
        req_out_e    = PORT_E;
        req_out_vc_e = 1'b0;
        req_pkt_e    = {PACKET_WIDTH{1'b0}};

        req_valid_w  = 1'b0;
        req_out_w    = PORT_W;
        req_out_vc_w = 1'b0;
        req_pkt_w    = {PACKET_WIDTH{1'b0}};

        req_valid_pe  = 1'b0;
        req_out_pe    = PORT_PE;
        req_out_vc_pe = 1'b0;
        req_pkt_pe    = {PACKET_WIDTH{1'b0}};

        // N input: continue on N during Y phase or eject to PE when both hop counts are zero.
        if (in_buf_valid[buf_idx(PORT_N, internal_vc)]) begin
            if ((get_hops_x(in_buf_data[buf_idx(PORT_N, internal_vc)]) == 4'd0) &&
                (get_hops_y(in_buf_data[buf_idx(PORT_N, internal_vc)]) != 4'd0) &&
                (get_diry(in_buf_data[buf_idx(PORT_N, internal_vc)]) == DIRY_N)) begin
                req_valid_n  = 1'b1;
                req_out_n    = PORT_N;
                req_out_vc_n = get_vc(in_buf_data[buf_idx(PORT_N, internal_vc)]) |
                               is_cross_y(get_diry(in_buf_data[buf_idx(PORT_N, internal_vc)]), local_y);
                req_pkt_n    = build_y_packet(in_buf_data[buf_idx(PORT_N, internal_vc)], req_out_vc_n);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_N, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_N, internal_vc)]) == 4'd0)) begin
                req_valid_n  = 1'b1;
                req_out_n    = PORT_PE;
                req_out_vc_n = internal_vc;
                req_pkt_n    = in_buf_data[buf_idx(PORT_N, internal_vc)];
            end
        end

        // S input: continue on S during Y phase or eject to PE when both hop counts are zero.
        if (in_buf_valid[buf_idx(PORT_S, internal_vc)]) begin
            if ((get_hops_x(in_buf_data[buf_idx(PORT_S, internal_vc)]) == 4'd0) &&
                (get_hops_y(in_buf_data[buf_idx(PORT_S, internal_vc)]) != 4'd0) &&
                (get_diry(in_buf_data[buf_idx(PORT_S, internal_vc)]) == DIRY_S)) begin
                req_valid_s  = 1'b1;
                req_out_s    = PORT_S;
                req_out_vc_s = get_vc(in_buf_data[buf_idx(PORT_S, internal_vc)]) |
                               is_cross_y(get_diry(in_buf_data[buf_idx(PORT_S, internal_vc)]), local_y);
                req_pkt_s    = build_y_packet(in_buf_data[buf_idx(PORT_S, internal_vc)], req_out_vc_s);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_S, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_S, internal_vc)]) == 4'd0)) begin
                req_valid_s  = 1'b1;
                req_out_s    = PORT_PE;
                req_out_vc_s = internal_vc;
                req_pkt_s    = in_buf_data[buf_idx(PORT_S, internal_vc)];
            end
        end

        // E input: continue on E during X phase, turn to N/S after X is finished, or eject.
        if (in_buf_valid[buf_idx(PORT_E, internal_vc)]) begin
            if ((get_hops_x(in_buf_data[buf_idx(PORT_E, internal_vc)]) != 4'd0) &&
                (get_dirx(in_buf_data[buf_idx(PORT_E, internal_vc)]) == DIRX_E)) begin
                req_valid_e  = 1'b1;
                req_out_e    = PORT_E;
                req_out_vc_e = get_vc(in_buf_data[buf_idx(PORT_E, internal_vc)]) |
                               is_cross_x(get_dirx(in_buf_data[buf_idx(PORT_E, internal_vc)]), local_x);
                req_pkt_e    = build_x_packet(in_buf_data[buf_idx(PORT_E, internal_vc)], req_out_vc_e);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_E, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_E, internal_vc)]) != 4'd0) &&
                         (get_diry(in_buf_data[buf_idx(PORT_E, internal_vc)]) == DIRY_N)) begin
                req_valid_e  = 1'b1;
                req_out_e    = PORT_N;
                req_out_vc_e = is_cross_y(get_diry(in_buf_data[buf_idx(PORT_E, internal_vc)]), local_y);
                req_pkt_e    = build_y_packet(in_buf_data[buf_idx(PORT_E, internal_vc)], req_out_vc_e);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_E, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_E, internal_vc)]) != 4'd0) &&
                         (get_diry(in_buf_data[buf_idx(PORT_E, internal_vc)]) == DIRY_S)) begin
                req_valid_e  = 1'b1;
                req_out_e    = PORT_S;
                req_out_vc_e = is_cross_y(get_diry(in_buf_data[buf_idx(PORT_E, internal_vc)]), local_y);
                req_pkt_e    = build_y_packet(in_buf_data[buf_idx(PORT_E, internal_vc)], req_out_vc_e);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_E, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_E, internal_vc)]) == 4'd0)) begin
                req_valid_e  = 1'b1;
                req_out_e    = PORT_PE;
                req_out_vc_e = internal_vc;
                req_pkt_e    = in_buf_data[buf_idx(PORT_E, internal_vc)];
            end
        end

        // W input: continue on W during X phase, turn to N/S after X is finished, or eject.
        if (in_buf_valid[buf_idx(PORT_W, internal_vc)]) begin
            if ((get_hops_x(in_buf_data[buf_idx(PORT_W, internal_vc)]) != 4'd0) &&
                (get_dirx(in_buf_data[buf_idx(PORT_W, internal_vc)]) == DIRX_W)) begin
                req_valid_w  = 1'b1;
                req_out_w    = PORT_W;
                req_out_vc_w = get_vc(in_buf_data[buf_idx(PORT_W, internal_vc)]) |
                               is_cross_x(get_dirx(in_buf_data[buf_idx(PORT_W, internal_vc)]), local_x);
                req_pkt_w    = build_x_packet(in_buf_data[buf_idx(PORT_W, internal_vc)], req_out_vc_w);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_W, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_W, internal_vc)]) != 4'd0) &&
                         (get_diry(in_buf_data[buf_idx(PORT_W, internal_vc)]) == DIRY_N)) begin
                req_valid_w  = 1'b1;
                req_out_w    = PORT_N;
                req_out_vc_w = is_cross_y(get_diry(in_buf_data[buf_idx(PORT_W, internal_vc)]), local_y);
                req_pkt_w    = build_y_packet(in_buf_data[buf_idx(PORT_W, internal_vc)], req_out_vc_w);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_W, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_W, internal_vc)]) != 4'd0) &&
                         (get_diry(in_buf_data[buf_idx(PORT_W, internal_vc)]) == DIRY_S)) begin
                req_valid_w  = 1'b1;
                req_out_w    = PORT_S;
                req_out_vc_w = is_cross_y(get_diry(in_buf_data[buf_idx(PORT_W, internal_vc)]), local_y);
                req_pkt_w    = build_y_packet(in_buf_data[buf_idx(PORT_W, internal_vc)], req_out_vc_w);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_W, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_W, internal_vc)]) == 4'd0)) begin
                req_valid_w  = 1'b1;
                req_out_w    = PORT_PE;
                req_out_vc_w = internal_vc;
                req_pkt_w    = in_buf_data[buf_idx(PORT_W, internal_vc)];
            end
        end

        // PE input: inject into E/W during X phase, or directly into N/S when X hops are zero.
        if (in_buf_valid[buf_idx(PORT_PE, internal_vc)]) begin
            if ((get_hops_x(in_buf_data[buf_idx(PORT_PE, internal_vc)]) != 4'd0) &&
                (get_dirx(in_buf_data[buf_idx(PORT_PE, internal_vc)]) == DIRX_E)) begin
                req_valid_pe  = 1'b1;
                req_out_pe    = PORT_E;
                req_out_vc_pe = get_vc(in_buf_data[buf_idx(PORT_PE, internal_vc)]) |
                                is_cross_x(get_dirx(in_buf_data[buf_idx(PORT_PE, internal_vc)]), local_x);
                req_pkt_pe    = build_x_packet(in_buf_data[buf_idx(PORT_PE, internal_vc)], req_out_vc_pe);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_PE, internal_vc)]) != 4'd0) &&
                         (get_dirx(in_buf_data[buf_idx(PORT_PE, internal_vc)]) == DIRX_W)) begin
                req_valid_pe  = 1'b1;
                req_out_pe    = PORT_W;
                req_out_vc_pe = get_vc(in_buf_data[buf_idx(PORT_PE, internal_vc)]) |
                                is_cross_x(get_dirx(in_buf_data[buf_idx(PORT_PE, internal_vc)]), local_x);
                req_pkt_pe    = build_x_packet(in_buf_data[buf_idx(PORT_PE, internal_vc)], req_out_vc_pe);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_PE, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_PE, internal_vc)]) != 4'd0) &&
                         (get_diry(in_buf_data[buf_idx(PORT_PE, internal_vc)]) == DIRY_N)) begin
                req_valid_pe  = 1'b1;
                req_out_pe    = PORT_N;
                req_out_vc_pe = is_cross_y(get_diry(in_buf_data[buf_idx(PORT_PE, internal_vc)]), local_y);
                req_pkt_pe    = build_y_packet(in_buf_data[buf_idx(PORT_PE, internal_vc)], req_out_vc_pe);
            end else if ((get_hops_x(in_buf_data[buf_idx(PORT_PE, internal_vc)]) == 4'd0) &&
                         (get_hops_y(in_buf_data[buf_idx(PORT_PE, internal_vc)]) != 4'd0) &&
                         (get_diry(in_buf_data[buf_idx(PORT_PE, internal_vc)]) == DIRY_S)) begin
                req_valid_pe  = 1'b1;
                req_out_pe    = PORT_S;
                req_out_vc_pe = is_cross_y(get_diry(in_buf_data[buf_idx(PORT_PE, internal_vc)]), local_y);
                req_pkt_pe    = build_y_packet(in_buf_data[buf_idx(PORT_PE, internal_vc)], req_out_vc_pe);
            end
        end
    end

    // Output-facing legal request generation.
    // A request is legal only if the selected output port matches and the target output VC slot can accept a packet.
    assign req_e_from_e  = req_valid_e  && (req_out_e  == PORT_E)  && (req_out_vc_e  ? e_buf_can_vc1  : e_buf_can_vc0);
    assign req_e_from_pe = req_valid_pe && (req_out_pe == PORT_E)  && (req_out_vc_pe ? e_buf_can_vc1  : e_buf_can_vc0);

    assign req_w_from_w  = req_valid_w  && (req_out_w  == PORT_W)  && (req_out_vc_w  ? w_buf_can_vc1  : w_buf_can_vc0);
    assign req_w_from_pe = req_valid_pe && (req_out_pe == PORT_W)  && (req_out_vc_pe ? w_buf_can_vc1  : w_buf_can_vc0);

    assign req_n_from_n  = req_valid_n  && (req_out_n  == PORT_N)  && (req_out_vc_n  ? n_buf_can_vc1  : n_buf_can_vc0);
    assign req_n_from_e  = req_valid_e  && (req_out_e  == PORT_N)  && (req_out_vc_e  ? n_buf_can_vc1  : n_buf_can_vc0);
    assign req_n_from_w  = req_valid_w  && (req_out_w  == PORT_N)  && (req_out_vc_w  ? n_buf_can_vc1  : n_buf_can_vc0);
    assign req_n_from_pe = req_valid_pe && (req_out_pe == PORT_N)  && (req_out_vc_pe ? n_buf_can_vc1  : n_buf_can_vc0);

    assign req_s_from_s  = req_valid_s  && (req_out_s  == PORT_S)  && (req_out_vc_s  ? s_buf_can_vc1  : s_buf_can_vc0);
    assign req_s_from_e  = req_valid_e  && (req_out_e  == PORT_S)  && (req_out_vc_e  ? s_buf_can_vc1  : s_buf_can_vc0);
    assign req_s_from_w  = req_valid_w  && (req_out_w  == PORT_S)  && (req_out_vc_w  ? s_buf_can_vc1  : s_buf_can_vc0);
    assign req_s_from_pe = req_valid_pe && (req_out_pe == PORT_S)  && (req_out_vc_pe ? s_buf_can_vc1  : s_buf_can_vc0);

    assign req_pe_from_n = req_valid_n && (req_out_n == PORT_PE) && (internal_vc ? pe_buf_can_vc1 : pe_buf_can_vc0);
    assign req_pe_from_s = req_valid_s && (req_out_s == PORT_PE) && (internal_vc ? pe_buf_can_vc1 : pe_buf_can_vc0);
    assign req_pe_from_e = req_valid_e && (req_out_e == PORT_PE) && (internal_vc ? pe_buf_can_vc1 : pe_buf_can_vc0);
    assign req_pe_from_w = req_valid_w && (req_out_w == PORT_PE) && (internal_vc ? pe_buf_can_vc1 : pe_buf_can_vc0);

    // Contention detection per output port.
    assign e_multi_req  = req_e_from_e & req_e_from_pe;
    assign w_multi_req  = req_w_from_w & req_w_from_pe;
    assign n_multi_req  = (req_n_from_n  & req_n_from_e)  | (req_n_from_n  & req_n_from_w)  |
                          (req_n_from_n  & req_n_from_pe) | (req_n_from_e  & req_n_from_w)  |
                          (req_n_from_e  & req_n_from_pe) | (req_n_from_w  & req_n_from_pe);
    assign s_multi_req  = (req_s_from_s  & req_s_from_e)  | (req_s_from_s  & req_s_from_w)  |
                          (req_s_from_s  & req_s_from_pe) | (req_s_from_e  & req_s_from_w)  |
                          (req_s_from_e  & req_s_from_pe) | (req_s_from_w  & req_s_from_pe);
    assign pe_multi_req = (req_pe_from_n & req_pe_from_e) | (req_pe_from_n & req_pe_from_s) |
                          (req_pe_from_n & req_pe_from_w) | (req_pe_from_e & req_pe_from_s) |
                          (req_pe_from_e & req_pe_from_w) | (req_pe_from_s & req_pe_from_w);

    // E output arbitration and grant generation.
    always @(*) begin
        e_grant_valid  = 1'b0;
        e_grant_src    = PORT_E;
        e_grant_out_vc = 1'b0;
        e_grant_pkt    = {PACKET_WIDTH{1'b0}};
        e_advance_ptr  = 1'b0;
        e_next_ptr     = cur_vc_e_ptr;
        e_pick_idx     = rr2_pick(cur_vc_e_ptr, req_e_from_e, req_e_from_pe);

        case (e_pick_idx)
            3'd0: begin
                e_grant_valid  = 1'b1;
                e_grant_src    = PORT_E;
                e_grant_out_vc = req_out_vc_e;
                e_grant_pkt    = req_pkt_e;
                e_next_ptr     = 1'b1;
            end
            3'd1: begin
                e_grant_valid  = 1'b1;
                e_grant_src    = PORT_PE;
                e_grant_out_vc = req_out_vc_pe;
                e_grant_pkt    = req_pkt_pe;
                e_next_ptr     = 1'b0;
            end
            default: begin
                e_grant_valid  = 1'b0;
                e_next_ptr     = cur_vc_e_ptr;
            end
        endcase

        e_advance_ptr = e_grant_valid && e_multi_req;
    end

    // W output arbitration and grant generation.
    always @(*) begin
        w_grant_valid  = 1'b0;
        w_grant_src    = PORT_W;
        w_grant_out_vc = 1'b0;
        w_grant_pkt    = {PACKET_WIDTH{1'b0}};
        w_advance_ptr  = 1'b0;
        w_next_ptr     = cur_vc_w_ptr;

        case (rr2_pick(cur_vc_w_ptr, req_w_from_w, req_w_from_pe))
            3'd0: begin
                w_grant_valid  = 1'b1;
                w_grant_src    = PORT_W;
                w_grant_out_vc = req_out_vc_w;
                w_grant_pkt    = req_pkt_w;
                w_next_ptr     = 1'b1;
            end
            3'd1: begin
                w_grant_valid  = 1'b1;
                w_grant_src    = PORT_PE;
                w_grant_out_vc = req_out_vc_pe;
                w_grant_pkt    = req_pkt_pe;
                w_next_ptr     = 1'b0;
            end
            default: begin
                w_grant_valid  = 1'b0;
                w_next_ptr     = cur_vc_w_ptr;
            end
        endcase

        w_advance_ptr = w_grant_valid && w_multi_req;
    end

    // N output arbitration and grant generation.
    always @(*) begin
        n_grant_valid  = 1'b0;
        n_grant_src    = PORT_N;
        n_grant_out_vc = 1'b0;
        n_grant_pkt    = {PACKET_WIDTH{1'b0}};
        n_advance_ptr  = 1'b0;
        n_next_ptr     = cur_vc_n_ptr;
        n_pick_idx     = rr4_pick(cur_vc_n_ptr, req_n_from_n, req_n_from_e, req_n_from_w, req_n_from_pe);

        case (n_pick_idx)
            3'd0: begin
                n_grant_valid  = 1'b1;
                n_grant_src    = PORT_N;
                n_grant_out_vc = req_out_vc_n;
                n_grant_pkt    = req_pkt_n;
                n_next_ptr     = 2'd1;
            end
            3'd1: begin
                n_grant_valid  = 1'b1;
                n_grant_src    = PORT_E;
                n_grant_out_vc = req_out_vc_e;
                n_grant_pkt    = req_pkt_e;
                n_next_ptr     = 2'd2;
            end
            3'd2: begin
                n_grant_valid  = 1'b1;
                n_grant_src    = PORT_W;
                n_grant_out_vc = req_out_vc_w;
                n_grant_pkt    = req_pkt_w;
                n_next_ptr     = 2'd3;
            end
            3'd3: begin
                n_grant_valid  = 1'b1;
                n_grant_src    = PORT_PE;
                n_grant_out_vc = req_out_vc_pe;
                n_grant_pkt    = req_pkt_pe;
                n_next_ptr     = 2'd0;
            end
            default: begin
                n_grant_valid  = 1'b0;
                n_next_ptr     = cur_vc_n_ptr;
            end
        endcase

        n_advance_ptr = n_grant_valid && n_multi_req;
    end

    // S output arbitration and grant generation.
    always @(*) begin
        s_grant_valid  = 1'b0;
        s_grant_src    = PORT_S;
        s_grant_out_vc = 1'b0;
        s_grant_pkt    = {PACKET_WIDTH{1'b0}};
        s_advance_ptr  = 1'b0;
        s_next_ptr     = cur_vc_s_ptr;
        s_pick_idx     = rr4_pick(cur_vc_s_ptr, req_s_from_s, req_s_from_e, req_s_from_w, req_s_from_pe);

        case (s_pick_idx)
            3'd0: begin
                s_grant_valid  = 1'b1;
                s_grant_src    = PORT_S;
                s_grant_out_vc = req_out_vc_s;
                s_grant_pkt    = req_pkt_s;
                s_next_ptr     = 2'd1;
            end
            3'd1: begin
                s_grant_valid  = 1'b1;
                s_grant_src    = PORT_E;
                s_grant_out_vc = req_out_vc_e;
                s_grant_pkt    = req_pkt_e;
                s_next_ptr     = 2'd2;
            end
            3'd2: begin
                s_grant_valid  = 1'b1;
                s_grant_src    = PORT_W;
                s_grant_out_vc = req_out_vc_w;
                s_grant_pkt    = req_pkt_w;
                s_next_ptr     = 2'd3;
            end
            3'd3: begin
                s_grant_valid  = 1'b1;
                s_grant_src    = PORT_PE;
                s_grant_out_vc = req_out_vc_pe;
                s_grant_pkt    = req_pkt_pe;
                s_next_ptr     = 2'd0;
            end
            default: begin
                s_grant_valid  = 1'b0;
                s_next_ptr     = cur_vc_s_ptr;
            end
        endcase

        s_advance_ptr = s_grant_valid && s_multi_req;
    end

    // PE output arbitration and grant generation.
    always @(*) begin
        pe_grant_valid  = 1'b0;
        pe_grant_src    = PORT_N;
        pe_grant_out_vc = internal_vc;
        pe_grant_pkt    = {PACKET_WIDTH{1'b0}};
        pe_advance_ptr  = 1'b0;
        pe_next_ptr     = cur_vc_pe_ptr;
        pe_pick_idx     = rr4_pick(cur_vc_pe_ptr, req_pe_from_n, req_pe_from_e, req_pe_from_s, req_pe_from_w);

        case (pe_pick_idx)
            3'd0: begin
                pe_grant_valid  = 1'b1;
                pe_grant_src    = PORT_N;
                pe_grant_out_vc = internal_vc;
                pe_grant_pkt    = req_pkt_n;
                pe_next_ptr     = 2'd1;
            end
            3'd1: begin
                pe_grant_valid  = 1'b1;
                pe_grant_src    = PORT_E;
                pe_grant_out_vc = internal_vc;
                pe_grant_pkt    = req_pkt_e;
                pe_next_ptr     = 2'd2;
            end
            3'd2: begin
                pe_grant_valid  = 1'b1;
                pe_grant_src    = PORT_S;
                pe_grant_out_vc = internal_vc;
                pe_grant_pkt    = req_pkt_s;
                pe_next_ptr     = 2'd3;
            end
            3'd3: begin
                pe_grant_valid  = 1'b1;
                pe_grant_src    = PORT_W;
                pe_grant_out_vc = internal_vc;
                pe_grant_pkt    = req_pkt_w;
                pe_next_ptr     = 2'd0;
            end
            default: begin
                pe_grant_valid  = 1'b0;
                pe_next_ptr     = cur_vc_pe_ptr;
            end
        endcase

        pe_advance_ptr = pe_grant_valid && pe_multi_req;
    end

    // Sequential state update.
    // Order of actions in one cycle:
    // 1) Clear output slots that successfully transmitted on the external VC.
    // 2) Capture newly received packets into input slots for the external VC.
    // 3) Move granted internal-VC packets from input slots into output slots.
    // 4) Update round-robin pointers for the internal VC when contention occurred.
    always @(posedge clk) begin
        if (reset) begin
            // Clear all buffer contents and valid bits.
            for (i = 0; i < 10; i = i + 1) begin
                in_buf_data[i]   <= {PACKET_WIDTH{1'b0}};
                in_buf_valid[i]  <= 1'b0;
                out_buf_data[i]  <= {PACKET_WIDTH{1'b0}};
                out_buf_valid[i] <= 1'b0;
            end

            // Initialize round-robin pointers to their first requester.
            e_rr_ptr_vc0  <= 1'b0;
            e_rr_ptr_vc1  <= 1'b0;
            w_rr_ptr_vc0  <= 1'b0;
            w_rr_ptr_vc1  <= 1'b0;
            n_rr_ptr_vc0  <= 2'd0;
            n_rr_ptr_vc1  <= 2'd0;
            s_rr_ptr_vc0  <= 2'd0;
            s_rr_ptr_vc1  <= 2'd0;
            pe_rr_ptr_vc0 <= 2'd0;
            pe_rr_ptr_vc1 <= 2'd0;
        end else begin
            // Clear output slots whose packets were transmitted on this cycle.
            if (n_tx_fire)
                out_buf_valid[buf_idx(PORT_N,  external_vc)] <= 1'b0;
            if (s_tx_fire)
                out_buf_valid[buf_idx(PORT_S,  external_vc)] <= 1'b0;
            if (e_tx_fire)
                out_buf_valid[buf_idx(PORT_E,  external_vc)] <= 1'b0;
            if (w_tx_fire)
                out_buf_valid[buf_idx(PORT_W,  external_vc)] <= 1'b0;
            if (pe_tx_fire)
                out_buf_valid[buf_idx(PORT_PE, external_vc)] <= 1'b0;

            // Capture incoming packets for the VC currently active on external links.
            if (n_si && n_ri) begin
                in_buf_data[buf_idx(PORT_N, external_vc)]  <= n_di;
                in_buf_valid[buf_idx(PORT_N, external_vc)] <= 1'b1;
            end
            if (s_si && s_ri) begin
                in_buf_data[buf_idx(PORT_S, external_vc)]  <= s_di;
                in_buf_valid[buf_idx(PORT_S, external_vc)] <= 1'b1;
            end
            if (e_si && e_ri) begin
                in_buf_data[buf_idx(PORT_E, external_vc)]  <= e_di;
                in_buf_valid[buf_idx(PORT_E, external_vc)] <= 1'b1;
            end
            if (w_si && w_ri) begin
                in_buf_data[buf_idx(PORT_W, external_vc)]  <= w_di;
                in_buf_valid[buf_idx(PORT_W, external_vc)] <= 1'b1;
            end
            if (pe_si && pe_ri) begin
                in_buf_data[buf_idx(PORT_PE, external_vc)]  <= pe_di;
                in_buf_valid[buf_idx(PORT_PE, external_vc)] <= 1'b1;
            end

            // Commit East output grant.
            if (e_grant_valid) begin
                out_buf_data[buf_idx(PORT_E, e_grant_out_vc)]  <= e_grant_pkt;
                out_buf_valid[buf_idx(PORT_E, e_grant_out_vc)] <= 1'b1;
                in_buf_valid[buf_idx(e_grant_src, internal_vc)] <= 1'b0;
            end

            // Commit West output grant.
            if (w_grant_valid) begin
                out_buf_data[buf_idx(PORT_W, w_grant_out_vc)]  <= w_grant_pkt;
                out_buf_valid[buf_idx(PORT_W, w_grant_out_vc)] <= 1'b1;
                in_buf_valid[buf_idx(w_grant_src, internal_vc)] <= 1'b0;
            end

            // Commit North output grant.
            if (n_grant_valid) begin
                out_buf_data[buf_idx(PORT_N, n_grant_out_vc)]  <= n_grant_pkt;
                out_buf_valid[buf_idx(PORT_N, n_grant_out_vc)] <= 1'b1;
                in_buf_valid[buf_idx(n_grant_src, internal_vc)] <= 1'b0;
            end

            // Commit South output grant.
            if (s_grant_valid) begin
                out_buf_data[buf_idx(PORT_S, s_grant_out_vc)]  <= s_grant_pkt;
                out_buf_valid[buf_idx(PORT_S, s_grant_out_vc)] <= 1'b1;
                in_buf_valid[buf_idx(s_grant_src, internal_vc)] <= 1'b0;
            end

            // Commit PE output grant.
            if (pe_grant_valid) begin
                out_buf_data[buf_idx(PORT_PE, pe_grant_out_vc)]  <= pe_grant_pkt;
                out_buf_valid[buf_idx(PORT_PE, pe_grant_out_vc)] <= 1'b1;
                in_buf_valid[buf_idx(pe_grant_src, internal_vc)] <= 1'b0;
            end

            // Update the round-robin pointers for the VC processed internally this cycle.
            if (internal_vc == 1'b0) begin
                if (e_advance_ptr)
                    e_rr_ptr_vc0  <= e_next_ptr;
                if (w_advance_ptr)
                    w_rr_ptr_vc0  <= w_next_ptr;
                if (n_advance_ptr)
                    n_rr_ptr_vc0  <= n_next_ptr;
                if (s_advance_ptr)
                    s_rr_ptr_vc0  <= s_next_ptr;
                if (pe_advance_ptr)
                    pe_rr_ptr_vc0 <= pe_next_ptr;
            end else begin
                if (e_advance_ptr)
                    e_rr_ptr_vc1  <= e_next_ptr;
                if (w_advance_ptr)
                    w_rr_ptr_vc1  <= w_next_ptr;
                if (n_advance_ptr)
                    n_rr_ptr_vc1  <= n_next_ptr;
                if (s_advance_ptr)
                    s_rr_ptr_vc1  <= s_next_ptr;
                if (pe_advance_ptr)
                    pe_rr_ptr_vc1 <= pe_next_ptr;
            end
        end
    end

endmodule
