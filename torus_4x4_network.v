module torus_4x4_network #(
    parameter PACKET_WIDTH = 64,
    parameter X_SIZE = 4,
    parameter Y_SIZE = 4,
    parameter NODE_COUNT = 16
)(
    input  wire                               clk,
    input  wire                               reset,

    input  wire [NODE_COUNT-1:0]              pe_si,
    output wire [NODE_COUNT-1:0]              pe_ri,
    input  wire [NODE_COUNT*PACKET_WIDTH-1:0] pe_di,
    output wire [NODE_COUNT-1:0]              pe_so,
    input  wire [NODE_COUNT-1:0]              pe_ro,
    output wire [NODE_COUNT*PACKET_WIDTH-1:0] pe_do
);

    reg polarity;

    /* Router-to-router send/data buses. */
    wire [NODE_COUNT-1:0] n_so_bus;
    wire [NODE_COUNT*PACKET_WIDTH-1:0] n_do_bus;
    wire [NODE_COUNT-1:0] s_so_bus;
    wire [NODE_COUNT*PACKET_WIDTH-1:0] s_do_bus;
    wire [NODE_COUNT-1:0] e_so_bus;
    wire [NODE_COUNT*PACKET_WIDTH-1:0] e_do_bus;
    wire [NODE_COUNT-1:0] w_so_bus;
    wire [NODE_COUNT*PACKET_WIDTH-1:0] w_do_bus;

    /* Router input-ready buses. */
    wire [NODE_COUNT-1:0] n_ri_bus;
    wire [NODE_COUNT-1:0] s_ri_bus;
    wire [NODE_COUNT-1:0] e_ri_bus;
    wire [NODE_COUNT-1:0] w_ri_bus;

    always @(posedge clk) begin
        if (reset)
            polarity <= 1'b0;
        else
            polarity <= ~polarity;
    end

    genvar gx;
    genvar gy;
    generate
        for (gy = 0; gy < Y_SIZE; gy = gy + 1) begin : GEN_Y
            for (gx = 0; gx < X_SIZE; gx = gx + 1) begin : GEN_X
                localparam integer IDX       = gy * X_SIZE + gx;
                localparam integer NORTH_Y   = (gy == 0) ? (Y_SIZE - 1) : (gy - 1);
                localparam integer SOUTH_Y   = (gy == (Y_SIZE - 1)) ? 0 : (gy + 1);
                localparam integer WEST_X    = (gx == 0) ? (X_SIZE - 1) : (gx - 1);
                localparam integer EAST_X    = (gx == (X_SIZE - 1)) ? 0 : (gx + 1);
                localparam integer NORTH_IDX = NORTH_Y * X_SIZE + gx;
                localparam integer SOUTH_IDX = SOUTH_Y * X_SIZE + gx;
                localparam integer WEST_IDX  = gy * X_SIZE + WEST_X;
                localparam integer EAST_IDX  = gy * X_SIZE + EAST_X;
                localparam [3:0] LOCAL_X     = gx;
                localparam [3:0] LOCAL_Y     = gy;

                /*
                 * Same-side torus connection rule:
                 * - N_out of a router goes to N_in of the north neighbor.
                 * - S_out of a router goes to S_in of the south neighbor.
                 * - E_out of a router goes to E_in of the east neighbor.
                 * - W_out of a router goes to W_in of the west neighbor.
                 */
                gold_router #(
                    .PACKET_WIDTH(PACKET_WIDTH)
                ) u_router (
                    .clk(clk),
                    .reset(reset),
                    .polarity(polarity),
                    .local_x(LOCAL_X),
                    .local_y(LOCAL_Y),

                    .n_si(n_so_bus[SOUTH_IDX]),
                    .n_ri(n_ri_bus[IDX]),
                    .n_di(n_do_bus[(SOUTH_IDX*PACKET_WIDTH) +: PACKET_WIDTH]),
                    .n_so(n_so_bus[IDX]),
                    .n_ro(n_ri_bus[NORTH_IDX]),
                    .n_do(n_do_bus[(IDX*PACKET_WIDTH) +: PACKET_WIDTH]),

                    .s_si(s_so_bus[NORTH_IDX]),
                    .s_ri(s_ri_bus[IDX]),
                    .s_di(s_do_bus[(NORTH_IDX*PACKET_WIDTH) +: PACKET_WIDTH]),
                    .s_so(s_so_bus[IDX]),
                    .s_ro(s_ri_bus[SOUTH_IDX]),
                    .s_do(s_do_bus[(IDX*PACKET_WIDTH) +: PACKET_WIDTH]),

                    .e_si(e_so_bus[WEST_IDX]),
                    .e_ri(e_ri_bus[IDX]),
                    .e_di(e_do_bus[(WEST_IDX*PACKET_WIDTH) +: PACKET_WIDTH]),
                    .e_so(e_so_bus[IDX]),
                    .e_ro(e_ri_bus[EAST_IDX]),
                    .e_do(e_do_bus[(IDX*PACKET_WIDTH) +: PACKET_WIDTH]),

                    .w_si(w_so_bus[EAST_IDX]),
                    .w_ri(w_ri_bus[IDX]),
                    .w_di(w_do_bus[(EAST_IDX*PACKET_WIDTH) +: PACKET_WIDTH]),
                    .w_so(w_so_bus[IDX]),
                    .w_ro(w_ri_bus[WEST_IDX]),
                    .w_do(w_do_bus[(IDX*PACKET_WIDTH) +: PACKET_WIDTH]),

                    .pe_si(pe_si[IDX]),
                    .pe_ri(pe_ri[IDX]),
                    .pe_di(pe_di[(IDX*PACKET_WIDTH) +: PACKET_WIDTH]),
                    .pe_so(pe_so[IDX]),
                    .pe_ro(pe_ro[IDX]),
                    .pe_do(pe_do[(IDX*PACKET_WIDTH) +: PACKET_WIDTH])
                );
            end
        end
    endgenerate

endmodule
