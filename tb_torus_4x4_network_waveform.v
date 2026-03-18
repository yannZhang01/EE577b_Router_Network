`timescale 1ns/1ps

module tb_torus_4x4_network_waveform;

    parameter PACKET_WIDTH = 64;
    parameter X_SIZE       = 4;
    parameter Y_SIZE       = 4;
    parameter NODE_COUNT   = 16;
    parameter WAVE_CASE    = 1;

    localparam [2:0] PORT_N  = 3'd0;
    localparam [2:0] PORT_S  = 3'd1;
    localparam [2:0] PORT_E  = 3'd2;
    localparam [2:0] PORT_W  = 3'd3;
    localparam [2:0] PORT_PE = 3'd4;

    reg                                clk;
    reg                                reset;
    reg  [NODE_COUNT-1:0]              pe_si;
    wire [NODE_COUNT-1:0]              pe_ri;
    reg  [NODE_COUNT*PACKET_WIDTH-1:0] pe_di;
    wire [NODE_COUNT-1:0]              pe_so;
    reg  [NODE_COUNT-1:0]              pe_ro;
    wire [NODE_COUNT*PACKET_WIDTH-1:0] pe_do;

    integer pass_count;
    integer fail_count;

    // Waveform markers
    reg [7:0] dbg_test_id;
    reg [7:0] dbg_phase;
    reg       dbg_test_active;
    reg       dbg_case_start_pulse;
    reg       dbg_case_done_pulse;

    // Helpful node indices
    localparam integer IDX_00 = 0;
    localparam integer IDX_10 = 1;
    localparam integer IDX_30 = 3;
    localparam integer IDX_01 = 4;
    localparam integer IDX_11 = 5;
    localparam integer IDX_21 = 6;
    localparam integer IDX_03 = 12;

    torus_4x4_network dut (
        .clk(clk),
        .reset(reset),
        .pe_si(pe_si),
        .pe_ri(pe_ri),
        .pe_di(pe_di),
        .pe_so(pe_so),
        .pe_ro(pe_ro),
        .pe_do(pe_do)
    );

    always #5 clk = ~clk;

    // -------------------------
    // Helper functions
    // -------------------------
    function [63:0] build_packet;
        input [3:0] src_x;
        input [3:0] src_y;
        input [3:0] dst_x;
        input [3:0] dst_y;
        input [31:0] payload;
        integer east_steps;
        integer west_steps;
        integer south_steps;
        integer north_steps;
        reg dirx;
        reg diry;
        reg vc;
        reg [3:0] hops_x;
        reg [3:0] hops_y;
        begin
            east_steps = dst_x - src_x; if (east_steps < 0) east_steps = east_steps + X_SIZE;
            west_steps = src_x - dst_x; if (west_steps < 0) west_steps = west_steps + X_SIZE;
            south_steps = dst_y - src_y; if (south_steps < 0) south_steps = south_steps + Y_SIZE;
            north_steps = src_y - dst_y; if (north_steps < 0) north_steps = north_steps + Y_SIZE;

            if (east_steps <= west_steps) begin dirx = 1'b0; hops_x = east_steps[3:0]; end
            else begin dirx = 1'b1; hops_x = west_steps[3:0]; end

            if (south_steps <= north_steps) begin diry = 1'b0; hops_y = south_steps[3:0]; end
            else begin diry = 1'b1; hops_y = north_steps[3:0]; end

            vc = 1'b0;
            if (hops_x != 0) begin
                if ((dirx == 1'b0 && src_x == 4'd3) || (dirx == 1'b1 && src_x == 4'd0)) vc = 1'b1;
            end else if (hops_y != 0) begin
                if ((diry == 1'b0 && src_y == 4'd3) || (diry == 1'b1 && src_y == 4'd0)) vc = 1'b1;
            end

            build_packet = 64'd0;
            build_packet[63]    = vc;
            build_packet[62]    = dirx;
            build_packet[61]    = diry;
            build_packet[60:57] = hops_x;
            build_packet[56:53] = hops_y;
            build_packet[52:49] = src_x;
            build_packet[48:45] = src_y;
            build_packet[44:41] = dst_x;
            build_packet[40:37] = dst_y;
            build_packet[31:0]  = payload;
        end
    endfunction

    function [3:0] pkt_hops_x;
        input [63:0] packet;
        begin
            pkt_hops_x = packet[60:57];
        end
    endfunction

    function [3:0] pkt_hops_y;
        input [63:0] packet;
        begin
            pkt_hops_y = packet[56:53];
        end
    endfunction

    function [31:0] pkt_payload;
        input [63:0] packet;
        begin
            pkt_payload = packet[31:0];
        end
    endfunction

    task clear_drive_signals;
        begin
            pe_si = {NODE_COUNT{1'b0}};
            pe_di = {(NODE_COUNT*PACKET_WIDTH){1'b0}};
            pe_ro = {NODE_COUNT{1'b1}};
        end
    endtask

    task pulse_case_markers;
        begin
            dbg_case_start_pulse = 1'b1;
            #1;
            dbg_case_start_pulse = 1'b0;
        end
    endtask

    task pulse_done_markers;
        begin
            dbg_case_done_pulse = 1'b1;
            #1;
            dbg_case_done_pulse = 1'b0;
        end
    endtask

    task inject_packet;
        input integer idx;
        input [63:0] packet;
        integer wait_cycles;
        begin
            wait_cycles = 0;
            while (pe_ri[idx] !== 1'b1) begin
                @(posedge clk);
                wait_cycles = wait_cycles + 1;
                if (wait_cycles > 200) begin
                    $display("FAIL: injection timeout at node %0d", idx);
                    fail_count = fail_count + 1;
                    disable inject_packet;
                end
            end
            @(negedge clk);
            pe_di[(idx*PACKET_WIDTH) +: PACKET_WIDTH] = packet;
            pe_si[idx] = 1'b1;
            @(posedge clk);
            @(negedge clk);
            pe_si[idx] = 1'b0;
            pe_di[(idx*PACKET_WIDTH) +: PACKET_WIDTH] = {PACKET_WIDTH{1'b0}};
        end
    endtask

    task wait_delivery;
        input integer idx;
        input integer timeout_cycles;
        output [63:0] packet;
        output integer seen;
        integer t;
        begin
            packet = 64'd0;
            seen   = 0;
            for (t = 0; t < timeout_cycles; t = t + 1) begin
                @(posedge clk);
                if (pe_so[idx] && pe_ro[idx]) begin
                    packet = pe_do[(idx*PACKET_WIDTH) +: PACKET_WIDTH];
                    seen   = 1;
                    t      = timeout_cycles;
                end
            end
        end
    endtask

    task expect_payload;
        input integer idx;
        input [31:0] exp_payload;
        input integer timeout_cycles;
        reg [63:0] packet;
        integer seen;
        begin
            wait_delivery(idx, timeout_cycles, packet, seen);
            if (!seen) begin
                $display("FAIL: no delivery observed at node %0d", idx);
                fail_count = fail_count + 1;
            end else if (pkt_payload(packet) !== exp_payload) begin
                $display("FAIL: wrong payload at node %0d, got %h expected %h", idx, pkt_payload(packet), exp_payload);
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: delivery at node %0d payload=%h", idx, pkt_payload(packet));
                pass_count = pass_count + 1;
            end
        end
    endtask

    // -------------------------
    // Waveform-friendly debug wires
    // -------------------------
    wire dbg_polarity = dut.polarity;

    // Router (0,0) east link debug
    wire dbg_r00_e_so = dut.e_so_bus[IDX_00];
    wire dbg_r00_e_ro = dut.e_ri_bus[IDX_10];
    wire [63:0] dbg_r00_e_do = dut.e_do_bus[(IDX_00*PACKET_WIDTH) +: PACKET_WIDTH];

    // Router (1,1) N_out contention debug
    wire dbg_req_n_from_e = dut.GEN_Y[1].GEN_X[1].u_router.req_n_from_e;
    wire dbg_req_n_from_w = dut.GEN_Y[1].GEN_X[1].u_router.req_n_from_w;
    wire dbg_n_grant_valid = dut.GEN_Y[1].GEN_X[1].u_router.n_grant_valid;
    wire [2:0] dbg_n_grant_src = dut.GEN_Y[1].GEN_X[1].u_router.n_grant_src;
    wire [1:0] dbg_n_rr_ptr_vc0 = dut.GEN_Y[1].GEN_X[1].u_router.n_rr_ptr_vc0;

    // Hop-update observation path for test 5
    wire [63:0] dbg_r00_e_out_vc0 = dut.GEN_Y[0].GEN_X[0].u_router.out_buf_data[4];
    wire        dbg_r00_e_out_vc0_valid = dut.GEN_Y[0].GEN_X[0].u_router.out_buf_valid[4];
    wire [63:0] dbg_r10_e_out_vc0 = dut.GEN_Y[0].GEN_X[1].u_router.out_buf_data[4];
    wire        dbg_r10_e_out_vc0_valid = dut.GEN_Y[0].GEN_X[1].u_router.out_buf_valid[4];
    wire [63:0] dbg_r20_s_out_vc0 = dut.GEN_Y[0].GEN_X[2].u_router.out_buf_data[2];
    wire        dbg_r20_s_out_vc0_valid = dut.GEN_Y[0].GEN_X[2].u_router.out_buf_valid[2];

    initial begin
        clk = 1'b0;
        reset = 1'b1;
        pass_count = 0;
        fail_count = 0;
        dbg_test_id = 0;
        dbg_phase = 0;
        dbg_test_active = 0;
        dbg_case_start_pulse = 0;
        dbg_case_done_pulse = 0;
        clear_drive_signals;

        $display("WAVEFORM TB: WAVE_CASE=%0d", WAVE_CASE);

        // Reset phase
        dbg_test_id = 8'd1;
        dbg_phase = 8'd0;
        dbg_test_active = 1'b1;
        pulse_case_markers;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        #1;
        if (dbg_polarity !== 1'b0) begin
            $display("FAIL: reset polarity init unexpected");
            fail_count = fail_count + 1;
        end
        @(posedge clk);
        #1;
        if (dbg_polarity !== 1'b1) begin
            $display("FAIL: polarity did not toggle after reset release");
            fail_count = fail_count + 1;
        end else begin
            $display("PASS: reset behavior observed");
            pass_count = pass_count + 1;
        end
        dbg_test_active = 1'b0;
        pulse_done_markers;
        repeat (4) @(posedge clk);

        case (WAVE_CASE)
            1: begin
                // Single-hop East, non-blocking handshake
                dbg_test_id = 8'd2;
                dbg_phase = 8'd0;
                dbg_test_active = 1'b1;
                pulse_case_markers;
                inject_packet(IDX_00, build_packet(4'd0,4'd0,4'd1,4'd0,32'h00001001));
                dbg_phase = 8'd1;
                expect_payload(IDX_10, 32'h00001001, 80);
                dbg_test_active = 1'b0;
                pulse_done_markers;
            end

            2: begin
                // Multi-hop X then Y with hop updates
                dbg_test_id = 8'd5;
                dbg_phase = 8'd0;
                dbg_test_active = 1'b1;
                pulse_case_markers;
                inject_packet(IDX_00, build_packet(4'd0,4'd0,4'd2,4'd1,32'h00001004));
                dbg_phase = 8'd1;
                expect_payload(6, 32'h00001004, 150);
                dbg_test_active = 1'b0;
                pulse_done_markers;
            end

            3: begin
                // Blocking handshake on East link between node 0 and node 1
                dbg_test_id = 8'd7;
                dbg_phase = 8'd0;
                dbg_test_active = 1'b1;
                pulse_case_markers;
                @(negedge clk);
                dut.e_ri_bus[IDX_10] = 1'b0;
                inject_packet(IDX_00, build_packet(4'd0,4'd0,4'd1,4'd0,32'h00002001));
                dbg_phase = 8'd1;
                repeat (10) @(posedge clk);
                @(negedge clk);
                dut.e_ri_bus[IDX_10] = 1'b1;
                dbg_phase = 8'd2;
                expect_payload(IDX_10, 32'h00002001, 120);
                dbg_test_active = 1'b0;
                pulse_done_markers;
            end

            4: begin
                // Deterministic contention at router (1,1) N_out via hierarchical preload
                dbg_test_id = 8'd8;
                dbg_phase = 8'd0;
                dbg_test_active = 1'b1;
                pulse_case_markers;

                // Align to internal_vc=0
                while (dut.polarity !== 1'b0) @(posedge clk);
                @(negedge clk);
                // Clear local state of router (1,1)
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4] = 1'b0; // E,vc0
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6] = 1'b0; // W,vc0
                dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[0] = 1'b0; // N,vc0
                dut.GEN_Y[1].GEN_X[1].u_router.n_rr_ptr_vc0 = 2'd0;
                // Preload two packets that both request N_out
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[4] = build_packet(4'd0,4'd1,4'd1,4'd0,32'h00003001);
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4] = 1'b1;
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[6] = build_packet(4'd2,4'd1,4'd1,4'd0,32'h00003002);
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6] = 1'b1;
                dbg_phase = 8'd1;
                @(negedge clk);
                #1;
                if (!(dbg_req_n_from_e && dbg_req_n_from_w && dbg_n_grant_valid)) begin
                    $display("FAIL: deterministic contention was not created at router (1,1) N_out");
                    fail_count = fail_count + 1;
                end else if (dbg_n_grant_src !== PORT_E) begin
                    $display("FAIL: first deterministic contention round granted wrong source, expected E_in");
                    fail_count = fail_count + 1;
                end else begin
                    $display("PASS: first contention round granted E_in");
                    pass_count = pass_count + 1;
                end
                @(posedge clk); #1;
                if (dbg_n_rr_ptr_vc0 !== 2'd2) begin
                    $display("FAIL: N_out RR pointer did not rotate to W after first round");
                    fail_count = fail_count + 1;
                end else begin
                    $display("PASS: N_out RR pointer rotated after first round");
                    pass_count = pass_count + 1;
                end

                // Second round
                while (dut.polarity !== 1'b0) @(posedge clk);
                @(negedge clk);
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4] = 1'b0;
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6] = 1'b0;
                dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[0] = 1'b0;
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[4] = build_packet(4'd0,4'd1,4'd1,4'd0,32'h00003003);
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4] = 1'b1;
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[6] = build_packet(4'd2,4'd1,4'd1,4'd0,32'h00003004);
                dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6] = 1'b1;
                dbg_phase = 8'd2;
                @(negedge clk);
                #1;
                if (!(dbg_req_n_from_e && dbg_req_n_from_w && dbg_n_grant_valid)) begin
                    $display("FAIL: second deterministic contention was not created at router (1,1) N_out");
                    fail_count = fail_count + 1;
                end else if (dbg_n_grant_src !== PORT_W) begin
                    $display("FAIL: second deterministic contention round granted wrong source, expected W_in after rotation");
                    fail_count = fail_count + 1;
                end else begin
                    $display("PASS: second contention round granted W_in after rotation");
                    pass_count = pass_count + 1;
                end
                @(posedge clk); #1;
                if (dbg_n_rr_ptr_vc0 !== 2'd3) begin
                    $display("FAIL: N_out RR pointer did not rotate after second round");
                    fail_count = fail_count + 1;
                end else begin
                    $display("PASS: N_out RR pointer rotated after second round");
                    pass_count = pass_count + 1;
                end
                dbg_test_active = 1'b0;
                pulse_done_markers;
            end

            default: begin
                $display("FAIL: unsupported WAVE_CASE=%0d", WAVE_CASE);
                fail_count = fail_count + 1;
            end
        endcase

        repeat (10) @(posedge clk);
        $display("WAVE TB DONE: pass_count=%0d fail_count=%0d", pass_count, fail_count);
        $finish;
    end

endmodule
