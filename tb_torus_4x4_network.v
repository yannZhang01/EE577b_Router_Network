`timescale 1ns/1ps

module tb_torus_4x4_network;

    parameter PACKET_WIDTH = 64;
    parameter X_SIZE       = 4;
    parameter Y_SIZE       = 4;
    parameter NODE_COUNT   = 16;

    localparam [2:0] PORT_N  = 3'd0;
    localparam [2:0] PORT_S  = 3'd1;
    localparam [2:0] PORT_E  = 3'd2;
    localparam [2:0] PORT_W  = 3'd3;
    localparam [2:0] PORT_PE = 3'd4;

    reg                               clk;
    reg                               reset;
    reg  [NODE_COUNT-1:0]             pe_si;
    wire [NODE_COUNT-1:0]             pe_ri;
    reg  [NODE_COUNT*PACKET_WIDTH-1:0] pe_di;
    wire [NODE_COUNT-1:0]             pe_so;
    reg  [NODE_COUNT-1:0]             pe_ro;
    wire [NODE_COUNT*PACKET_WIDTH-1:0] pe_do;

    integer pass_count;
    integer fail_count;
    integer cycle_count;

    reg [63:0] pkt;
    reg [63:0] rx_pkt0;
    reg [63:0] rx_pkt1;
    integer    seen0;
    integer    seen1;
    integer    report_fd;
    integer    pass_before;
    integer    fail_before;

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

    always @(posedge clk) begin
        if (reset)
            cycle_count <= 0;
        else
            cycle_count <= cycle_count + 1;
    end

    function integer coord_to_idx;
        input integer x;
        input integer y;
        begin
            coord_to_idx = (y * X_SIZE) + x;
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

    function pkt_vc;
        input [63:0] packet;
        begin
            pkt_vc = packet[63];
        end
    endfunction

    function [3:0] pkt_src_x;
        input [63:0] packet;
        begin
            pkt_src_x = packet[52:49];
        end
    endfunction

    function [3:0] pkt_src_y;
        input [63:0] packet;
        begin
            pkt_src_y = packet[48:45];
        end
    endfunction

    function [3:0] pkt_dst_x;
        input [63:0] packet;
        begin
            pkt_dst_x = packet[44:41];
        end
    endfunction

    function [3:0] pkt_dst_y;
        input [63:0] packet;
        begin
            pkt_dst_y = packet[40:37];
        end
    endfunction

    function [31:0] pkt_payload;
        input [63:0] packet;
        begin
            pkt_payload = packet[31:0];
        end
    endfunction

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
            east_steps = dst_x - src_x;
            if (east_steps < 0)
                east_steps = east_steps + X_SIZE;

            west_steps = src_x - dst_x;
            if (west_steps < 0)
                west_steps = west_steps + X_SIZE;

            south_steps = dst_y - src_y;
            if (south_steps < 0)
                south_steps = south_steps + Y_SIZE;

            north_steps = src_y - dst_y;
            if (north_steps < 0)
                north_steps = north_steps + Y_SIZE;

            if (east_steps <= west_steps) begin
                dirx   = 1'b0;
                hops_x = east_steps[3:0];
            end else begin
                dirx   = 1'b1;
                hops_x = west_steps[3:0];
            end

            if (south_steps <= north_steps) begin
                diry   = 1'b0;
                hops_y = south_steps[3:0];
            end else begin
                diry   = 1'b1;
                hops_y = north_steps[3:0];
            end

            vc = 1'b0;
            if (hops_x != 4'd0) begin
                if ((dirx == 1'b0 && src_x == 4'd3) || (dirx == 1'b1 && src_x == 4'd0))
                    vc = 1'b1;
            end else if (hops_y != 4'd0) begin
                if ((diry == 1'b0 && src_y == 4'd3) || (diry == 1'b1 && src_y == 4'd0))
                    vc = 1'b1;
            end

            build_packet          = 64'd0;
            build_packet[63]      = vc;
            build_packet[62]      = dirx;
            build_packet[61]      = diry;
            build_packet[60:57]   = hops_x;
            build_packet[56:53]   = hops_y;
            build_packet[52:49]   = src_x;
            build_packet[48:45]   = src_y;
            build_packet[44:41]   = dst_x;
            build_packet[40:37]   = dst_y;
            build_packet[31:0]    = payload;
        end
    endfunction

    task clear_drive_signals;
        begin
            pe_si = {NODE_COUNT{1'b0}};
            pe_di = {(NODE_COUNT*PACKET_WIDTH){1'b0}};
            pe_ro = {NODE_COUNT{1'b1}};
        end
    endtask

    task inject_packet;
        input integer idx;
        input [63:0] packet;
        integer wait_cycles;
        begin : INJECT_WAIT
            wait_cycles = 0;
            while (pe_ri[idx] !== 1'b1) begin
                @(posedge clk);
                wait_cycles = wait_cycles + 1;
                if (wait_cycles > 200) begin
                    $display("ERROR: injection timeout at node %0d", idx);
                    fail_count = fail_count + 1;
                    disable INJECT_WAIT;
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

    task wait_next_delivery;
        input  integer idx;
        input  integer timeout_cycles;
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

    task wait_delivery_and_check;
        input integer idx;
        input [3:0] exp_dst_x;
        input [3:0] exp_dst_y;
        input [31:0] exp_payload;
        input integer timeout_cycles;
        reg [63:0] packet;
        integer seen;
        begin
            wait_next_delivery(idx, timeout_cycles, packet, seen);
            if (!seen) begin
                $display("FAIL: no delivery observed at node %0d", idx);
                fail_count = fail_count + 1;
            end else if ((pkt_payload(packet) !== exp_payload) ||
                         (pkt_dst_x(packet) !== exp_dst_x) ||
                         (pkt_dst_y(packet) !== exp_dst_y) ||
                         (pkt_hops_x(packet) !== 4'd0) ||
                         (pkt_hops_y(packet) !== 4'd0)) begin
                $display("FAIL: delivery mismatch at node %0d payload=%h dst=(%0d,%0d) hops=(%0d,%0d)",
                         idx, pkt_payload(packet), pkt_dst_x(packet), pkt_dst_y(packet),
                         pkt_hops_x(packet), pkt_hops_y(packet));
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: delivery at node %0d payload=%h", idx, pkt_payload(packet));
                pass_count = pass_count + 1;
            end
        end
    endtask

    task wait_e_send_and_check;
        input integer src_idx;
        input [3:0] exp_hops_x;
        input [3:0] exp_hops_y;
        input       exp_vc;
        input [31:0] exp_payload;
        input integer timeout_cycles;
        reg [63:0] packet;
        integer t;
        integer seen;
        begin
            packet = 64'd0;
            seen   = 0;
            for (t = 0; t < timeout_cycles; t = t + 1) begin
                @(posedge clk);
                if (dut.e_so_bus[src_idx]) begin
                    packet = dut.e_do_bus[(src_idx*PACKET_WIDTH) +: PACKET_WIDTH];
                    seen   = 1;
                    t      = timeout_cycles;
                end
            end
            if (!seen) begin
                $display("FAIL: no East send observed from node %0d", src_idx);
                fail_count = fail_count + 1;
            end else if ((pkt_hops_x(packet) !== exp_hops_x) ||
                         (pkt_hops_y(packet) !== exp_hops_y) ||
                         (pkt_vc(packet) !== exp_vc) ||
                         (pkt_payload(packet) !== exp_payload)) begin
                $display("FAIL: East send mismatch from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: East send from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                pass_count = pass_count + 1;
            end
        end
    endtask

    task wait_w_send_and_check;
        input integer src_idx;
        input [3:0] exp_hops_x;
        input [3:0] exp_hops_y;
        input       exp_vc;
        input [31:0] exp_payload;
        input integer timeout_cycles;
        reg [63:0] packet;
        integer t;
        integer seen;
        begin
            packet = 64'd0;
            seen   = 0;
            for (t = 0; t < timeout_cycles; t = t + 1) begin
                @(posedge clk);
                if (dut.w_so_bus[src_idx]) begin
                    packet = dut.w_do_bus[(src_idx*PACKET_WIDTH) +: PACKET_WIDTH];
                    seen   = 1;
                    t      = timeout_cycles;
                end
            end
            if (!seen) begin
                $display("FAIL: no West send observed from node %0d", src_idx);
                fail_count = fail_count + 1;
            end else if ((pkt_hops_x(packet) !== exp_hops_x) ||
                         (pkt_hops_y(packet) !== exp_hops_y) ||
                         (pkt_vc(packet) !== exp_vc) ||
                         (pkt_payload(packet) !== exp_payload)) begin
                $display("FAIL: West send mismatch from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: West send from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                pass_count = pass_count + 1;
            end
        end
    endtask

    task wait_n_send_and_check;
        input integer src_idx;
        input [3:0] exp_hops_x;
        input [3:0] exp_hops_y;
        input       exp_vc;
        input [31:0] exp_payload;
        input integer timeout_cycles;
        reg [63:0] packet;
        integer t;
        integer seen;
        begin
            packet = 64'd0;
            seen   = 0;
            for (t = 0; t < timeout_cycles; t = t + 1) begin
                @(posedge clk);
                if (dut.n_so_bus[src_idx]) begin
                    packet = dut.n_do_bus[(src_idx*PACKET_WIDTH) +: PACKET_WIDTH];
                    seen   = 1;
                    t      = timeout_cycles;
                end
            end
            if (!seen) begin
                $display("FAIL: no North send observed from node %0d", src_idx);
                fail_count = fail_count + 1;
            end else if ((pkt_hops_x(packet) !== exp_hops_x) ||
                         (pkt_hops_y(packet) !== exp_hops_y) ||
                         (pkt_vc(packet) !== exp_vc) ||
                         (pkt_payload(packet) !== exp_payload)) begin
                $display("FAIL: North send mismatch from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: North send from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                pass_count = pass_count + 1;
            end
        end
    endtask

    task wait_s_send_and_check;
        input integer src_idx;
        input [3:0] exp_hops_x;
        input [3:0] exp_hops_y;
        input       exp_vc;
        input [31:0] exp_payload;
        input integer timeout_cycles;
        reg [63:0] packet;
        integer t;
        integer seen;
        begin
            packet = 64'd0;
            seen   = 0;
            for (t = 0; t < timeout_cycles; t = t + 1) begin
                @(posedge clk);
                if (dut.s_so_bus[src_idx]) begin
                    packet = dut.s_do_bus[(src_idx*PACKET_WIDTH) +: PACKET_WIDTH];
                    seen   = 1;
                    t      = timeout_cycles;
                end
            end
            if (!seen) begin
                $display("FAIL: no South send observed from node %0d", src_idx);
                fail_count = fail_count + 1;
            end else if ((pkt_hops_x(packet) !== exp_hops_x) ||
                         (pkt_hops_y(packet) !== exp_hops_y) ||
                         (pkt_vc(packet) !== exp_vc) ||
                         (pkt_payload(packet) !== exp_payload)) begin
                $display("FAIL: South send mismatch from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: South send from node %0d hx=%0d hy=%0d vc=%0d payload=%h",
                         src_idx, pkt_hops_x(packet), pkt_hops_y(packet), pkt_vc(packet), pkt_payload(packet));
                pass_count = pass_count + 1;
            end
        end
    endtask

    task wait_for_block_condition;
        input integer timeout_cycles;
        integer t;
        integer seen;
        begin
            seen = 0;
            for (t = 0; t < timeout_cycles; t = t + 1) begin
                @(posedge clk);
                if ((dut.e_ri_bus[1] == 1'b0) &&
                    (dut.GEN_Y[0].GEN_X[0].u_router.out_buf_valid[4] == 1'b1) &&
                    (dut.e_so_bus[0] == 1'b0)) begin
                    seen = 1;
                    t = timeout_cycles;
                end
            end
            if (!seen) begin
                $display("FAIL: blocking condition was not observed on the East link between node 0 and node 1");
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: blocking condition observed on the East link between node 0 and node 1");
                pass_count = pass_count + 1;
            end
        end
    endtask

    initial begin
        clk        = 1'b0;
        reset      = 1'b1;
        pass_count = 0;
        fail_count = 0;
        cycle_count = 0;
        clear_drive_signals();

        report_fd = $fopen("tb_torus_4x4_network_report.txt", "w");
        if (report_fd == 0) begin
            $display("ERROR: failed to open tb_torus_4x4_network_report.txt");
            $finish;
        end
        $fdisplay(report_fd, "tb_torus_4x4_network report");
        $fdisplay(report_fd, "==========================");

        $dumpfile("torus_4x4_network_tb.vcd");
        $dumpvars(0, tb_torus_4x4_network);

        repeat (4) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 1: reset behavior */
        $display("TEST 1: reset behavior");
        if (dut.polarity !== 1'b0) begin
            $display("FAIL: polarity is not 0 during reset");
            fail_count = fail_count + 1;
        end else if (pe_ri !== 16'hFFFF) begin
            $display("FAIL: pe_ri is not all ones during reset");
            fail_count = fail_count + 1;
        end else begin
            $display("PASS: reset initializes polarity and clears input readiness state");
            pass_count = pass_count + 1;
        end
        $fdisplay(report_fd, "TEST 1 - reset behavior : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);

        @(negedge clk);
        reset = 1'b0;
        @(posedge clk);
        #1;
        if (dut.polarity !== 1'b1) begin
            $display("FAIL: polarity did not toggle after reset release");
            fail_count = fail_count + 1;
        end else begin
            $display("PASS: polarity toggles after reset release");
            pass_count = pass_count + 1;
        end

        repeat (4) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 2: single-hop East, non-blocking handshake */
        $display("TEST 2: single-hop East, non-blocking handshake");
        pkt = build_packet(4'd0, 4'd0, 4'd1, 4'd0, 32'h0000_1001);
        inject_packet(coord_to_idx(0,0), pkt);
        wait_e_send_and_check(coord_to_idx(0,0), 4'd0, 4'd0, 1'b0, 32'h0000_1001, 40);
        wait_delivery_and_check(coord_to_idx(1,0), 4'd1, 4'd0, 32'h0000_1001, 40);
        $fdisplay(report_fd, "TEST 2 - single-hop East, non-blocking handshake : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);
        repeat (6) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 3: single-hop West wrap-around */
        $display("TEST 3: single-hop West wrap-around");
        pkt = build_packet(4'd0, 4'd0, 4'd3, 4'd0, 32'h0000_1002);
        inject_packet(coord_to_idx(0,0), pkt);
        wait_w_send_and_check(coord_to_idx(0,0), 4'd0, 4'd0, 1'b1, 32'h0000_1002, 40);
        wait_delivery_and_check(coord_to_idx(3,0), 4'd3, 4'd0, 32'h0000_1002, 40);
        $fdisplay(report_fd, "TEST 3 - single-hop West wrap-around : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);
        repeat (6) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 4: single-hop North wrap-around */
        $display("TEST 4: single-hop North wrap-around");
        pkt = build_packet(4'd0, 4'd0, 4'd0, 4'd3, 32'h0000_1003);
        inject_packet(coord_to_idx(0,0), pkt);
        wait_n_send_and_check(coord_to_idx(0,0), 4'd0, 4'd0, 1'b1, 32'h0000_1003, 40);
        wait_delivery_and_check(coord_to_idx(0,3), 4'd0, 4'd3, 32'h0000_1003, 40);
        $fdisplay(report_fd, "TEST 4 - single-hop North wrap-around : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);
        repeat (6) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 5: multi-hop X then Y with hop update checks */
        $display("TEST 5: multi-hop X then Y with hop update checks");
        pkt = build_packet(4'd0, 4'd0, 4'd2, 4'd1, 32'h0000_1004);
        inject_packet(coord_to_idx(0,0), pkt);
        wait_e_send_and_check(coord_to_idx(0,0), 4'd1, 4'd1, 1'b0, 32'h0000_1004, 60);
        wait_e_send_and_check(coord_to_idx(1,0), 4'd0, 4'd1, 1'b0, 32'h0000_1004, 60);
        wait_s_send_and_check(coord_to_idx(2,0), 4'd0, 4'd0, 1'b0, 32'h0000_1004, 60);
        wait_delivery_and_check(coord_to_idx(2,1), 4'd2, 4'd1, 32'h0000_1004, 60);
        $fdisplay(report_fd, "TEST 5 - multi-hop X then Y with hop update checks : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);
        repeat (6) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 6: X dateline crossing followed by X1 to Y0 phase switch */
        $display("TEST 6: X dateline crossing followed by X1 to Y0 phase switch");
        pkt = build_packet(4'd3, 4'd0, 4'd1, 4'd2, 32'h0000_1005);
        inject_packet(coord_to_idx(3,0), pkt);
        wait_e_send_and_check(coord_to_idx(3,0), 4'd1, 4'd2, 1'b1, 32'h0000_1005, 100);
        wait_e_send_and_check(coord_to_idx(0,0), 4'd0, 4'd2, 1'b1, 32'h0000_1005, 100);
        wait_s_send_and_check(coord_to_idx(1,0), 4'd0, 4'd1, 1'b0, 32'h0000_1005, 100);
        wait_s_send_and_check(coord_to_idx(1,1), 4'd0, 4'd0, 1'b0, 32'h0000_1005, 100);
        wait_delivery_and_check(coord_to_idx(1,2), 4'd1, 4'd2, 32'h0000_1005, 100);
        $fdisplay(report_fd, "TEST 6 - X dateline crossing followed by X1 to Y0 phase switch : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);
        repeat (8) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 7: blocking handshake on a router-to-router link */
        $display("TEST 7: blocking handshake on a router-to-router link");
        pe_ro[coord_to_idx(1,0)] = 1'b0;
        pkt = build_packet(4'd0, 4'd0, 4'd1, 4'd0, 32'h0000_2001);
        inject_packet(coord_to_idx(0,0), pkt);
        pkt = build_packet(4'd0, 4'd0, 4'd1, 4'd0, 32'h0000_2002);
        inject_packet(coord_to_idx(0,0), pkt);
        pkt = build_packet(4'd0, 4'd0, 4'd1, 4'd0, 32'h0000_2003);
        inject_packet(coord_to_idx(0,0), pkt);
        wait_for_block_condition(200);
        pe_ro[coord_to_idx(1,0)] = 1'b1;
        wait_delivery_and_check(coord_to_idx(1,0), 4'd1, 4'd0, 32'h0000_2001, 120);
        wait_delivery_and_check(coord_to_idx(1,0), 4'd1, 4'd0, 32'h0000_2002, 120);
        wait_delivery_and_check(coord_to_idx(1,0), 4'd1, 4'd0, 32'h0000_2003, 120);
        $fdisplay(report_fd, "TEST 7 - blocking handshake on a router-to-router link : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);
        repeat (8) @(posedge clk);

        pass_before = pass_count;
        fail_before = fail_count;
        /* Test 8: deterministic contention and arbiter priority rotation at router (1,1) N_out */
        $display("TEST 8: deterministic contention and arbiter priority rotation at router (1,1) N_out");

        /* Wait until VC0 is the internal VC for deterministic observation. */
        while (dut.polarity !== 1'b0)
            @(posedge clk);
        @(negedge clk);

        /* Clear router (1,1) local state used by this test to avoid stale traffic. */
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[0] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[1] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[2] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[3] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[5] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[7] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[8] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[9] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[0] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[1] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[2] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[3] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[4] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[5] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[6] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[7] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[8] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[9] = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.n_rr_ptr_vc0 = 2'd0;

        /* Build packets as they should appear inside router (1,1):
         * one packet arrived from E_in and one from W_in, both requesting N_out.
         * Each packet has hx=0, hy=1, vc=0, and diry=N.
         */
        rx_pkt0 = build_packet(4'd0, 4'd1, 4'd1, 4'd0, 32'h0000_3001);
        rx_pkt0[63]    = 1'b0;
        rx_pkt0[60:57] = 4'd0;
        rx_pkt0[56:53] = 4'd1;

        rx_pkt1 = build_packet(4'd2, 4'd1, 4'd1, 4'd0, 32'h0000_3002);
        rx_pkt1[63]    = 1'b0;
        rx_pkt1[60:57] = 4'd0;
        rx_pkt1[56:53] = 4'd1;

        /* First contention round: preload E_in.vc0 and W_in.vc0 together. */
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[4]  = rx_pkt0; /* E_in, vc0 */
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4] = 1'b1;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[6]  = rx_pkt1; /* W_in, vc0 */
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6] = 1'b1;
        #1;
        if (!(dut.GEN_Y[1].GEN_X[1].u_router.req_n_from_e &&
              dut.GEN_Y[1].GEN_X[1].u_router.req_n_from_w &&
              dut.GEN_Y[1].GEN_X[1].u_router.n_grant_valid)) begin
            $display("FAIL: first deterministic contention round was not created at router (1,1) N_out");
            fail_count = fail_count + 1;
        end else if (dut.GEN_Y[1].GEN_X[1].u_router.n_grant_src !== PORT_E) begin
            $display("FAIL: first deterministic contention round granted wrong source, expected E_in");
            fail_count = fail_count + 1;
        end else begin
            @(posedge clk);
            #1;
            if (dut.GEN_Y[1].GEN_X[1].u_router.n_rr_ptr_vc0 !== 2'd2) begin
                $display("FAIL: N_out pointer did not rotate to expected value after first deterministic contention round");
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: first deterministic contention round granted E_in before W_in and rotated pointer");
                pass_count = pass_count + 1;
            end
        end

        /* Clear first-round residue before the second contention round. */
        @(negedge clk);
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4]  = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6]  = 1'b0;
        dut.GEN_Y[1].GEN_X[1].u_router.out_buf_valid[0] = 1'b0; /* N_out, vc0 */

        /* Second contention round: same two requesters, pointer should now favor W_in. */
        rx_pkt0 = build_packet(4'd0, 4'd1, 4'd1, 4'd0, 32'h0000_3003);
        rx_pkt0[63]    = 1'b0;
        rx_pkt0[60:57] = 4'd0;
        rx_pkt0[56:53] = 4'd1;

        rx_pkt1 = build_packet(4'd2, 4'd1, 4'd1, 4'd0, 32'h0000_3004);
        rx_pkt1[63]    = 1'b0;
        rx_pkt1[60:57] = 4'd0;
        rx_pkt1[56:53] = 4'd1;

        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[4]  = rx_pkt0;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[4] = 1'b1;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_data[6]  = rx_pkt1;
        dut.GEN_Y[1].GEN_X[1].u_router.in_buf_valid[6] = 1'b1;
        #1;
        if (!(dut.GEN_Y[1].GEN_X[1].u_router.req_n_from_e &&
              dut.GEN_Y[1].GEN_X[1].u_router.req_n_from_w &&
              dut.GEN_Y[1].GEN_X[1].u_router.n_grant_valid)) begin
            $display("FAIL: second deterministic contention round was not created at router (1,1) N_out");
            fail_count = fail_count + 1;
        end else if (dut.GEN_Y[1].GEN_X[1].u_router.n_grant_src !== PORT_W) begin
            $display("FAIL: second deterministic contention round granted wrong source, expected W_in after rotation");
            fail_count = fail_count + 1;
        end else begin
            @(posedge clk);
            #1;
            if (dut.GEN_Y[1].GEN_X[1].u_router.n_rr_ptr_vc0 !== 2'd3) begin
                $display("FAIL: N_out pointer did not rotate to expected value after second deterministic contention round");
                fail_count = fail_count + 1;
            end else begin
                $display("PASS: second deterministic contention round granted W_in before E_in and rotated pointer");
                pass_count = pass_count + 1;
            end
        end

        $fdisplay(report_fd, "TEST 8 - deterministic contention and arbiter priority rotation at router (1,1) N_out : %0s (delta pass=%0d, delta fail=%0d)", ((fail_count == fail_before) ? "PASS" : "FAIL"), pass_count-pass_before, fail_count-fail_before);
        repeat (10) @(posedge clk);

        $display("========================================");
        $display("TB DONE: pass_count=%0d fail_count=%0d", pass_count, fail_count);
        $display("========================================");
        $fdisplay(report_fd, "========================================");
        $fdisplay(report_fd, "TB DONE: pass_count=%0d fail_count=%0d", pass_count, fail_count);
        $fdisplay(report_fd, "========================================");
        $fclose(report_fd);
        $finish;
    end

endmodule
