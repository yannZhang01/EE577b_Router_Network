`timescale 1ns / 1ps

module tb_gold_router;

    localparam PACKET_WIDTH = 64;

    localparam PORT_N  = 0;
    localparam PORT_S  = 1;
    localparam PORT_E  = 2;
    localparam PORT_W  = 3;
    localparam PORT_PE = 4;

    localparam DIRX_E = 1'b0;
    localparam DIRX_W = 1'b1;
    localparam DIRY_S = 1'b0;
    localparam DIRY_N = 1'b1;

    reg                    clk;
    reg                    reset;
    reg                    polarity;
    reg  [3:0]             local_x;
    reg  [3:0]             local_y;

    reg                    n_si;
    wire                   n_ri;
    reg  [PACKET_WIDTH-1:0] n_di;
    wire                   n_so;
    reg                    n_ro;
    wire [PACKET_WIDTH-1:0] n_do;

    reg                    s_si;
    wire                   s_ri;
    reg  [PACKET_WIDTH-1:0] s_di;
    wire                   s_so;
    reg                    s_ro;
    wire [PACKET_WIDTH-1:0] s_do;

    reg                    e_si;
    wire                   e_ri;
    reg  [PACKET_WIDTH-1:0] e_di;
    wire                   e_so;
    reg                    e_ro;
    wire [PACKET_WIDTH-1:0] e_do;

    reg                    w_si;
    wire                   w_ri;
    reg  [PACKET_WIDTH-1:0] w_di;
    wire                   w_so;
    reg                    w_ro;
    wire [PACKET_WIDTH-1:0] w_do;

    reg                    pe_si;
    wire                   pe_ri;
    reg  [PACKET_WIDTH-1:0] pe_di;
    wire                   pe_so;
    reg                    pe_ro;
    wire [PACKET_WIDTH-1:0] pe_do;

    integer pass_count;
    integer fail_count;

    gold_router #(
        .PACKET_WIDTH(PACKET_WIDTH)
    ) dut (
        .clk(clk),
        .reset(reset),
        .polarity(polarity),
        .local_x(local_x),
        .local_y(local_y),
        .n_si(n_si),
        .n_ri(n_ri),
        .n_di(n_di),
        .n_so(n_so),
        .n_ro(n_ro),
        .n_do(n_do),
        .s_si(s_si),
        .s_ri(s_ri),
        .s_di(s_di),
        .s_so(s_so),
        .s_ro(s_ro),
        .s_do(s_do),
        .e_si(e_si),
        .e_ri(e_ri),
        .e_di(e_di),
        .e_so(e_so),
        .e_ro(e_ro),
        .e_do(e_do),
        .w_si(w_si),
        .w_ri(w_ri),
        .w_di(w_di),
        .w_so(w_so),
        .w_ro(w_ro),
        .w_do(w_do),
        .pe_si(pe_si),
        .pe_ri(pe_ri),
        .pe_di(pe_di),
        .pe_so(pe_so),
        .pe_ro(pe_ro),
        .pe_do(pe_do)
    );



    function integer buf_idx;
        input integer port;
        input integer vc;
        begin
            buf_idx = (port << 1) + vc;
        end
    endfunction

    function [PACKET_WIDTH-1:0] make_packet;
        input                    vc;
        input                    dirx;
        input                    diry;
        input [3:0]              hops_x;
        input [3:0]              hops_y;
        input [3:0]              source_x;
        input [3:0]              source_y;
        input [3:0]              dest_x;
        input [3:0]              dest_y;
        input [31:0]             payload;
        begin
            make_packet = {
                vc,
                dirx,
                diry,
                hops_x,
                hops_y,
                source_x,
                source_y,
                dest_x,
                dest_y,
                5'b00000,
                payload
            };
        end
    endfunction

    function [PACKET_WIDTH-1:0] step_x_packet;
        input [PACKET_WIDTH-1:0] pkt;
        input                    next_vc;
        begin
            step_x_packet = pkt;
            step_x_packet[63]    = next_vc;
            step_x_packet[60:57] = pkt[60:57] - 4'd1;
        end
    endfunction

    function [PACKET_WIDTH-1:0] step_y_packet;
        input [PACKET_WIDTH-1:0] pkt;
        input                    next_vc;
        begin
            step_y_packet = pkt;
            step_y_packet[63]    = next_vc;
            step_y_packet[56:53] = pkt[56:53] - 4'd1;
        end
    endfunction

    task clear_inputs;
        begin
            n_si  = 1'b0;
            s_si  = 1'b0;
            e_si  = 1'b0;
            w_si  = 1'b0;
            pe_si = 1'b0;

            n_di  = {PACKET_WIDTH{1'b0}};
            s_di  = {PACKET_WIDTH{1'b0}};
            e_di  = {PACKET_WIDTH{1'b0}};
            w_di  = {PACKET_WIDTH{1'b0}};
            pe_di = {PACKET_WIDTH{1'b0}};
        end
    endtask

    task clear_outputs_ready;
        begin
            n_ro  = 1'b1;
            s_ro  = 1'b1;
            e_ro  = 1'b1;
            w_ro  = 1'b1;
            pe_ro = 1'b1;
        end
    endtask

    task do_reset;
        begin
            reset    = 1'b1;
            polarity = 1'b0;
            clear_inputs;
            clear_outputs_ready;
            local_x  = 4'd0;
            local_y  = 4'd0;
            repeat (4) @(posedge clk);
            @(negedge clk);
            reset = 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    task wait_cycles;
        input integer cycles;
        integer k;
        begin
            for (k = 0; k < cycles; k = k + 1)
                @(posedge clk);
        end
    endtask

    task inject_n;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            @(negedge clk);
            while (n_ri !== 1'b1)
                @(negedge clk);
            n_si <= 1'b1;
            n_di <= pkt;
            @(posedge clk);
            @(negedge clk);
            n_si <= 1'b0;
            n_di <= {PACKET_WIDTH{1'b0}};
        end
    endtask

    task inject_s;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            @(negedge clk);
            while (s_ri !== 1'b1)
                @(negedge clk);
            s_si <= 1'b1;
            s_di <= pkt;
            @(posedge clk);
            @(negedge clk);
            s_si <= 1'b0;
            s_di <= {PACKET_WIDTH{1'b0}};
        end
    endtask

    task inject_e;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            @(negedge clk);
            while (e_ri !== 1'b1)
                @(negedge clk);
            e_si <= 1'b1;
            e_di <= pkt;
            @(posedge clk);
            @(negedge clk);
            e_si <= 1'b0;
            e_di <= {PACKET_WIDTH{1'b0}};
        end
    endtask

    task inject_w;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            @(negedge clk);
            while (w_ri !== 1'b1)
                @(negedge clk);
            w_si <= 1'b1;
            w_di <= pkt;
            @(posedge clk);
            @(negedge clk);
            w_si <= 1'b0;
            w_di <= {PACKET_WIDTH{1'b0}};
        end
    endtask

    task inject_pe;
        input [PACKET_WIDTH-1:0] pkt;
        begin
            @(negedge clk);
            while (pe_ri !== 1'b1)
                @(negedge clk);
            pe_si <= 1'b1;
            pe_di <= pkt;
            @(posedge clk);
            @(negedge clk);
            pe_si <= 1'b0;
            pe_di <= {PACKET_WIDTH{1'b0}};
        end
    endtask

    task inject_n_and_e_same_cycle;
        input [PACKET_WIDTH-1:0] pkt_n;
        input [PACKET_WIDTH-1:0] pkt_e;
        begin
            @(negedge clk);
            while ((n_ri !== 1'b1) || (e_ri !== 1'b1))
                @(negedge clk);
            n_si <= 1'b1;
            n_di <= pkt_n;
            e_si <= 1'b1;
            e_di <= pkt_e;
            @(posedge clk);
            @(negedge clk);
            n_si <= 1'b0;
            e_si <= 1'b0;
            n_di <= {PACKET_WIDTH{1'b0}};
            e_di <= {PACKET_WIDTH{1'b0}};
        end
    endtask

    task check_no_send;
        input integer port;
        input integer cycles;
        integer c;
        begin
            for (c = 0; c < cycles; c = c + 1) begin
                @(posedge clk);
                case (port)
                    PORT_N:  if (n_so) begin
                                 $display("[FAIL] Unexpected N send at time %0t", $time);
                                 fail_count = fail_count + 1;
                             end
                    PORT_S:  if (s_so) begin
                                 $display("[FAIL] Unexpected S send at time %0t", $time);
                                 fail_count = fail_count + 1;
                             end
                    PORT_E:  if (e_so) begin
                                 $display("[FAIL] Unexpected E send at time %0t", $time);
                                 fail_count = fail_count + 1;
                             end
                    PORT_W:  if (w_so) begin
                                 $display("[FAIL] Unexpected W send at time %0t", $time);
                                 fail_count = fail_count + 1;
                             end
                    PORT_PE: if (pe_so) begin
                                 $display("[FAIL] Unexpected PE send at time %0t", $time);
                                 fail_count = fail_count + 1;
                             end
                endcase
            end
        end
    endtask

    task wait_for_send_and_check;
        input integer            port;
        input [PACKET_WIDTH-1:0] expected_pkt;
        input integer            timeout_cycles;
        integer count;
        reg     found;
        reg [PACKET_WIDTH-1:0] observed_pkt;
        begin
            found = 1'b0;
            observed_pkt = {PACKET_WIDTH{1'b0}};
            count = 0;
            while ((count < timeout_cycles) && !found) begin
                @(posedge clk);
                case (port)
                    PORT_N: begin
                        if (n_so && n_ro) begin
                            found = 1'b1;
                            observed_pkt = n_do;
                        end
                    end
                    PORT_S: begin
                        if (s_so && s_ro) begin
                            found = 1'b1;
                            observed_pkt = s_do;
                        end
                    end
                    PORT_E: begin
                        if (e_so && e_ro) begin
                            found = 1'b1;
                            observed_pkt = e_do;
                        end
                    end
                    PORT_W: begin
                        if (w_so && w_ro) begin
                            found = 1'b1;
                            observed_pkt = w_do;
                        end
                    end
                    PORT_PE: begin
                        if (pe_so && pe_ro) begin
                            found = 1'b1;
                            observed_pkt = pe_do;
                        end
                    end
                endcase
                count = count + 1;
            end

            if (!found) begin
                $display("[FAIL] Timed out waiting for send on port %0d at time %0t", port, $time);
                fail_count = fail_count + 1;
            end else if (observed_pkt !== expected_pkt) begin
                $display("[FAIL] Packet mismatch on port %0d at time %0t", port, $time);
                $display("       expected = 0x%016h", expected_pkt);
                $display("       observed = 0x%016h", observed_pkt);
                fail_count = fail_count + 1;
            end else begin
                $display("[PASS] Port %0d transmitted expected packet 0x%016h at time %0t", port, observed_pkt, $time);
                pass_count = pass_count + 1;
            end
        end
    endtask

    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (reset)
            polarity <= 1'b0;
        else
            polarity <= ~polarity;
    end

    initial begin
        clk = 1'b0;
        pass_count = 0;
        fail_count = 0;

        $dumpfile("gold_router_tb.vcd");
        $dumpvars(0, tb_gold_router);

        do_reset();

        // ------------------------------------------------------------
        // Test 1: PE -> E, single X hop, no dateline crossing
        // ------------------------------------------------------------
        $display("\n[Test 1] PE -> E, single X hop, no X dateline crossing");
        local_x = 4'd1;
        local_y = 4'd1;
        inject_pe(make_packet(1'b0, DIRX_E, DIRY_S, 4'd1, 4'd0, 4'd1, 4'd1, 4'd2, 4'd1, 32'h11111111));
        wait_for_send_and_check(
            PORT_E,
            make_packet(1'b0, DIRX_E, DIRY_S, 4'd0, 4'd0, 4'd1, 4'd1, 4'd2, 4'd1, 32'h11111111),
            20
        );
        wait_cycles(4);

        // ------------------------------------------------------------
        // Test 2: PE -> E, X dateline crossing should raise VC to 1
        // ------------------------------------------------------------
        $display("\n[Test 2] PE -> E, X dateline crossing updates VC to 1");
        local_x = 4'd3;
        local_y = 4'd1;
        inject_pe(make_packet(1'b0, DIRX_E, DIRY_S, 4'd1, 4'd0, 4'd3, 4'd1, 4'd0, 4'd1, 32'h22222222));
        wait_for_send_and_check(
            PORT_E,
            make_packet(1'b1, DIRX_E, DIRY_S, 4'd0, 4'd0, 4'd3, 4'd1, 4'd0, 4'd1, 32'h22222222),
            20
        );
        wait_cycles(4);

        // ------------------------------------------------------------
        // Test 3: E_in -> S_out, X phase complete, first Y hop does not
        //         cross Y dateline, so VC must become 0 (X1 -> Y0)
        // ------------------------------------------------------------
        $display("\n[Test 3] E_in -> S_out, first Y hop non-cross forces VC to 0");
        local_x = 4'd2;
        local_y = 4'd1;
        inject_e(make_packet(1'b1, DIRX_E, DIRY_S, 4'd0, 4'd2, 4'd3, 4'd1, 4'd3, 4'd3, 32'h33333333));
        wait_for_send_and_check(
            PORT_S,
            make_packet(1'b0, DIRX_E, DIRY_S, 4'd0, 4'd1, 4'd3, 4'd1, 4'd3, 4'd3, 32'h33333333),
            20
        );
        wait_cycles(4);

        // ------------------------------------------------------------
        // Test 4: N_in -> N_out, Y dateline crossing updates VC to 1
        // ------------------------------------------------------------
        $display("\n[Test 4] N_in -> N_out, Y dateline crossing updates VC to 1");
        local_x = 4'd0;
        local_y = 4'd0;
        inject_n(make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd2, 4'd0, 4'd0, 4'd0, 4'd2, 32'h44444444));
        wait_for_send_and_check(
            PORT_N,
            make_packet(1'b1, DIRX_E, DIRY_N, 4'd0, 4'd1, 4'd0, 4'd0, 4'd0, 4'd2, 32'h44444444),
            20
        );
        wait_cycles(4);

        // ------------------------------------------------------------
        // Test 5: N_in -> PE_out, eject when hops_x == 0 and hops_y == 0
        // ------------------------------------------------------------
        $display("\n[Test 5] N_in -> PE_out eject");
        local_x = 4'd2;
        local_y = 4'd2;
        inject_n(make_packet(1'b0, DIRX_E, DIRY_S, 4'd0, 4'd0, 4'd1, 4'd1, 4'd2, 4'd2, 32'h55555555));
        wait_for_send_and_check(
            PORT_PE,
            make_packet(1'b0, DIRX_E, DIRY_S, 4'd0, 4'd0, 4'd1, 4'd1, 4'd2, 4'd2, 32'h55555555),
            20
        );
        wait_cycles(4);

        // ------------------------------------------------------------
        // Test 6: E_out backpressure. Packet must stay buffered while
        //         e_ro == 0 and leave only after e_ro becomes 1.
        // ------------------------------------------------------------
        $display("\n[Test 6] E_out backpressure blocks send until ready returns");
        local_x = 4'd1;
        local_y = 4'd1;
        e_ro = 1'b0;
        inject_pe(make_packet(1'b0, DIRX_E, DIRY_S, 4'd1, 4'd0, 4'd1, 4'd1, 4'd2, 4'd1, 32'h66666666));
        check_no_send(PORT_E, 6);
        e_ro = 1'b1;
        wait_for_send_and_check(
            PORT_E,
            make_packet(1'b0, DIRX_E, DIRY_S, 4'd0, 4'd0, 4'd1, 4'd1, 4'd2, 4'd1, 32'h66666666),
            20
        );
        wait_cycles(4);

        // ------------------------------------------------------------
        // Test 7: N_out arbitration with round-robin behavior.
        //         Round 1: N_in and E_in contend, N_in should win first.
        //         Round 2: inject a new N_in packet while E_in loser stays,
        //                  then E_in should win because pointer advanced.
        //         Round 3: remaining N_in packet leaves.
        // ------------------------------------------------------------
        $display("\n[Test 7] N_out arbitration and round-robin priority");
        local_x = 4'd1;
        local_y = 4'd2;

        inject_n_and_e_same_cycle(
            make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd1, 4'd0, 4'd2, 4'd0, 4'd1, 32'h77770001),
            make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd1, 4'd1, 4'd2, 4'd1, 4'd1, 32'h77770002)
        );

        wait_for_send_and_check(
            PORT_N,
            make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd0, 4'd0, 4'd2, 4'd0, 4'd1, 32'h77770001),
            20
        );

        inject_n(make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd1, 4'd2, 4'd2, 4'd2, 4'd1, 32'h77770003));

        wait_for_send_and_check(
            PORT_N,
            make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd0, 4'd1, 4'd2, 4'd1, 4'd1, 32'h77770002),
            20
        );

        wait_for_send_and_check(
            PORT_N,
            make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd0, 4'd2, 4'd2, 4'd2, 4'd1, 32'h77770003),
            20
        );
        wait_cycles(6);

        // ------------------------------------------------------------
        // Test 8: Deterministic N_out contention and round-robin
        //         rotation inside a single router.
        // ------------------------------------------------------------
        $display("\n[Test 8] Dedicated contention and arbiter rotation on N_out");
        do_reset();
        local_x = 4'd1;
        local_y = 4'd2;

        // Ensure VC0 is the active internal VC for deterministic checking.
        if (polarity !== 1'b0)
            wait (polarity == 1'b0);

        // Round 1: preload E_in.vc0 and W_in.vc0 with two packets that both
        // request N_out. Reset priority order for N_out VC0 is
        // N_in -> E_in -> W_in -> PE_in, so E_in must win first.
        @(negedge clk);
        dut.in_buf_valid[buf_idx(PORT_E, 1'b0)]  = 1'b1;
        dut.in_buf_data [buf_idx(PORT_E, 1'b0)]  = make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd1, 4'd4, 4'd2, 4'd4, 4'd1, 32'h88880001);
        dut.in_buf_valid[buf_idx(PORT_W, 1'b0)]  = 1'b1;
        dut.in_buf_data [buf_idx(PORT_W, 1'b0)]  = make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd1, 4'd5, 4'd2, 4'd5, 4'd1, 32'h88880002);
        dut.out_buf_valid[buf_idx(PORT_N, 1'b0)] = 1'b0;
        #1;

        if (!(dut.req_n_from_e && dut.req_n_from_w && dut.n_grant_valid)) begin
            $display("[FAIL] Round 1 contention was not created on N_out at time %0t", $time);
            fail_count = fail_count + 1;
        end else if (dut.n_grant_src !== PORT_E) begin
            $display("[FAIL] Round 1 wrong winner on N_out. Expected E_in, observed src=%0d", dut.n_grant_src);
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] Round 1 contention observed on N_out and E_in won first");
            pass_count = pass_count + 1;
        end

        @(posedge clk);
        #1;
        if (dut.n_rr_ptr_vc0 !== 2'd2) begin
            $display("[FAIL] Round 1 pointer did not rotate to W_in. Expected 2, observed %0d", dut.n_rr_ptr_vc0);
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] Round 1 pointer rotated to W_in (n_rr_ptr_vc0 = %0d)", dut.n_rr_ptr_vc0);
            pass_count = pass_count + 1;
        end

        // Round 2: keep the W_in loser from Round 1, clear N_out.vc0 so it can
        // accept a new winner, and inject a fresh E_in contender. After
        // rotation, W_in must win.
        if (polarity !== 1'b0)
            wait (polarity == 1'b0);
        @(negedge clk);
        dut.out_buf_valid[buf_idx(PORT_N, 1'b0)] = 1'b0;
        dut.in_buf_valid[buf_idx(PORT_E, 1'b0)]  = 1'b1;
        dut.in_buf_data [buf_idx(PORT_E, 1'b0)]  = make_packet(1'b0, DIRX_E, DIRY_N, 4'd0, 4'd1, 4'd6, 4'd2, 4'd6, 4'd1, 32'h88880003);
        #1;

        if (!(dut.req_n_from_e && dut.req_n_from_w && dut.n_grant_valid)) begin
            $display("[FAIL] Round 2 contention was not created on N_out at time %0t", $time);
            fail_count = fail_count + 1;
        end else if (dut.n_grant_src !== PORT_W) begin
            $display("[FAIL] Round 2 wrong winner on N_out. Expected W_in after rotation, observed src=%0d", dut.n_grant_src);
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] Round 2 contention observed on N_out and W_in won after rotation");
            pass_count = pass_count + 1;
        end

        @(posedge clk);
        #1;
        if (dut.n_rr_ptr_vc0 !== 2'd3) begin
            $display("[FAIL] Round 2 pointer did not rotate after W_in grant. Expected 3, observed %0d", dut.n_rr_ptr_vc0);
            fail_count = fail_count + 1;
        end else begin
            $display("[PASS] Round 2 pointer rotated after W_in grant (n_rr_ptr_vc0 = %0d)", dut.n_rr_ptr_vc0);
            pass_count = pass_count + 1;
        end
        wait_cycles(4);


        $display("\n============================================================");
        $display("Testbench finished. PASS = %0d, FAIL = %0d", pass_count, fail_count);
        $display("============================================================\n");

        if (fail_count == 0)
            $display("All directed tests passed.");
        else
            $display("Some directed tests failed. Please inspect the waveform/log.");

        $finish;
    end

endmodule
