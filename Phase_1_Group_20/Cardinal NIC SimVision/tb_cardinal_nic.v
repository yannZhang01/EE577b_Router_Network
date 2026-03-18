`timescale 1ns/1ps

module tb_cardinal_nic;
reg  [1:0]  addr;
reg  [63:0] d_in;
wire [63:0] d_out;
reg         nicEn;
reg         nicWrEn;

reg         net_si;
wire        net_ri;
reg  [63:0] net_di;
wire        net_so;
reg         net_ro;
wire [63:0] net_do;
reg         net_polarity;

reg clk;
reg reset;

// DUT
cardinal_nic dut (
    .addr(addr),
    .d_in(d_in),
    .d_out(d_out),
    .nicEn(nicEn),
    .nicWrEn(nicWrEn),

    .net_si(net_si),
    .net_ri(net_ri),
    .net_di(net_di),
    .net_so(net_so),
    .net_ro(net_ro),
    .net_do(net_do),
    .net_polarity(net_polarity),

    .clk(clk),
    .reset(reset)
);

// clock
always #5 clk = ~clk;

integer logfile;

initial begin
    logfile = $fopen("sim.log", "w");
end


always @(posedge clk) begin
    if (logfile != 0) begin  
        $fdisplay(logfile,
        "T=%0t | rst=%b | nicEn=%b wr=%b addr=%b | out_stat=%b in_stat=%b | net_so=%b net_ro=%b net_ri=%b | VC=%b pol=%b",
        $time,
        reset,
        nicEn,
        nicWrEn,
        addr,
        dut.output_status,
        dut.input_status,
        net_so,
        net_ro,
        net_ri,
        dut.output_buf[63],
        net_polarity
    );
    end
end

initial begin
    clk = 0;
    reset = 1;
    nicEn = 0;
    nicWrEn = 0;
    addr = 0;
    d_in = 0;
    net_si = 0;
    net_di = 0;
    net_ro = 0;
    net_polarity = 0;

    // 2.2 Reset waveform
    $display("==== 2.2 RESET TEST ====");
    #20 reset = 0;
    #20;

    // 2.3 Handshake WITHOUT blocking
    $display("==== 2.3.2 WITHOUT BLOCKING ====");

    net_polarity = 0;
    net_ro = 1;

    nicEn = 1;
    nicWrEn = 1;
    addr = 2'b10;
    d_in = 64'h0000_0000_0000_0001;
    #10;
    nicWrEn = 0;

    #30;

    // 2.3 Handshake WITH blocking (backpressure)
    $display("==== 2.3.1 WITH BLOCKING (net_ro=0) ====");

    reset = 1; #10; reset = 0;

    net_polarity = 0;

    nicWrEn = 1; addr = 2'b10;
    d_in = 64'h0000_0000_0000_0002;
    #10; nicWrEn = 0;

    net_ro = 0;   // block
    #40;

    net_ro = 1;   // release
    #30;

    // 2.3 WITH blocking (VC mismatch)
    $display("==== 2.3.1 WITH BLOCKING (VC mismatch) ====");

    reset = 1; #10; reset = 0;

    net_polarity = 0;
    net_ro = 1;

    nicWrEn = 1; addr = 2'b10;
    d_in = 64'h8000_0000_0000_0003; // VC mismatch
    #10; nicWrEn = 0;

    #50;

    // 2.4.1 Load (RX) buffer available
    $display("==== 2.4.1 LOAD AVAILABLE ====");

    reset = 1; #10; reset = 0;

    net_si = 1;
    net_di = 64'hAAAA_BBBB_CCCC_DDDD;
    #10;
    net_si = 0;

    #10;

    nicEn = 1;
    nicWrEn = 0;
    addr = 2'b00; // read RX
    #20;

    // 2.4.2 Load buffer unavailable
    //$display("==== 2.4.2 LOAD UNAVAILABLE ====");

    //reset = 1; #10; reset = 0;

    // fill buffer
    //net_si = 1;
    //net_di = 64'h1111_2222_3333_4444;
    //#10; net_si = 0;

    //#10;

    // try second packet (should be blocked)
    //net_si = 1;
    //net_di = 64'h5555_6666_7777_8888;
    //#10; net_si = 0;

    //#30;

    // 2.4.3 Store available
    $display("==== 2.4.3 STORE AVAILABLE ====");

    reset = 1; #10; reset = 0;

    net_ro = 1;
    net_polarity = 0;

    nicWrEn = 1;
    addr = 2'b10;
    d_in = 64'h0000_0000_0000_00AA;
    #10;
    nicWrEn = 0;

    #30;

    // 2.4.4 Store unavailable
    $display("==== 2.4.4 STORE UNAVAILABLE ====");

    reset = 1; #10; reset = 0;

    net_ro = 0; // block send
    net_polarity = 0;

    nicWrEn = 1;
    addr = 2'b10;
    d_in = 64'h0000_0000_0000_00BB;
    #10;
    nicWrEn = 0;

    #40;

    // try overwrite (should NOT overwrite)
    nicWrEn = 1;
    d_in = 64'h0000_0000_0000_00CC;
    #10;
    nicWrEn = 0;

    #50;

    $display("==== NEW: ROUTER WRITE SUCCESS (NIC accepts) ====");

    reset = 1; #10; reset = 0;

    // NIC buffer empty → ready

    net_si = 1;
    net_di = 64'hDEAD_BEEF_0000_0001;
    #10;
    net_si = 0;

    #20;
    $display("==== NEW: ROUTER WRITE FAIL (NIC not ready) ====");

    reset = 1; #10; reset = 0;

    // Step 1: fill buffer
    net_si = 1;
    net_di = 64'hAAAA_AAAA_AAAA_AAAA;
    #10;
    net_si = 0;

    #10;

    // Step 2: try second write (should FAIL)
    net_si = 1;
    net_di = 64'hBBBB_BBBB_BBBB_BBBB;
    #10;
    net_si = 0;
    #30;


    // 2.4.2 Load buffer unavailable (new VERSION)
    $display("==== 2.4.2 LOAD UNAVAILABLE (BUFFER EMPTY) ====");

    reset = 1; #10; reset = 0;


    #10;

    // CPU try to load
    nicEn = 1;
    nicWrEn = 0;      // read
    addr = 2'b00;     // RX buffer
    #20;

    nicEn = 0;

    #20;

   $display("==== ALL TESTS DONE ====");
   $fclose(logfile);  
   $finish;
end

initial begin
    $timeformat(-9, 0, " ns", 0);
end



endmodule


