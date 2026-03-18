module cardinal_nic(
    input  wire [1:0]  addr,
    input  wire [63:0] d_in,
    output reg  [63:0] d_out,
    input  wire        nicEn,
    input  wire        nicWrEn,

    input  wire        net_si,
    output wire        net_ri,
    input  wire [63:0] net_di,
    output wire        net_so,
    input  wire        net_ro,
    output wire [63:0] net_do,
    input  wire        net_polarity,

    input  wire        clk,
    input  wire        reset
);

reg [63:0] input_buf;
reg [63:0] output_buf;
reg        input_status;
reg        output_status;
wire vc_match = (output_buf[63] == net_polarity);

// Handshake signals
assign net_ri = ~input_status;
assign net_so = output_status & net_ro & vc_match;
assign net_do = output_buf;

// Sequential Logic
always @(posedge clk) begin
    if (reset) begin
        input_buf      <= 64'd0;
        output_buf     <= 64'd0;
        input_status   <= 1'b0;
        output_status  <= 1'b0;
        d_out          <= 64'd0;
    end else begin
        // CPU read process
        if (!nicEn) begin
            d_out <= 64'd0;
        end else if (!nicWrEn) begin
            case (addr)
                2'b00: d_out <= input_buf;                 // RX data
                2'b01: d_out <= {63'd0, input_status};     // RX status
                2'b10: d_out <= 64'd0;                     // unused
                2'b11: d_out <= {63'd0, output_status};    // TX status
                default: d_out <= 64'd0;
            endcase
        end

        // CPU write process
        if (nicEn && nicWrEn && (addr == 2'b10) && !output_status) begin
            output_buf    <= d_in;
            output_status <= 1'b1;
        end

        // Receive from Router
        if (net_si && !input_status) begin
            input_buf    <= net_di;
            input_status <= 1'b1;
        end

        // CPU consumes RX
        if (nicEn && !nicWrEn && (addr == 2'b00) && input_status) begin
            input_status <= 1'b0;
        end

        // Send completes
        if (output_status && net_ro && vc_match) begin
            output_status <= 1'b0;
        end
    end
end

endmodule