//========================================================================
// Simple Test for Real 1-Cycle Latency 2-Port Memory
//========================================================================
// Use vcd dump for detailed information.

`include "vc-RealMemories.v"

module memtest;

  reg  [40:0] srcm0[25:0];
  reg  [43:0] srcm1[25:0];

  reg         clk = 1'b0;
  reg         reset_int = 1'b1;

  always #5 clk = ~clk;

  reg  [5:0]  idx;
  reg         waitbit;

  reg  [40:0] src0_bits;
  reg         src0_val;
  wire        src0_bits_rw   = src0_bits[40];
  wire [ 7:0] src0_bits_addr = src0_bits[39:32];
  wire [31:0] src0_bits_data = src0_bits[31:0];

  reg  [43:0] src1_bits;
  reg         src1_val;
  wire        src1_sub_en    = src1_bits[43];
  wire        src1_sub_sign  = src1_bits[42];
  wire        src1_sub_type  = src1_bits[41];
  wire        src1_bits_rw   = src1_bits[40];
  wire [ 7:0] src1_bits_addr = src1_bits[39:32];
  wire [31:0] src1_bits_data = src1_bits[31:0];

  wire [31:0] sink0_data;
  wire [31:0] sink1_data;

  vc_Mem_randsub2port#(16,8,32,2) mem
  (
    .clk               (clk),
    .reset             (reset_int),

    .memreq0_bits_rw    (src0_bits_rw),
    .memreq0_bits_addr  (src0_bits_addr),
    .memreq0_bits_data  (src0_bits_data),
    .memreq0_val        (src0_val),
    .memreq0_rdy        (src0_rdy),
    .memresp0_bits_data (sink0_data),
    .memresp0_val       (),

    .memreq1_bits_rw    (src1_bits_rw),
    .memreq1_bits_addr  (src1_bits_addr),
    .memreq1_bits_data  (src1_bits_data),
    .subword_en         (src1_sub_en),
    .subword_sign       (src1_sub_sign),
    .subword_type       (src1_sub_type),
    .memreq1_val        (src1_val),
    .memreq1_rdy        (src1_rdy),
    .memresp1_bits_data (sink1_data),
    .memresp1_val       ()
  );

  initial begin
    reset_int <= 1'b1;
    idx <= 5'b0;
    waitbit <= 1'b0;

    srcm0[ 0] <= 41'h1_00_aaaaaaaa;
    srcm0[ 1] <= 41'h0_00_xxxxxxxx;
    srcm0[ 2] <= 41'h1_04_bbbbbbbb;
    srcm0[ 3] <= 41'h1_08_cccccccc;
    srcm0[ 4] <= 41'h0_04_xxxxxxxx;
    srcm0[ 5] <= 41'h0_08_xxxxxxxx;
    srcm0[ 6] <= 41'h1_00_dddddddd;
    srcm0[ 7] <= 41'h0_00_xxxxxxxx;
    srcm0[ 8] <= 41'h1_04_eeeeeeee;
    srcm0[ 9] <= 41'h1_08_ffffffff;
    srcm0[10] <= 41'h0_04_xxxxxxxx;
    srcm0[11] <= 41'h0_08_xxxxxxxx;
    srcm0[12] <= 41'h0_08_xxxxxxxx;
    srcm0[13] <= 41'h0_08_xxxxxxxx;
    srcm0[14] <= 41'h0_08_xxxxxxxx;
    srcm0[15] <= 41'h0_08_xxxxxxxx;
    srcm0[16] <= 41'h0_08_xxxxxxxx;
    srcm0[17] <= 41'h0_04_xxxxxxxx;
    srcm0[18] <= 41'h0_08_xxxxxxxx;
    srcm0[19] <= 41'h0_08_xxxxxxxx;
    srcm0[20] <= 41'h0_08_xxxxxxxx;
    srcm0[21] <= 41'h0_08_xxxxxxxx;
    srcm0[22] <= 41'h0_08_xxxxxxxx;
    srcm0[23] <= 41'h0_08_xxxxxxxx;
    srcm0[24] <= 41'h0_08_xxxxxxxx;
    srcm0[25] <= 41'h0_08_xxxxxxxx;

    // init
    srcm1[ 0] <= {4'b0001, 40'h0c_deadbeef};
    srcm1[ 1] <= {4'b0000, 40'h0c_xxxxxxxx};

    // lbu
    srcm1[ 2] <= {4'b1_00_0, 40'h0c_xxxxxxxx};
    srcm1[ 3] <= {4'b1_00_0, 40'h0d_xxxxxxxx};
    srcm1[ 4] <= {4'b1_00_0, 40'h0e_xxxxxxxx};
    srcm1[ 5] <= {4'b1_00_0, 40'h0f_xxxxxxxx};

    // lb
    srcm1[ 6] <= {4'b1_10_0, 40'h0c_xxxxxxxx};
    srcm1[ 7] <= {4'b1_10_0, 40'h0d_xxxxxxxx};
    srcm1[ 8] <= {4'b1_10_0, 40'h0e_xxxxxxxx};
    srcm1[ 9] <= {4'b1_10_0, 40'h0f_xxxxxxxx};

    // lhu
    srcm1[10] <= {4'b1_01_0, 40'h0c_xxxxxxxx};
    srcm1[11] <= {4'b1_01_0, 40'h0e_xxxxxxxx};

    // lh
    srcm1[12] <= {4'b1_11_0, 40'h0c_xxxxxxxx};
    srcm1[13] <= {4'b1_11_0, 40'h0e_xxxxxxxx};

    // sb
    srcm1[14] <= {4'b1_10_1, 40'h0c_00000077};
    srcm1[15] <= {4'b0_xx_0, 40'h0c_xxxxxxxx};
    srcm1[16] <= {4'b1_10_1, 40'h0d_00000077};
    srcm1[17] <= {4'b0_xx_0, 40'h0c_xxxxxxxx};
    srcm1[18] <= {4'b1_10_1, 40'h0e_00000077};
    srcm1[19] <= {4'b0_xx_0, 40'h0c_xxxxxxxx};
    srcm1[20] <= {4'b1_10_1, 40'h0f_00000077};
    srcm1[21] <= {4'b0_xx_0, 40'h0c_xxxxxxxx};

    // sh
    srcm1[22] <= {4'b1_11_1, 40'h0c_0000beef};
    srcm1[23] <= {4'b0_xx_0, 40'h0c_xxxxxxxx};
    srcm1[24] <= {4'b1_11_1, 40'h0e_0000dead};
    srcm1[25] <= {4'b0_xx_0, 40'h0c_xxxxxxxx};

    $dumpfile("dump.vcd");
    $dumpvars;

    #10 reset_int <= 1'b0;
  end

always @ (posedge clk) begin
  if( !reset_int ) begin
    if( idx == 5'd30 ) begin
      $finish;
    end
    else if( waitbit ) begin
      src0_bits <= srcm0[idx];
      src1_bits <= srcm1[idx];

      src0_val <= 1'b1;
      src1_val <= 1'b1;

      idx <= idx + 1;
    end
    waitbit <= waitbit + 1;
  end
end

always @ (posedge clk) begin
  $display("reset = %d", reset_int);
  $display("idx = %d, waitbit = %d", idx, waitbit);
  $display("sub_en = %d", src1_bits[43]);
  $display("sub_sign = %d", src1_bits[42]);
  $display("sub_type = %d", src1_bits[41]);
  $display("src1_rw = %d", src1_bits[40]);
  $display("src1_addr = %h", src1_bits[39:32]);
  $display("src0_bits = %h, src1_bits = %h", src0_bits, src1_bits[31:0]);
  $display("sink0_data = %h, sink1_data = %h", sink0_data, sink1_data);
  $display("====================================");
end

endmodule

