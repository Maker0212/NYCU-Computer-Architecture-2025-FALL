//========================================================================
// Test for Mul/Div Unit
//========================================================================

`include "imuldiv-MulDivReqMsg.v"
`include "imuldiv-IntMulDivSingleCycle.v"
`include "vc-TestSource.v"
`include "vc-TestSink.v"
`include "vc-Test.v"

//------------------------------------------------------------------------
// Helper Module
//------------------------------------------------------------------------

module imuldiv_IntMulDivSingleCycle_helper
(
  input clk, reset,
  output done
);

  wire [66:0] src_msg;
  wire [ 2:0] src_msg_fn;
  wire [31:0] src_msg_a;
  wire [31:0] src_msg_b;
  wire        src_val;
  wire        src_rdy;
  wire        src_done;

  wire [31:0] sink_msg;
  wire        sink_val;
  wire        sink_rdy;
  wire        sink_done;

  assign done = src_done && sink_done;

  vc_TestSource#(67,3) src
  (
    .clk   (clk),
    .reset (reset),
    .bits  (src_msg),
    .val   (src_val),
    .rdy   (src_rdy),
    .done  (src_done)
  );

  imuldiv_MulDivReqMsgFromBits msgfrombits
  (
    .bits (src_msg),
    .func (src_msg_fn),
    .a    (src_msg_a),
    .b    (src_msg_b)
  );

  imuldiv_IntMulDivSingleCycle imuldiv
  (
    .clk                    (clk),
    .reset                  (reset),
    .muldivreq_msg_fn       (src_msg_fn),
    .muldivreq_msg_a        (src_msg_a),
    .muldivreq_msg_b        (src_msg_b),
    .muldivreq_val          (src_val),
    .muldivreq_rdy          (src_rdy),
    .muldivresp_msg_result  (sink_msg),
    .muldivresp_val         (sink_val),
    .muldivresp_rdy         (sink_rdy)
  );

  vc_TestSink#(32,3) sink
  (
    .clk   (clk),
    .reset (reset),
    .bits  (sink_msg),
    .val   (sink_val),
    .rdy   (sink_rdy),
    .done  (sink_done)
  );

endmodule

//------------------------------------------------------------------------
// Main Tester Module
//------------------------------------------------------------------------

module tester;

  // VCD Dump
  initial begin
    $dumpfile("imuldiv-IntMulDivSingleCycle.vcd");
    $dumpvars;
  end

  `VC_TEST_SUITE_BEGIN( "imuldiv-IntMulDivSingleCycle" )

  reg  t0_reset = 1'b1;
  wire t0_done;

  imuldiv_IntMulDivSingleCycle_helper t0
  (
    .clk   (clk),
    .reset (t0_reset),
    .done  (t0_done)
   );

  `VC_TEST_CASE_BEGIN( 1, "mul" )
  begin

    t0.src.m[0] = 67'h0_00000000_00000000; t0.sink.m[0] = 32'h00000000;
    t0.src.m[1] = 67'h0_00000001_00000001; t0.sink.m[1] = 32'h00000001;
    t0.src.m[2] = 67'h0_ffffffff_00000001; t0.sink.m[2] = 32'hffffffff;
    t0.src.m[3] = 67'h0_00000001_ffffffff; t0.sink.m[3] = 32'hffffffff;
    t0.src.m[4] = 67'h0_ffffffff_ffffffff; t0.sink.m[4] = 32'h00000001;
    t0.src.m[5] = 67'h0_00000008_00000003; t0.sink.m[5] = 32'h00000018;
    t0.src.m[6] = 67'h0_fffffff8_00000008; t0.sink.m[6] = 32'hffffffc0;
    t0.src.m[7] = 67'h0_fffffff8_fffffff8; t0.sink.m[7] = 32'h00000040;

    #5;   t0_reset = 1'b1;
    #20;  t0_reset = 1'b0;
    #10000; `VC_TEST_CHECK( "Is sink finished?", t0_done )

  end
  `VC_TEST_CASE_END

  `VC_TEST_SUITE_END( 1 )

endmodule
