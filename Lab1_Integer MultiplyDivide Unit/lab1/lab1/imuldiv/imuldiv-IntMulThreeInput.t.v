//========================================================================
// Test for Three Input Mul Unit
//========================================================================

`include "imuldiv-ThreeMulReqMsg.v"
`include "imuldiv-ThreeMulRespMsg.v"
`include "imuldiv-IntMulThreeInput.v"
`include "vc-TestSource.v"
`include "vc-TestSink.v"
`include "vc-Test.v"


//------------------------------------------------------------------------
// Helper Module
//------------------------------------------------------------------------

module imuldiv_IntMulThreeInput_helper
(
  input       clk,
  input       reset,
  output      done
);

  wire [`IMULDIV_THREE_MULDIVREQ_MSG_SZ-1:0] src_msg;
  wire [2:0] src_msg_fn;
  wire [`IMULDIV_THREE_MULDIVREQ_MSG_A_SZ-1:0] src_msg_a;
  wire [`IMULDIV_THREE_MULDIVREQ_MSG_B_SZ-1:0] src_msg_b;
  wire [`IMULDIV_THREE_MULDIVREQ_MSG_C_SZ-1:0] src_msg_c;
  wire        src_val;
  wire        src_rdy;
  wire        src_done;

  wire [`IMULDIV_THREE_MULDIVRESP_MSG_SZ-1:0] sink_msg;
  wire        sink_val;
  wire        sink_rdy;
  wire        sink_done;

  assign done = src_done && sink_done;

  vc_TestSource#(`IMULDIV_THREE_MULDIVREQ_MSG_SZ,3) src
  (
    .clk   (clk),
    .reset (reset),
    .bits  (src_msg),
    .val   (src_val),
    .rdy   (src_rdy),
    .done  (src_done)
  );

  imuldiv_ThreeMulReqMsgFromBits msgfrombits
  (
    .bits (src_msg),
    .func (src_msg_fn),
    .a    (src_msg_a),
    .b    (src_msg_b),
    .c    (src_msg_c)
  );

  imuldiv_IntMulThreeInput imuldiv
  (
    .clk                (clk),
    .reset              (reset),
    .muldivreq_msg_fn      (src_msg_fn),
    .muldivreq_msg_a       (src_msg_a),
    .muldivreq_msg_b       (src_msg_b),
    .muldivreq_msg_c       (src_msg_c),
    .muldivreq_val         (src_val),
    .muldivreq_rdy         (src_rdy),
    .muldivresp_msg_result (sink_msg),
    .muldivresp_val        (sink_val),
    .muldivresp_rdy        (sink_rdy)
  );

  vc_TestSink#(`IMULDIV_THREE_MULDIVRESP_MSG_SZ,3) sink
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
    $dumpfile("imuldiv-IntMulThreeInput.vcd");
    $dumpvars;
  end

  `VC_TEST_SUITE_BEGIN( "imuldiv-IntMulThreeInput" )

  reg  t0_reset = 1'b1;
  wire t0_done;

  imuldiv_IntMulThreeInput_helper t0
  (
    .clk   (clk),
    .reset (t0_reset),
    .done  (t0_done)
  );

  `VC_TEST_CASE_BEGIN( 1, "mul" )
  `VC_TEST_CASE_INIT(`IMULDIV_THREE_MULDIVREQ_MSG_SZ, `IMULDIV_THREE_MULDIVRESP_MSG_SZ)
  begin

    t0.src.m[0] = 99'h0_00000001_00000001_00000005;t0.sink.m[0] = 96'h000000000000000000000005;
    t0.src.m[1] = 99'h0_00000001_00000001_13ae3fe6;t0.sink.m[1] = 96'h000000000000000013ae3fe6;
    t0.src.m[2] = 99'h0_00000001_16414511_00000001;t0.sink.m[2] = 96'h000000000000000016414511;
    t0.src.m[3] = 99'h0_7fffffff_7fffffff_00000001;t0.sink.m[3] = 96'h000000003fffffff00000001;
    t0.src.m[4] = 99'h0_80000000_80000000_00000001;t0.sink.m[4] = 96'h000000004000000000000000;
    t0.src.m[5] = 99'h0_7fffffff_80000000_00000001;t0.sink.m[5] = 96'hffffffffc000000080000000;
    t0.src.m[6] = 99'h0_00000000_7fffffff_00000001;t0.sink.m[6] = 96'h000000000000000000000000;
    t0.src.m[7] = 99'h0_7fffffff_00000000_00000001;t0.sink.m[7] = 96'h000000000000000000000000;

    #5;   t0_reset = 1'b1;
    #20;  t0_reset = 1'b0;
    #100000; `VC_TEST_CHECK( "Is sink finished?", t0_done )

  end
  `VC_TEST_CASE_END
  `VC_TEST_SUITE_END( 1 )
endmodule
