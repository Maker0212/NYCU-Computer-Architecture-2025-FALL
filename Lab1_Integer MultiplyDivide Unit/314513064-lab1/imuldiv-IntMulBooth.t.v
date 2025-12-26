//========================================================================
// Test for Booth Mul Unit
//========================================================================

`include "imuldiv-MulDivReqMsg.v"
`include "imuldiv-IntMulBooth.v"
`include "vc-TestSource.v"
`include "vc-TestSink.v"
`include "vc-Test.v"

//------------------------------------------------------------------------
// Helper Module
//------------------------------------------------------------------------

module imuldiv_IntMulBooth_helper
  (
    input       clk,
    input       reset,
    output      done
  );

  wire [66:0] src_msg;
  wire [31:0] src_msg_a;
  wire [31:0] src_msg_b;
  wire        src_val;
  wire        src_rdy;
  wire        src_done;

  wire [63:0] sink_msg;
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
                                 .func (),
                                 .a    (src_msg_a),
                                 .b    (src_msg_b)
                               );

  imuldiv_IntMulBooth imul
                      (
                        .clk                (clk),
                        .reset              (reset),
                        .mulreq_msg_a       (src_msg_a),
                        .mulreq_msg_b       (src_msg_b),
                        .mulreq_val         (src_val),
                        .mulreq_rdy         (src_rdy),
                        .mulresp_msg_result (sink_msg),
                        .mulresp_val        (sink_val),
                        .mulresp_rdy        (sink_rdy)
                      );

  vc_TestSink#(64,3) sink
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
  initial
  begin
    $dumpfile("imuldiv-IntMulBooth.vcd");
    $dumpvars;
  end

  `VC_TEST_SUITE_BEGIN( "imuldiv-IntMulBooth" )

                      reg  t0_reset = 1'b1;
  wire t0_done;

  imuldiv_IntMulBooth_helper t0
                             (
                               .clk   (clk),
                               .reset (t0_reset),
                               .done  (t0_done)
                             );

  `VC_TEST_CASE_BEGIN( 1, "mul" )
                     begin

                       t0.src.m[0] = 67'h0_00000000_00000000;
                       t0.sink.m[0] = 64'h00000000_00000000;
                       t0.src.m[1] = 67'h0_00000001_00000001;
                       t0.sink.m[1] = 64'h00000000_00000001;
                       t0.src.m[2] = 67'h0_ffffffff_00000001;
                       t0.sink.m[2] = 64'hffffffff_ffffffff;
                       t0.src.m[3] = 67'h0_00000001_ffffffff;
                       t0.sink.m[3] = 64'hffffffff_ffffffff;
                       t0.src.m[4] = 67'h0_ffffffff_ffffffff;
                       t0.sink.m[4] = 64'h00000000_00000001;
                       t0.src.m[5] = 67'h0_00000008_00000003;
                       t0.sink.m[5] = 64'h00000000_00000018;
                       t0.src.m[6] = 67'h0_fffffff8_00000008;
                       t0.sink.m[6] = 64'hffffffff_ffffffc0;
                       t0.src.m[7] = 67'h0_fffffff8_fffffff8;
                       t0.sink.m[7] = 64'h00000000_00000040;
                       t0.src.m[8] = 67'h0_0deadbee_10000000;
                       t0.sink.m[8] = 64'h00deadbe_e0000000;
                       t0.src.m[9] = 67'h0_deadbeef_10000000;
                       t0.sink.m[9] = 64'hfdeadbee_f0000000;

                       // 續接老師的 m[0]..m[9] 之後加入：

                       // 0 * maxpos
                       t0.src.m[10] = 67'h0_00000000_7fffffff;
                       t0.sink.m[10] = 64'h00000000_00000000;

                       // -1 * 1 = -1
                       t0.src.m[11] = 67'h0_ffffffff_00000001;
                       t0.sink.m[11] = 64'hffffffff_ffffffff;

                       // 最小值 * 2 = -2^32
                       t0.src.m[12] = 67'h0_80000000_00000002;
                       t0.sink.m[12] = 64'hffffffff_00000000;

                       // maxpos * maxpos = 0x3fffffff00000001
                       t0.src.m[13] = 67'h0_7fffffff_7fffffff;
                       t0.sink.m[13] = 64'h3fffffff_00000001;

                       // 16 * (-1) = -16
                       t0.src.m[14] = 67'h0_00000010_ffffffff;
                       t0.sink.m[14] = 64'hffffffff_fffffff0;

                       // 隨機正*負
                       t0.src.m[15] = 67'h0_12345678_9abcdef0;
                       t0.sink.m[15] = 64'hf8cc93d6_242d2080;

                       // (-1) * (-1) = 1
                       t0.src.m[16] = 67'h0_ffffffff_ffffffff;
                       t0.sink.m[16] = 64'h00000000_00000001;

                       // 3 * (-2) = -6
                       t0.src.m[17] = 67'h0_00000003_fffffffe;
                       t0.sink.m[17] = 64'hffffffff_fffffffA;

                       // 最小值 * 最小值 = 2^62  (注意：負*負為正)
                       t0.src.m[18] = 67'h0_80000000_80000000;
                       t0.sink.m[18] = 64'h40000000_00000000;

                       t0.src.m[19] = 67'h0_00000002_00000003;
                       t0.sink.m[19] = 64'h00000000_00000006; // 2*3

                       t0.src.m[20] = 67'h0_80000000_ffffffff;
                       t0.sink.m[20] = 64'h00000000_80000000; // -2^31 * -1




                       #5;
                       t0_reset = 1'b1;
                       #20;
                       t0_reset = 1'b0;
                       #10000;
                       `VC_TEST_CHECK( "Is sink finished?", t0_done )

                                   end
                                   `VC_TEST_CASE_END

                                   `VC_TEST_SUITE_END( 1 )

                                 endmodule
