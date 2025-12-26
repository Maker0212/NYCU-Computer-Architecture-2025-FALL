//========================================================================
// Lab 1 - Three Input Iterative Mul Unit
//========================================================================

`ifndef RISCV_INT_MULDIV_THREEINPUT_V
`define RISCV_INT_MULDIV_THREEINPUT_V

`include "imuldiv-ThreeMulReqMsg.v"

module imuldiv_IntMulThreeInput
(
  input         clk,
  input         reset,

  input   [2:0] muldivreq_msg_fn,
  input  [31:0] muldivreq_msg_a,
  input  [31:0] muldivreq_msg_b,
  input  [31:0] muldivreq_msg_c,
  input         muldivreq_val,
  output        muldivreq_rdy,

  output [95:0] muldivresp_msg_result,
  output        muldivresp_val,
  input         muldivresp_rdy
);

 // TODO

endmodule

`endif
