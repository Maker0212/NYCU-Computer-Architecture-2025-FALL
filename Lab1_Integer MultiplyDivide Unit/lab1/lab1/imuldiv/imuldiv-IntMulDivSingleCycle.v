//========================================================================
// Lab 1 - Single-Cycle Mul/Div Unit
//========================================================================

`ifndef RISCV_INT_MULDIV_SINGLE_V
`define RISCV_INT_MULDIV_SINGLE_V

`include "imuldiv-MulDivReqMsg.v"

module imuldiv_IntMulDivSingleCycle
(
  input         clk,
  input         reset,

  input   [2:0] muldivreq_msg_fn,      // Function of MulDiv Unit
  input  [31:0] muldivreq_msg_a,       // Operand A
  input  [31:0] muldivreq_msg_b,       // Operand B
  input         muldivreq_val,         // Request val Signal
  output        muldivreq_rdy,         // Request rdy Signal

  output [31:0] muldivresp_msg_result, // Result of operation
  output        muldivresp_val,        // Response val Signal
  input         muldivresp_rdy         // Response rdy Signal
);

  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  reg   [2:0] fn_reg;      // Register for storing function
  reg  [63:0] a_reg;       // Register for storing operand A
  reg  [31:0] b_reg;       // Register for storing operand B
  reg         val_reg;     // Register for storing valid bit

  always @( posedge clk ) begin

    // Stall the pipeline if the response interface is not ready
    if ( muldivresp_rdy ) begin
      fn_reg  <= muldivreq_msg_fn;
      a_reg   <= muldivreq_msg_a;
      b_reg   <= muldivreq_msg_b;
      val_reg <= muldivreq_val;
    end

  end

  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // Extract sign bits

  wire sign_bit_a = a_reg[31];
  wire sign_bit_b = b_reg[31];

  // Unsign operands if necessary

  wire [31:0] unsigned_a = ( sign_bit_a ) ? (~a_reg + 1'b1) : a_reg;
  wire [31:0] unsigned_b = ( sign_bit_b ) ? (~b_reg + 1'b1) : b_reg;

  // Computation logic

  wire [31:0] unsigned_result
    = ( fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_MUL )  ? unsigned_a * unsigned_b
    : ( fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_DIV )  ? unsigned_a / unsigned_b
    : ( fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_DIVU ) ? a_reg / b_reg
    : ( fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_REM )  ? unsigned_a % unsigned_b
    : ( fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_REMU ) ? a_reg % b_reg
    :                                                  32'bx;

  // Determine whether or not result is signed. Usually the result is
  // signed if one and only one of the input operands is signed. In other
  // words, the result is signed if the xor of the sign bits of the input
  // operands is true. Remainder opeartions are a bit trickier, and here
  // we simply assume that the result is signed if the dividend for the
  // rem operation is signed.

  wire is_result_signed_divmul = sign_bit_a ^ sign_bit_b;
  wire is_result_signed_rem    = sign_bit_a;

  wire is_result_signed
    = ( fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_MUL
    ||  fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_DIV ) ? is_result_signed_divmul
    : ( fn_reg == `IMULDIV_MULDIVREQ_MSG_FUNC_REM ) ? is_result_signed_rem
    :                                                 1'b0;

  assign muldivresp_msg_result
    = ( is_result_signed ) ? (~unsigned_result + 1'b1) : unsigned_result;

  // Set the val/rdy signals. The request is ready when the response is
  // ready, and the response is valid when there is valid data in the
  // input registers.

  assign muldivreq_rdy  = muldivresp_rdy;
  assign muldivresp_val = val_reg;

endmodule

`endif
