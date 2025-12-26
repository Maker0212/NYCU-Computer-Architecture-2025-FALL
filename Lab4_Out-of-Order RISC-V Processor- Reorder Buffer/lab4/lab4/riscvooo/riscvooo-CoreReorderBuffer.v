//=========================================================================
// 5-Stage RISCV Reorder Buffer
//=========================================================================

`ifndef RISCV_CORE_REORDERBUFFER_V
`define RISCV_CORE_REORDERBUFFER_V

`include "riscvooo-InstMsg.v"

module riscv_CoreReorderBuffer
(
  input               clk,
  input               reset,

  input               rob_alloc_req_val,
  output              rob_alloc_req_rdy, 
  input  [ 4:0]       rob_alloc_req_preg,
  output [`LOG_S-1:0] rob_alloc_resp_slot,

  input               rob_fill_val,  
  input  [`LOG_S-1:0] rob_fill_slot,

  output              rob_commit_wen,
  output [`LOG_S-1:0] rob_commit_slot,
  output [ 4:0]       rob_commit_rf_waddr
);
  // original
  assign rob_alloc_req_rdy   = 1'b1;
  assign rob_alloc_resp_slot = `LOG_S'b0;
  assign rob_commit_wen      = 1'b0;
  assign rob_commit_rf_waddr = 5'b0;
  assign rob_commit_slot     = `LOG_S'b0;

endmodule

`endif
