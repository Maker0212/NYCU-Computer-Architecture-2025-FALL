//=========================================================================
// Cache Bypass FL Module
//=========================================================================

`ifndef RISCV_CACHE_BYPASS_V
`define RISCV_CACHE_BYPASS_V

// `include "vc/mem-msgs.v"

module riscv_CacheBypass (
    input clk,
    input reset,


    // imem
    input                                  memreq_val,
    output                                 memreq_rdy,
    input  [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] memreq_msg,

    output                               memresp_val,
    input                                memresp_rdy,
    output [`VC_MEM_RESP_MSG_SZ(32)-1:0] memresp_msg,

    //cache
    output                                 cachereq_val,
    input                                  cachereq_rdy,
    output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] cachereq_msg,

    input                                cacheresp_val,
    output                               cacheresp_rdy,
    input  [`VC_MEM_RESP_MSG_SZ(32)-1:0] cacheresp_msg
);

  assign cachereq_val = memreq_val;
  assign memreq_rdy = cachereq_rdy;
  assign cachereq_msg = memreq_msg;

  assign memresp_val = cacheresp_val;
  assign cacheresp_rdy = memresp_rdy;
  assign memresp_msg = cacheresp_msg;

endmodule


`endif  /* RISCV_CACHE_BYPASS_V */
