//=========================================================================
// Cache Both set as bypass
//=========================================================================

`ifndef RISCV_CACHE_DCACHE_V
`define RISCV_CACHE_DCACHE_V

`include "riscvbc-CacheBypass.v"
`include "riscvbc-CacheAlt.v"

module riscv_Cache_Dcache (
    input clk,
    input reset,

    // imem
    input                                  imemreq_val,
    output                                 imemreq_rdy,
    input  [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] imemreq_msg,

    // dmem
    input                                  dmemreq_val,
    output                                 dmemreq_rdy,
    input  [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] dmemreq_msg,

    // imem
    output                               imemresp_val,
    input                                imemresp_rdy,
    output [`VC_MEM_RESP_MSG_SZ(32)-1:0] imemresp_msg,

    // dmem
    output                               dmemresp_val,
    input                                dmemresp_rdy,
    output [`VC_MEM_RESP_MSG_SZ(32)-1:0] dmemresp_msg,

    //cache
    output                                 cache0req_val,
    input                                  cache0req_rdy,
    output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] cache0req_msg,

    input                                cache0resp_val,
    output                               cache0resp_rdy,
    input  [`VC_MEM_RESP_MSG_SZ(32)-1:0] cache0resp_msg,

    output                                 cache1req_val,
    input                                  cache1req_rdy,
    output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] cache1req_msg,

    input                                cache1resp_val,
    output                               cache1resp_rdy,
    input  [`VC_MEM_RESP_MSG_SZ(32)-1:0] cache1resp_msg
);

  riscv_CacheBypass icache (
      .clk  (clk),
      .reset(reset),

      // input request interface
      .memreq_val(imemreq_val),
      .memreq_rdy(imemreq_rdy),
      .memreq_msg(imemreq_msg),

      // input response interface
      .memresp_val(imemresp_val),
      .memresp_rdy(imemresp_rdy),
      .memresp_msg(imemresp_msg),


      // Memory interface
      .cachereq_val(cache0req_val),
      .cachereq_rdy(cache0req_rdy),
      .cachereq_msg(cache0req_msg),

      .cacheresp_val(cache0resp_val),
      .cacheresp_rdy(cache0resp_rdy),
      .cacheresp_msg(cache0resp_msg)
  );

  riscv_CacheAlt dcache (
      .clk  (clk),
      .reset(reset),

      // input request interface
      .memreq_val(dmemreq_val),
      .memreq_rdy(dmemreq_rdy),
      .memreq_msg(dmemreq_msg),

      // input response interface
      .memresp_val(dmemresp_val),
      .memresp_rdy(dmemresp_rdy),
      .memresp_msg(dmemresp_msg),

      // Memory interface
      .cachereq_val(cache1req_val),
      .cachereq_rdy(cache1req_rdy),
      .cachereq_msg(cache1req_msg),

      .cacheresp_val(cache1resp_val),
      .cacheresp_rdy(cache1resp_rdy),
      .cacheresp_msg(cache1resp_msg)
  );


endmodule


`endif  /* RISCV_CACHE_DCACHE_V */
