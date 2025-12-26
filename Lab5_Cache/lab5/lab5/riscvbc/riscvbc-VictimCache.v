//====================================================================================
// Victim Cache Design
//====================================================================================

`ifndef RISCV_VICTIM_CACHE_V
`define RISCV_VICTIM_CACHE_V

`include "riscvbc-CacheMsg.v"
`include "vc-RAMs.v"

module riscv_VictimCache 
(
    input clk,
    input reset
    
);

/* Uncomment the following code when you start working on the victim cache */

// // TODO: design your 2-entry victim cache

// vc_RAM_rst_1w1r_pf #(
//     .DATA_SZ     (),
//     .ENTRIES     (),
//     .ADDR_SZ     (),
//     .RESET_VALUE ()
// ) VC_data (
//     .clk         (),
//     .reset_p     (),
//     .raddr       (),
//     .rdata       (),
//     .wen_p       (),
//     .waddr_p     (),
//     .wdata_p     ()
// ); 

// vc_RAM_rst_1w1r_pf #(
//     .DATA_SZ     (),
//     .ENTRIES     (),
//     .ADDR_SZ     (),
//     .RESET_VALUE ()
// ) VC_tag (
//     .clk         (),
//     .reset_p     (),
//     .raddr       (),
//     .rdata       (),
//     .wen_p       (),
//     .waddr_p     (),
//     .wdata_p     ()
// );


endmodule

`endif  /* RISCV_VICTIM_CACHE_V */