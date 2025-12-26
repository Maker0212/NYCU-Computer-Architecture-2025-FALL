//=========================================================================
// 7-Stage RISCV Scoreboard
//=========================================================================

`ifndef RISCV_CORE_SCOREBOARD_V
`define RISCV_CORE_SCOREBOARD_V

module riscv_CoreScoreboard
(
  input            clk              ,
  input            reset            ,

  input            inst_val_Dhl     ,

  input      [4:0] src00            ,
  input            src00_en         ,
  input      [4:0] src01            ,
  input            src01_en         ,
  input      [4:0] src10            ,
  input            src10_en         ,
  input      [4:0] src11            ,
  input            src11_en         ,

  output           stall_0_hazard   ,
  output           stall_1_hazard   ,

  output reg [3:0] src00_byp_mux_sel,
  output reg [3:0] src01_byp_mux_sel,
  output reg [3:0] src10_byp_mux_sel,
  output reg [3:0] src11_byp_mux_sel,

  input      [4:0] dstA             ,
  input            dstA_en          ,
  input            stall_A_Dhl      ,
  input            is_muldiv_A      ,
  input            is_load_A        ,

  input            stall_X0hl       ,
  input            stall_X1hl
);

  // TODO: implement your scoreboard here!

  genvar r;
  generate
    for ( r = 0; r < 32; r = r + 1 ) begin
      // TODO
    end
  endgenerate

endmodule

`endif
