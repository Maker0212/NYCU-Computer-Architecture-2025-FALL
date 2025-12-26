//=========================================================================
// 7-Stage RISCV Register File
//=========================================================================

`ifndef RISCV_CORE_DPATH_REGFILE_V
`define RISCV_CORE_DPATH_REGFILE_V

module riscv_CoreDpathRegfile
(
  input         clk,
  input  [ 4:0] raddr0,  // Read 0 address (combinational input)
  output [31:0] rdata0,  // Read 0 data (combinational on raddr)
  input  [ 4:0] raddr1,  // Read 1 address (combinational input)
  output [31:0] rdata1,  // Read 1 data (combinational on raddr)
  input  [ 4:0] raddr2,  // Read 2 address (combinational input)
  output [31:0] rdata2,  // Read 2 data (combinational on raddr)
  input  [ 4:0] raddr3,  // Read 3 address (combinational input)
  output [31:0] rdata3,  // Read 3 data (combinational on raddr)
  input         wen0_p,  // Write 0 enable (sample on rising clk edge)
  input  [ 4:0] waddr0_p,// Write 0 address (sample on rising clk edge)
  input  [31:0] wdata0_p,// Write 0 data (sample on rising clk edge)
  input         wen1_p,  // Write 1 enable (sample on rising clk edge)
  input  [ 4:0] waddr1_p,// Write 1 address (sample on rising clk edge)
  input  [31:0] wdata1_p // Write 1 data (sample on rising clk edge)
);

  // We use an array of 32 bit register for the regfile itself
  reg [31:0] registers[31:0];

  // Combinational read ports
  assign rdata0 = ( raddr0 == 0 ) ? 32'b0 : registers[raddr0];
  assign rdata1 = ( raddr1 == 0 ) ? 32'b0 : registers[raddr1];
  assign rdata2 = ( raddr2 == 0 ) ? 32'b0 : registers[raddr2];
  assign rdata3 = ( raddr3 == 0 ) ? 32'b0 : registers[raddr3];

  // Write port is active only when wen is asserted
  always @( posedge clk )
  begin
    if ( wen0_p && (waddr0_p != 5'b0) )
      registers[waddr0_p] <= wdata0_p;
    if ( wen1_p && (waddr1_p != 5'b0) )
      registers[waddr1_p] <= wdata1_p;
  end

endmodule

`endif

