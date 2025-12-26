//=========================================================================
// 5-Stage RISCV Scoreboard
//=========================================================================

`ifndef RISCV_CORE_SCOREBOARD_V
`define RISCV_CORE_SCOREBOARD_V

`define FUNC_UNIT_ALU 1
`define FUNC_UNIT_MEM 2
`define FUNC_UNIT_MUL 3

`include "riscvooo-InstMsg.v"

// Didn't check whether the reg id is 0 or not.
// Thus may have unnecessary stalls

module riscv_CoreScoreboard
(
  input                   clk,
  input                   reset,
  input      [ 4:0]       src0,             // Source register 0
  input                   src0_en,          // Use source register 0
  input      [ 4:0]       src1,             // Source register 1
  input                   src1_en,          // Use source register 1
  input      [ 4:0]       dst,              // Destination register
  input                   dst_en,           // Write to destination register
  input      [ 2:0]       func_unit,        // Functional Unit
  input      [ 4:0]       latency,          // Instruction latency (one-hot)
  input                   inst_val_Dhl,     // Instruction valid
  input                   stall_Dhl,

  input      [`LOG_S-1:0] rob_alloc_slot,   // ROB slot allocated to dst reg
  input      [`LOG_S-1:0] rob_commit_slot,  // ROB slot emptied during commit
  input                   rob_commit_wen,   // ROB slot emptied during commit

  input      [ 4:0]       stalls,           // Input stall signals

  output reg [ 2:0]       src0_byp_mux_sel, // Source reg 0 byp mux
  output     [`LOG_S-1:0] src0_byp_rob_slot,// Source reg 0 ROB slot
  output reg [ 2:0]       src1_byp_mux_sel, // Source reg 1 byp mux
  output     [`LOG_S-1:0] src1_byp_rob_slot,// Source reg 1 ROB slot

  output                  stall_hazard,     // Destination register ready
  output     [ 1:0]       wb_mux_sel,       // Writeback mux sel out
  output                  stall_wb_hazard_M,
  output                  stall_wb_hazard_X
);

  reg              pending         [31:0];
  reg [2:0]        functional_unit [31:0];
  reg [4:0]        reg_latency     [31:0];
  reg [`LOG_S-1:0] reg_rob_slot    [31:0];

  reg [4:0]        wb_alu_latency;
  reg [4:0]        wb_mem_latency;
  reg [4:0]        wb_mul_latency;

  // Store ROB slots (for bypassing)

  always @(posedge clk) begin
    if( accept && (!stall_Dhl)) begin
      reg_rob_slot[dst] <= rob_alloc_slot;
    end
  end

  assign src0_byp_rob_slot = reg_rob_slot[src0];
  assign src1_byp_rob_slot = reg_rob_slot[src1];

  // Check if src registers are ready

  wire src0_can_byp = pending[src0] && (reg_latency[src0] < 5'b00100);
  wire src1_can_byp = pending[src1] && (reg_latency[src1] < 5'b00100);

  wire src0_ok = !pending[src0] || src0_can_byp || !src0_en;
  wire src1_ok = !pending[src1] || src1_can_byp || !src1_en;

  wire [4:0] stalls_alu = {3'b0, stalls[4], stalls[0]};
  wire [4:0] stalls_mem = {2'b0, stalls[4:3], stalls[0]};
  wire [4:0] stalls_muldiv = stalls;

  wire [4:0] reg_latency_cur = reg_latency[src0];

  always @(*) begin
    if (!pending[src0] || src0 == 5'b00000)
      src0_byp_mux_sel = 3'b0;
    else if (reg_latency[src0] == 5'b00001)
      src0_byp_mux_sel = 3'd4;
    else if (reg_latency[src0] == 5'b00000)
      //src0_byp_mux_sel = 3'd5; // UNCOMMENT THIS WHEN YOUR ROB IS READY!
      src0_byp_mux_sel = 3'd0; // DELETE THIS WHEN YOUR ROB IS READY!
    else
      src0_byp_mux_sel = functional_unit[src0];
  end

  always @(*) begin
    if (!pending[src1] || src1 == 5'b00000)
      src1_byp_mux_sel = 3'b0;
    else if (reg_latency[src1] == 5'b00001)
      src1_byp_mux_sel = 3'd4;
    else if (reg_latency[src1] == 5'b00000)
      //src1_byp_mux_sel = 3'd5; // UNCOMMENT THIS WHEN YOUR ROB IS READY!
      src1_byp_mux_sel = 3'd0;   // DELETE THIS WHEN YOUR ROB IS READY!
    else
      src1_byp_mux_sel = functional_unit[src1];
  end

  // Check for hazards

  wire stall_wb_hazard =
    ((wb_alu_latency >> 1) & latency) > 5'b0 ? 1'b1 :
    ((wb_mem_latency >> 1) & latency) > 5'b0 ? 1'b1 :
    ((wb_mul_latency >> 1) & latency) > 5'b0 ? 1'b1 : 1'b0;

  wire accept =
    src0_ok && src1_ok && !stall_wb_hazard && inst_val_Dhl;

  assign stall_hazard = ~accept;

  
  // Advance one cycle
  
  genvar r;
  generate
  for( r = 0; r < 32; r = r + 1)
  begin: sb_entry
    always @(posedge clk) begin
      if (reset) begin
        reg_latency[r]     <= 5'b0;
        pending[r]         <= 1'b0;
        functional_unit[r] <= 3'b0; 
      end else if ( accept && (r == dst) && (!stall_Dhl)) begin
        reg_latency[r]     <= latency;
        pending[r]         <= 1'b1;
        functional_unit[r] <= func_unit;
      end else begin

        pending[r]         <= pending[r] &&
          !(rob_commit_wen && rob_commit_slot == reg_rob_slot[r]);

        // Depending on what functional unit we're talking about,
        // we need to shift the stall vector over so that its stages
        // line up with the latency vector.
        if ((functional_unit[r] == `FUNC_UNIT_ALU)) begin
          reg_latency[r]     <= ( ( reg_latency[r] & (stalls_alu) ) |
                                ( ( reg_latency[r] & ~(stalls_alu) ) >> 1) );
        end
        else if ( functional_unit[r] == `FUNC_UNIT_MEM ) begin
          reg_latency[r]     <= ( ( reg_latency[r] & (stalls_mem) ) |
                                ( ( reg_latency[r] & ~(stalls_mem) ) >> 1) );
        end
        else begin
          reg_latency[r]     <= ( ( reg_latency[r] & stalls ) |
                                ( ( reg_latency[r] & ~stalls ) >> 1) );
        end
      end
    end
  end
  endgenerate

  // ALU Latency 

  always @(posedge clk) begin
    if (reset) begin
      wb_alu_latency <= 5'b0;
    end else if (accept && (func_unit == 2'd1) && (!stall_Dhl)) begin
      wb_alu_latency <= 
        (wb_alu_latency & (stalls_alu)) |
        ((wb_alu_latency & ~(stalls_alu)) >> 1) |
        latency;
    end else begin
      wb_alu_latency <= 
        (wb_alu_latency & (stalls_alu)) |
        ((wb_alu_latency & ~(stalls_alu)) >> 1);
    end
  end

  // MEM Latency 

  always @(posedge clk) begin
    if (reset) begin
      wb_mem_latency <= 5'b0;
    end else if (accept && (func_unit == 2'd2) && (!stall_Dhl)) begin
      wb_mem_latency <= 
        (wb_mem_latency & (stalls_mem)) |
        ((wb_mem_latency & ~(stalls_mem)) >> 1) |
        latency;
    end else begin
      wb_mem_latency <= 
        (wb_mem_latency & (stalls_mem)) |
        ((wb_mem_latency & ~(stalls_mem)) >> 1);
    end
  end

  // MUL Latency 

  always @(posedge clk) begin
    if (reset) begin
      wb_mul_latency <= 5'b0;
    end else if (accept && (func_unit == 2'd3) && (!stall_Dhl)) begin
      wb_mul_latency <= 
        (wb_mul_latency & stalls) |
        ((wb_mul_latency & ~stalls) >> 1) |
        latency;
    end else begin
      wb_mul_latency <= 
        (wb_mul_latency & stalls) |
        ((wb_mul_latency & ~stalls) >> 1);
    end
  end

  assign stall_wb_hazard_X = wb_alu_latency[1] && (wb_mul_latency[1] || wb_mem_latency[1]);
  assign stall_wb_hazard_M = wb_mem_latency[1] && (wb_mul_latency[1]);

  assign wb_mux_sel = (wb_mul_latency[1]) ? 2'd3 :
                      (wb_mem_latency[1]) ? 2'd2 :
                      (wb_alu_latency[1]) ? 2'd1 : 2'd0;

endmodule

`endif

