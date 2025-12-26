//=========================================================================
// 5-Stage RISCV Scoreboard
//=========================================================================

`ifndef RISCV_CORE_SCOREBOARD_V
`define RISCV_CORE_SCOREBOARD_V

        // Functional unit identifiers
`define FUNC_UNIT_ALU 1
`define FUNC_UNIT_MEM 2
`define FUNC_UNIT_MUL 3

        module riscv_CoreScoreboard
          (
            input         clk,
            input         reset,

            // Source operand information
            input  [ 4:0] src0,             // Source register 0
            input         src0_en,          // Use source register 0
            input  [ 4:0] src1,             // Source register 1
            input         src1_en,          // Use source register 1

            // Destination operand information
            input  [ 4:0] dst,              // Destination register
            input         dst_en,           // Write to destination register
            input  [ 2:0] func_unit,        // Functional Unit
            input  [ 4:0] latency,          // Instruction latency (one-hot)
            input         inst_val_Dhl,     // Instruction valid
            input         stall_Dhl,

            // ROB interface
            input  [ 3:0] rob_alloc_slot,   // ROB slot allocated to dst reg
            input  [ 3:0] rob_commit_slot,  // ROB slot emptied during commit
            input         rob_commit_wen,   // ROB commit write enable

            // Pipeline stall signals
            input  [ 4:0] stalls,           // Input stall signals

            // Bypass control outputs
            output reg [ 2:0] src0_byp_mux_sel, // Source reg 0 bypass mux
            output [ 3:0] src0_byp_rob_slot,    // Source reg 0 ROB slot
            output reg [ 2:0] src1_byp_mux_sel, // Source reg 1 bypass mux
            output [ 3:0] src1_byp_rob_slot,    // Source reg 1 ROB slot

            // Hazard detection outputs
            output        stall_hazard,         // Stall due to data hazard
            output [ 1:0] wb_mux_sel,           // Writeback mux selection
            output        stall_wb_hazard_M,    // Writeback hazard in M stage
            output        stall_wb_hazard_X,    // Writeback hazard in X stage

            // Performance monitoring (optional)
            output [ 4:0] num_pending_regs      // Number of registers with pending writes
          );

          //----------------------------------------------------------------------
          // Scoreboard State
          //----------------------------------------------------------------------

          reg       pending          [31:0];  // Register waiting for result
          reg [2:0] functional_unit  [31:0];  // Which functional unit producing result
          reg [4:0] reg_latency      [31:0];  // Latency countdown (one-hot)
          reg [3:0] reg_rob_slot     [31:0];  // ROB slot for this register

          reg [4:0] wb_alu_latency;       // ALU writeback latency tracker
          reg [4:0] wb_mem_latency;       // MEM writeback latency tracker
          reg [4:0] wb_mul_latency;       // MUL writeback latency tracker

          //----------------------------------------------------------------------
          // Performance Monitoring
          //----------------------------------------------------------------------

          // Count pending registers for performance analysis
          integer i;
          reg [4:0] pending_count;

          always @(*)
          begin
            pending_count = 5'b0;
            for (i = 1; i < 32; i = i + 1)
            begin
              if (pending[i])
                pending_count = pending_count + 5'b1;
            end
          end

          assign num_pending_regs = pending_count;

          //----------------------------------------------------------------------
          // ROB Slot Tracking & Hazard Detection
          //----------------------------------------------------------------------

          assign src0_byp_rob_slot = reg_rob_slot[src0];
          assign src1_byp_rob_slot = reg_rob_slot[src1];

          // Operand ready if: not pending, can bypass (latency < 4), or not used
          wire src0_can_byp = pending[src0] && (reg_latency[src0] < 5'b00100);
          wire src1_can_byp = pending[src1] && (reg_latency[src1] < 5'b00100);
          wire src0_ok = !pending[src0] || src0_can_byp || !src0_en;
          wire src1_ok = !pending[src1] || src1_can_byp || !src1_en;

          function [4:0] calc_stall_mask;
            input [2:0] fu_type;
            input [4:0] stall_vec;
            begin
              case (fu_type)
                `FUNC_UNIT_ALU:
                  calc_stall_mask = {3'b0, stall_vec[4], stall_vec[0]};
                `FUNC_UNIT_MEM:
                  calc_stall_mask = {2'b0, stall_vec[4:3], stall_vec[0]};
                default:
                  calc_stall_mask = stall_vec;
              endcase
            end
          endfunction

          function [4:0] shift_latency;
            input [4:0] lat;
            input [4:0] stall_mask;
            begin
              shift_latency = (lat & stall_mask) | ((lat & ~stall_mask) >> 1);
            end
          endfunction

          //----------------------------------------------------------------------
          // Bypass Selection with Priority Encoding
          //----------------------------------------------------------------------

          function [2:0] calc_bypass_sel;
            input        is_pending;
            input [4:0]  latency_val;
            input [2:0]  func_unit_val;
            input [4:0]  reg_id;
            begin
              // Priority: ROB > W > Functional Unit
              if (!is_pending || reg_id == 5'b0)
                calc_bypass_sel = 3'd0;          // RF (no bypass)
              else
              case (latency_val)
                5'b00000:
                  calc_bypass_sel = 3'd5;          // ROB bypass (highest priority when ready)
                5'b00001:
                  calc_bypass_sel = 3'd4;          // W stage bypass
                default:
                  calc_bypass_sel = func_unit_val; // X/M/X3 stage bypass
              endcase
            end
          endfunction

          always @(*)
          begin
            src0_byp_mux_sel = calc_bypass_sel(pending[src0], reg_latency[src0],
                                               functional_unit[src0], src0);
            src1_byp_mux_sel = calc_bypass_sel(pending[src1], reg_latency[src1],
                                               functional_unit[src1], src1);
          end

          //----------------------------------------------------------------------
          // Structural Hazard Detection
          //----------------------------------------------------------------------

          // Check for writeback structural hazards
          wire stall_wb_hazard =
               ((wb_alu_latency >> 1) & latency) > 5'b0 ? 1'b1 :
               ((wb_mem_latency >> 1) & latency) > 5'b0 ? 1'b1 :
               ((wb_mul_latency >> 1) & latency) > 5'b0 ? 1'b1 : 1'b0;

          // Accept instruction if no hazards detected
          wire accept = src0_ok && src1_ok && !stall_wb_hazard && inst_val_Dhl;

          assign stall_hazard = ~accept;

          //----------------------------------------------------------------------
          // Scoreboard Entry Updates
          //----------------------------------------------------------------------

          genvar r;
          generate
            for( r = 0; r < 32; r = r + 1)
            begin: sb_entry

              wire commit_clears_r = rob_commit_wen && (rob_commit_slot == reg_rob_slot[r]);
              wire allocate_to_r = accept && (r == dst) && !stall_Dhl;

              always @(posedge clk)
              begin
                if (reset)
                begin
                  reg_latency[r]     <= 5'b0;
                  pending[r]         <= 1'b0;
                  functional_unit[r] <= 3'b0;
                  reg_rob_slot[r]    <= 4'b0;
                end
                else if (allocate_to_r)
                begin
                  reg_latency[r]     <= latency;
                  pending[r]         <= 1'b1;
                  functional_unit[r] <= func_unit;
                  reg_rob_slot[r]    <= rob_alloc_slot;
                end
                else
                begin
                  pending[r]     <= pending[r] && !commit_clears_r;
                  reg_latency[r] <= shift_latency(reg_latency[r],
                                                  calc_stall_mask(functional_unit[r], stalls));
                end
              end
            end
          endgenerate

          function [4:0] update_wb_latency;
            input [4:0] current_latency;
            input [4:0] stall_mask;
            input       is_new_inst;
            input [4:0] new_latency;
            begin
              update_wb_latency = shift_latency(current_latency, stall_mask) |
                                (is_new_inst ? new_latency : 5'b0);
            end
          endfunction

          wire accept_alu = accept && (func_unit == `FUNC_UNIT_ALU) && (!stall_Dhl);
          wire accept_mem = accept && (func_unit == `FUNC_UNIT_MEM) && (!stall_Dhl);
          wire accept_mul = accept && (func_unit == `FUNC_UNIT_MUL) && (!stall_Dhl);

          always @(posedge clk)
          begin
            if (reset)
            begin
              wb_alu_latency <= 5'b0;
              wb_mem_latency <= 5'b0;
              wb_mul_latency <= 5'b0;
            end
            else
            begin
              wb_alu_latency <= update_wb_latency(wb_alu_latency, calc_stall_mask(`FUNC_UNIT_ALU, stalls), accept_alu, latency);
              wb_mem_latency <= update_wb_latency(wb_mem_latency, calc_stall_mask(`FUNC_UNIT_MEM, stalls), accept_mem, latency);
              wb_mul_latency <= update_wb_latency(wb_mul_latency, calc_stall_mask(`FUNC_UNIT_MUL, stalls), accept_mul, latency);
            end
          end

          //----------------------------------------------------------------------
          // Writeback Hazard and Mux Selection
          //----------------------------------------------------------------------

          // Detect structural hazards for specific stages
          assign stall_wb_hazard_X = wb_alu_latency[1] && (wb_mul_latency[1] || wb_mem_latency[1]);
          assign stall_wb_hazard_M = wb_mem_latency[1] && (wb_mul_latency[1]);

          // Priority-based writeback mux selection (MUL > MEM > ALU)
          assign wb_mux_sel = (wb_mul_latency[1]) ? 2'd3 :
                 (wb_mem_latency[1]) ? 2'd2 :
                 (wb_alu_latency[1]) ? 2'd1 : 2'd0;

        endmodule

`endif

