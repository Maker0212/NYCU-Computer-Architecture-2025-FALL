//=========================================================================
// 5-Stage RISCV Reorder Buffer
//=========================================================================

`ifndef RISCV_CORE_REORDERBUFFER_V
`define RISCV_CORE_REORDERBUFFER_V

`include "riscvooo-InstMsg.v"

        module riscv_CoreReorderBuffer
          #(
             parameter ROB_SIZE = `SLOTS,
             parameter ADDR_WIDTH = `LOG_S
           )
           (
             input         clk,
             input         reset,

             // Allocation interface
             input                      rob_alloc_req_val,
             output                     rob_alloc_req_rdy,
             input  [ 4:0]              rob_alloc_req_preg,
             output [ADDR_WIDTH-1:0]    rob_alloc_resp_slot,

             // Fill interface (mark instruction as complete)
             input                      rob_fill_val,
             input  [ADDR_WIDTH-1:0]    rob_fill_slot,

             // Commit interface
             output                     rob_commit_wen,
             output [ADDR_WIDTH-1:0]    rob_commit_slot,
             output [ 4:0]              rob_commit_rf_waddr
           );

          //----------------------------------------------------------------------
          // ROB State
          //----------------------------------------------------------------------

          reg                   valid   [ROB_SIZE-1:0];  // Entry is allocated
          reg                   pending [ROB_SIZE-1:0];  // Waiting for completion
          reg [ 4:0]            phys_reg [ROB_SIZE-1:0]; // Destination register

          reg [ADDR_WIDTH-1:0]  head;                    // Oldest instruction
          reg [ADDR_WIDTH-1:0]  tail;                    // Next allocation slot

          //----------------------------------------------------------------------
          // ROB Full Detection
          //----------------------------------------------------------------------

          // Circular queue full: when tail+1 wraps around to head
          wire [ADDR_WIDTH-1:0] tail_plus_one = tail + {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
          wire rob_full = (tail_plus_one == head);

          //----------------------------------------------------------------------
          // Allocation Interface
          //----------------------------------------------------------------------

          assign rob_alloc_req_rdy   = ~rob_full;
          assign rob_alloc_resp_slot = tail;

          //----------------------------------------------------------------------
          // Commit Interface
          //----------------------------------------------------------------------

          wire can_commit = valid[head] && ~pending[head];

          assign rob_commit_wen      = can_commit;
          assign rob_commit_rf_waddr = phys_reg[head];  // Don't care when !can_commit
          assign rob_commit_slot     = head;            // Always provide head slot

          genvar idx;
          generate
            for (idx = 0; idx < ROB_SIZE; idx = idx + 1)
            begin: rob_entry

              wire alloc_this = rob_alloc_req_val && rob_alloc_req_rdy && (tail == idx);
              wire fill_this  = rob_fill_val && (rob_fill_slot == idx);
              wire commit_this = can_commit && (head == idx);

              always @(posedge clk)
              begin
                if (reset)
                begin
                  valid[idx]    <= 1'b0;
                  pending[idx]  <= 1'b0;
                  phys_reg[idx] <= 5'b0;
                end
                else
                begin
                  // Priority: Allocation > Fill > Commit
                  casez ({alloc_this, fill_this, commit_this})
                    3'b1??:  // Allocation (highest priority)
                    begin
                      valid[idx]    <= 1'b1;
                      pending[idx]  <= 1'b1;
                      phys_reg[idx] <= rob_alloc_req_preg;
                    end
                    3'b01?:  // Fill (mark complete)
                    begin
                      pending[idx] <= 1'b0;
                    end
                    3'b001:  // Commit (deallocate)
                    begin
                      valid[idx] <= 1'b0;
                    end
                    default:
                      ; // No change
                  endcase
                end
              end
            end
          endgenerate

          always @(posedge clk)
          begin
            if (reset)
            begin
              head <= {ADDR_WIDTH{1'b0}};
              tail <= {ADDR_WIDTH{1'b0}};
            end
            else
            begin
              if (rob_alloc_req_val && rob_alloc_req_rdy)
                tail <= tail_plus_one;
              if (can_commit)
                head <= head + {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
            end
          end

        endmodule

`endif
