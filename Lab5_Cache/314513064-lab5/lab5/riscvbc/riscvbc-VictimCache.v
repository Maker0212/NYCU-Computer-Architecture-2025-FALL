//====================================================================================
// Victim Cache Design
//====================================================================================

`ifndef RISCV_VICTIM_CACHE_V
`define RISCV_VICTIM_CACHE_V

`include "riscvbc-CacheMsg.v"

        module riscv_VictimCache
          (
            input clk,
            input reset,

            // lookup
            input                     lookup_en,
            input [`TAG_BITS-1:0]     lookup_tag,
            input [`IDX_BITS-1:0]     lookup_idx,
            output                    lookup_hit,
            output                    lookup_way,
            output [`BLK_SIZE-1:0]    lookup_data,

            // replacement info
            output [1:0]              valid_bits,
            output                    repl_way,

            // write/insert
            input                     write_en,
            input                     write_way,
            input [`TAG_BITS-1:0]     write_tag,
            input [`IDX_BITS-1:0]     write_idx,
            input [`BLK_SIZE-1:0]     write_data,
            input                     write_valid
          );

          reg [`BLK_SIZE-1:0] data_arr [1:0];
          reg [`TAG_BITS-1:0] tag_arr  [1:0];
          reg [`IDX_BITS-1:0] idx_arr  [1:0];
          reg                 valid_arr[1:0];
          reg                 lru_way; // 0 -> way0 victim, 1 -> way1 victim

          wire way0_hit = lookup_en && valid_arr[0] && (tag_arr[0] == lookup_tag) && (idx_arr[0] == lookup_idx);
          wire way1_hit = lookup_en && valid_arr[1] && (tag_arr[1] == lookup_tag) && (idx_arr[1] == lookup_idx);

          assign lookup_hit  = way0_hit || way1_hit;
          assign lookup_way  = way1_hit;
          assign lookup_data = (way1_hit) ? data_arr[1] : data_arr[0];

          assign valid_bits = { valid_arr[1], valid_arr[0] };

          assign repl_way = ( ~valid_arr[0] ) ? 1'b0 :
                 ( ~valid_arr[1] ) ? 1'b1 :
                 lru_way;

          integer i;
          always @ ( posedge clk )
          begin
            if ( reset )
            begin
              for ( i = 0; i < 2; i = i + 1 )
              begin
                valid_arr[i] <= 1'b0;
                tag_arr[i]   <= {`TAG_BITS{1'b0}};
                idx_arr[i]   <= {`IDX_BITS{1'b0}};
                data_arr[i]  <= {`BLK_SIZE{1'b0}};
              end
              lru_way <= 1'b0;
            end
            else if ( write_en )
            begin
              data_arr[write_way]  <= write_data;
              tag_arr[write_way]   <= write_tag;
              idx_arr[write_way]   <= write_idx;
              valid_arr[write_way] <= write_valid;
              lru_way              <= ~write_way;
            end
            else if ( lookup_hit )
            begin
              lru_way <= ~lookup_way;
            end
          end

        endmodule

`endif  /* RISCV_VICTIM_CACHE_V */
