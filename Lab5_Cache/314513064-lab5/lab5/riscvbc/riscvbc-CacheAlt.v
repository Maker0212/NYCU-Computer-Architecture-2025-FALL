//=========================================================================
// Cache Alt Design
//=========================================================================

`ifndef RISCV_CACHE_ALT_V
`define RISCV_CACHE_ALT_V

`include "vc-RAMs.v"
`include "riscvbc-CacheMsg.v"
`include "riscvbc-VictimCache.v"

        module riscv_CacheAlt (
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
            input  [`VC_MEM_RESP_MSG_SZ(32)-1:0] cacheresp_msg,


            // flush
            input  tri0 flush,
            output flush_done
          );

          // Provide a default pull-down so an unconnected flush port is not floating
          pulldown (flush);

          parameter  NUM_BLOCK = 64;
          parameter  BLOCK_ADDR = 6;
          parameter  MEMREQ_ADDR = 5;
          parameter  BLOCK_SIZE = 512;
          parameter  SLOT_PER_BLOCK = 16;
          parameter  TAG_SIZE = 21;
          parameter  LEN_SIZE = 2;

          // {0, addr_index} is used => {1, addr_index}

          reg  [NUM_BLOCK-1:0] dirty ;
          reg  [NUM_BLOCK-1:0] valid ;
          reg  [31:0] old;
          wire [BLOCK_ADDR-1:0] cache_raddr0, cache_raddr1;

          wire set_full, set_half_full, set_empty;
          assign set_empty = (valid[cache_raddr0] == 0) && (valid[cache_raddr1] == 0);
          assign set_half_full = (valid[cache_raddr0] == 1) && (valid[cache_raddr1] == 0);
          assign set_full = (valid[cache_raddr0] == 1) && (valid[cache_raddr1] == 1);

          reg target_reg;
          wire target_way_sel;
          wire [BLOCK_ADDR-1:0] target_addr;
          wire target_valid;
          wire target_dirty;
          wire [TAG_SIZE-1:0] evict_tag_now;
          wire [BLOCK_SIZE-1:0] evict_block_now;
          assign target_way_sel = set_empty ? 1'b0 : (set_half_full ? 1'b1 : replace);
          assign target_addr = {target_way_sel, addr_index};
          assign target_valid = valid[target_addr];
          assign target_dirty = dirty[target_addr];
          assign evict_tag_now = target_way_sel ? cache_tag_rdata1 : cache_tag_rdata0;
          assign evict_block_now = target_way_sel ? cache_data_rdata1 : cache_data_rdata0;

          // NEED TO BE FIXED
          wire [BLOCK_ADDR-1:0] cache_waddr;

          wire addr_sel;
          assign addr_sel = (state == S_IDLE) ? 1'b0 : 1'b1;
          assign cache_raddr0 = (state == S_FLUSH) ? flush_addr : (addr_sel ? {1'b0, addr_index_reg} : {1'b0, addr_index});
          assign cache_raddr1 = addr_sel ? {1'b1, addr_index_reg} : {1'b1, addr_index};
          assign cache_waddr = addr_sel ? {target_reg, addr_index_reg} : {hit1, addr_index};

          wire [TAG_SIZE-1:0] cache_tag_rdata0;
          wire [TAG_SIZE-1:0] cache_tag_rdata1;
          reg  [TAG_SIZE-1:0] cache_tag_wdata;

          wire data_wen_p;
          reg  tag_wen_p;

          vc_RAM_1w2r_pf #(
                           .DATA_SZ(TAG_SIZE),
                           .ENTRIES(NUM_BLOCK),
                           .ADDR_SZ(BLOCK_ADDR)
                         ) tag (
                           .clk(clk),
                           .raddr0(cache_raddr0),
                           .rdata0(cache_tag_rdata0),
                           .raddr1(cache_raddr1),
                           .rdata1(cache_tag_rdata1),

                           .wen_p(tag_wen_p),
                           .waddr_p(cache_waddr),
                           .wdata_p(cache_tag_wdata)
                         );

          wire [BLOCK_SIZE-1:0] cache_data_rdata0, cache_data_rdata1;
          reg  [BLOCK_SIZE-1:0] cache_data_wdata;
          vc_RAM_1w2r_pf #(
                           .DATA_SZ(BLOCK_SIZE),
                           .ENTRIES(NUM_BLOCK),
                           .ADDR_SZ(BLOCK_ADDR)
                         ) data (
                           .clk(clk),
                           .raddr0(cache_raddr0),
                           .rdata0(cache_data_rdata0),
                           .raddr1(cache_raddr1),
                           .rdata1(cache_data_rdata1),

                           .wen_p(data_wen_p),
                           .waddr_p(cache_waddr),
                           .wdata_p(cache_data_wdata)
                         );


          // Req from CPU
          // 0: read, 1: write
          wire memreq_msg_type;
          wire [31:0] memreq_msg_addr;
          wire [31:0] memreq_msg_data;
          wire [1:0] memreq_msg_len;
          wire [1:0] len_mask;
          assign len_mask = (addr_sel) ? memreq_msg_len_reg : memreq_msg_len;

          assign memreq_msg_type = memreq_msg[`VC_MEM_REQ_MSG_TYPE_FIELD(32,32)];
          assign memreq_msg_addr = memreq_msg[`VC_MEM_REQ_MSG_ADDR_FIELD(32,32)];
          assign memreq_msg_len  = memreq_msg[`VC_MEM_REQ_MSG_LEN_FIELD(32,32)];
          assign memreq_msg_data = memreq_msg[`VC_MEM_REQ_MSG_DATA_FIELD(32,32)];

          wire [TAG_SIZE-1:0] addr_tag;
          wire [4:0] addr_index;
          wire [3:0] addr_slot;
          wire [1:0] addr_byte_offset;
          assign addr_tag = memreq_msg_addr[31:11];
          assign addr_index = memreq_msg_addr[10:6];
          assign addr_slot = memreq_msg_addr[5:2];
          assign addr_byte_offset = memreq_msg_addr[1:0];

          wire victim_lookup_hit;
          wire victim_lookup_way;
          wire [BLOCK_SIZE-1:0] victim_lookup_data;
          wire [1:0] victim_valid_bits;
          wire victim_repl_way;
          wire victim_lookup_en;

          reg victim_write_en;
          reg victim_write_way;
          reg [TAG_SIZE-1:0] victim_write_tag;
          reg [4:0] victim_write_idx;
          reg [BLOCK_SIZE-1:0] victim_write_data;
          reg victim_write_valid;

          // Debug: flag if we ever drive X to memresp or cachereq
          always @(posedge clk)
          begin
            if (!reset && memresp_val_reg && (^memresp_msg_reg === 1'bx))
            begin
              $display("DBG-X memresp_msg_reg time %0t state %0d addr %h type %b len %b", $time, state, memreq_msg_addr_reg, memreq_msg_type_reg, memreq_msg_len_reg);
            end
            if (!reset && cachereq_val_reg && (^cachereq_msg_reg === 1'bx))
            begin
              $display("DBG-X cachereq_msg_reg time %0t state %0d msg %h", $time, state, cachereq_msg_reg);
            end
            if (!reset && state == S_WRITE_BLOCK && cacheresp_val)
            begin
              if (target_reg && (^slot2mem_wdata1 === 1'bx))
                $display("DBG-X writeback data way1 time %0t idx %0d slot %0d", $time, addr_index_reg, slot2mem_addr_plus1);
              if (!target_reg && (^slot2mem_wdata0 === 1'bx))
                $display("DBG-X writeback data way0 time %0t idx %0d slot %0d", $time, addr_index_reg, slot2mem_addr_plus1);
            end
          end

          riscv_VictimCache victim_cache (
                              .clk(clk),
                              .reset(reset),
                              .lookup_en(victim_lookup_en),
                              .lookup_tag(addr_tag),
                              .lookup_idx(addr_index),
                              .lookup_hit(victim_lookup_hit),
                              .lookup_way(victim_lookup_way),
                              .lookup_data(victim_lookup_data),
                              .valid_bits(victim_valid_bits),
                              .repl_way(victim_repl_way),
                              .write_en(victim_write_en),
                              .write_way(victim_write_way),
                              .write_tag(victim_write_tag),
                              .write_idx(victim_write_idx),
                              .write_data(victim_write_data),
                              .write_valid(victim_write_valid)
                            );

          reg [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] memreq_msg_reg;
          wire memreq_msg_type_reg;
          wire [31:0] memreq_msg_addr_reg;
          wire [31:0] memreq_msg_data_reg;
          wire [1:0] memreq_msg_len_reg;

          assign memreq_msg_type_reg = memreq_msg_reg[`VC_MEM_REQ_MSG_TYPE_FIELD(32,32)];
          assign memreq_msg_addr_reg = memreq_msg_reg[`VC_MEM_REQ_MSG_ADDR_FIELD(32,32)];
          assign memreq_msg_len_reg  = memreq_msg_reg[`VC_MEM_REQ_MSG_LEN_FIELD(32,32)];
          assign memreq_msg_data_reg = memreq_msg_reg[`VC_MEM_REQ_MSG_DATA_FIELD(32,32)];

          wire [TAG_SIZE-1:0] addr_tag_reg;
          wire [4:0] addr_index_reg;
          wire [3:0] addr_slot_reg;
          wire [1:0] addr_byte_offset_reg;
          assign addr_tag_reg = memreq_msg_addr_reg[31:11];
          assign addr_index_reg = memreq_msg_addr_reg[10:6];
          assign addr_slot_reg = memreq_msg_addr_reg[5:2];
          assign addr_byte_offset_reg = memreq_msg_addr_reg[1:0];

          reg victim_hit_reg;
          reg victim_way_reg;
          reg [BLOCK_SIZE-1:0] victim_data_reg;
          reg victim_fill_pending;
          reg victim_insert_pending;
          reg victim_repl_way_reg;
          reg evict_valid_reg;
          reg evict_dirty_reg;
          reg [TAG_SIZE-1:0] evict_tag_reg;
          reg [BLOCK_SIZE-1:0] evict_block_reg;
          reg [4:0] evict_idx_reg;
          reg writeback_go_victim;

          // Req rdy
          assign memreq_rdy = (state == S_IDLE) && ~(memresp_val_reg && ~memresp_rdy);

          // Resp to CPU
          reg [`VC_MEM_RESP_MSG_SZ(32)-1:0] memresp_msg_reg;
          reg memresp_val_reg;

          wire is_read;
          assign is_read = (memreq_val && memreq_rdy) ? ~memreq_msg_type : 0;

          assign memresp_val = memresp_val_reg;
          assign memresp_msg = memresp_msg_reg;

          function [31:0] format_load;
            input [31:0] word;
            input [1:0] len;
            input [1:0] byte_off;
            reg [31:0] clean_word;
            begin
              clean_word = (^word === 1'bx) ? 32'b0 : word;
              case (len)
                2'b00:
                  format_load = clean_word;
                2'b01:
                case (byte_off)
                  2'b00:
                    format_load = {24'b0, clean_word[7:0]};
                  2'b01:
                    format_load = {24'b0, clean_word[15:8]};
                  2'b10:
                    format_load = {24'b0, clean_word[23:16]};
                  default:
                    format_load = {24'b0, clean_word[31:24]};
                endcase
                2'b10:
                  format_load = byte_off[1] ? {16'b0, clean_word[31:16]} : {16'b0, clean_word[15:0]};
                default:
                  format_load = clean_word;
              endcase
            end
          endfunction

          // Resp from MEM
          wire cacheresp_msg_type;
          wire [1:0] cacheresp_msg_len;
          wire [31:0] cacheresp_msg_data;
          assign {cacheresp_msg_type, cacheresp_msg_len, cacheresp_msg_data} = cacheresp_msg;

          // Resp rdy
          assign cacheresp_rdy = 1'b1;

          // Req to MEM
          reg [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] cachereq_msg_reg;
          reg cachereq_val_reg;
          assign cachereq_val = cachereq_val_reg;
          assign cachereq_msg = cachereq_msg_reg;

          reg [BLOCK_SIZE-1:0] write_block;
          wire  [31:0] cache_slot_data;
          reg   [31:0] cache_slot_data_masked;
          wire  [3:0]  cache_slot_addr;
          reg   [3:0]  slot_addr_reg;
          wire [31:0] victim_slot_data;
          wire [31:0] way0_slot_reg_data;
          wire [31:0] way1_slot_reg_data;
          wire [31:0] resp_read_word;

          assign cache_slot_addr = ((state == S_IDLE) && (memreq_val && memreq_rdy) && memreq_msg_type) ? addr_slot :
                 (state == S_READ_BLOCK) ? mem2slot_addr :
                 (state == S_VICTIM_FILL) ? mem2slot_addr :
                 (state == S_RESPONSE || state == S_WAIT_MEMRESP) ? addr_slot_reg : slot_addr_reg;

          assign victim_slot_data = victim_data_reg[cache_slot_addr*32 +: 32];

          assign way0_slot_reg_data =   (addr_slot_reg == 4'd0)  ? cache_data_rdata0[31:0] :
                 (addr_slot_reg == 4'd1)  ? cache_data_rdata0[63:32] :
                 (addr_slot_reg == 4'd2)  ? cache_data_rdata0[95:64] :
                 (addr_slot_reg == 4'd3)  ? cache_data_rdata0[127:96] :
                 (addr_slot_reg == 4'd4)  ? cache_data_rdata0[159:128] :
                 (addr_slot_reg == 4'd5)  ? cache_data_rdata0[191:160] :
                 (addr_slot_reg == 4'd6)  ? cache_data_rdata0[223:192] :
                 (addr_slot_reg == 4'd7)  ? cache_data_rdata0[255:224] :
                 (addr_slot_reg == 4'd8)  ? cache_data_rdata0[287:256] :
                 (addr_slot_reg == 4'd9)  ? cache_data_rdata0[319:288] :
                 (addr_slot_reg == 4'd10) ? cache_data_rdata0[351:320] :
                 (addr_slot_reg == 4'd11) ? cache_data_rdata0[383:352] :
                 (addr_slot_reg == 4'd12) ? cache_data_rdata0[415:384] :
                 (addr_slot_reg == 4'd13) ? cache_data_rdata0[447:416] :
                 (addr_slot_reg == 4'd14) ? cache_data_rdata0[479:448] : cache_data_rdata0[511:480];

          assign way1_slot_reg_data =   (addr_slot_reg == 4'd0)  ? cache_data_rdata1[31:0] :
                 (addr_slot_reg == 4'd1)  ? cache_data_rdata1[63:32] :
                 (addr_slot_reg == 4'd2)  ? cache_data_rdata1[95:64] :
                 (addr_slot_reg == 4'd3)  ? cache_data_rdata1[127:96] :
                 (addr_slot_reg == 4'd4)  ? cache_data_rdata1[159:128] :
                 (addr_slot_reg == 4'd5)  ? cache_data_rdata1[191:160] :
                 (addr_slot_reg == 4'd6)  ? cache_data_rdata1[223:192] :
                 (addr_slot_reg == 4'd7)  ? cache_data_rdata1[255:224] :
                 (addr_slot_reg == 4'd8)  ? cache_data_rdata1[287:256] :
                 (addr_slot_reg == 4'd9)  ? cache_data_rdata1[319:288] :
                 (addr_slot_reg == 4'd10) ? cache_data_rdata1[351:320] :
                 (addr_slot_reg == 4'd11) ? cache_data_rdata1[383:352] :
                 (addr_slot_reg == 4'd12) ? cache_data_rdata1[415:384] :
                 (addr_slot_reg == 4'd13) ? cache_data_rdata1[447:416] :
                 (addr_slot_reg == 4'd14) ? cache_data_rdata1[479:448] : cache_data_rdata1[511:480];

          assign resp_read_word = target_reg ? way1_slot_reg_data : way0_slot_reg_data;

          assign cache_slot_data = ((state == S_IDLE) && (memreq_val && memreq_rdy) && memreq_msg_type) ? memreq_msg_data :
                 (state == S_READ_BLOCK) ? cacheresp_msg_data :
                 (state == S_VICTIM_FILL) ? victim_slot_data :
                 (state == S_RESPONSE || state == S_WAIT_MEMRESP) ? (memreq_msg_type_reg ? memreq_msg_data_reg : resp_read_word) : cacheresp_msg_data;
          wire [31:0] cache_slot_data_clean = (^cache_slot_data === 1'bx) ? 32'b0 : cache_slot_data;

          always @(*)
          begin
            // Apply byte/half masking for store hits/misses; fills use full words
            if ( (state == S_IDLE && memreq_val && memreq_rdy && memreq_msg_type && hit) ||
                 ((state == S_RESPONSE || state == S_WAIT_MEMRESP) && memreq_msg_type_reg) )
            begin
              case(cache_slot_addr)
                4'd0:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[31:0]  : cache_data_rdata1[31:0];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[31:0];
                  else
                    cache_slot_data_masked = cache_data_rdata0[31:0];
                end
                4'd1:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[63:32]  : cache_data_rdata1[63:32];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[63:32];
                  else
                    cache_slot_data_masked = cache_data_rdata0[63:32];
                end
                4'd2:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[95:64]  : cache_data_rdata1[95:64];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[95:64];
                  else
                    cache_slot_data_masked = cache_data_rdata0[95:64];
                end
                4'd3:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[127:96] : cache_data_rdata1[127:96];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[127:96];
                  else
                    cache_slot_data_masked = cache_data_rdata0[127:96];
                end
                4'd4:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[159:128] : cache_data_rdata1[159:128];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[159:128];
                  else
                    cache_slot_data_masked = cache_data_rdata0[159:128];
                end
                4'd5:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[191:160] : cache_data_rdata1[191:160];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[191:160];
                  else
                    cache_slot_data_masked = cache_data_rdata0[191:160];
                end
                4'd6:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[223:192] : cache_data_rdata1[223:192];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[223:192];
                  else
                    cache_slot_data_masked = cache_data_rdata0[223:192];
                end
                4'd7:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[255:224] : cache_data_rdata1[255:224];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[255:224];
                  else
                    cache_slot_data_masked = cache_data_rdata0[255:224];
                end
                4'd8:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[287:256] : cache_data_rdata1[287:256];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[287:256];
                  else
                    cache_slot_data_masked = cache_data_rdata0[287:256];
                end
                4'd9:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[319:288] : cache_data_rdata1[319:288];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[319:288];
                  else
                    cache_slot_data_masked = cache_data_rdata0[319:288];
                end
                4'd10:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[351:320] : cache_data_rdata1[351:320];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[351:320];
                  else
                    cache_slot_data_masked = cache_data_rdata0[351:320];
                end
                4'd11:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[383:352] : cache_data_rdata1[383:352];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[383:352];
                  else
                    cache_slot_data_masked = cache_data_rdata0[383:352];
                end
                4'd12:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[415:384] : cache_data_rdata1[415:384];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[415:384];
                  else
                    cache_slot_data_masked = cache_data_rdata0[415:384];
                end
                4'd13:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[447:416] : cache_data_rdata1[447:416];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[447:416];
                  else
                    cache_slot_data_masked = cache_data_rdata0[447:416];
                end
                4'd14:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[479:448] : cache_data_rdata1[479:448];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[479:448];
                  else
                    cache_slot_data_masked = cache_data_rdata0[479:448];
                end
                4'd15:
                begin
                  if (state == S_IDLE)
                    cache_slot_data_masked = hit0 ? cache_data_rdata0[511:480] : cache_data_rdata1[511:480];
                  else if (target_reg)
                    cache_slot_data_masked = cache_data_rdata1[511:480];
                  else
                    cache_slot_data_masked = cache_data_rdata0[511:480];
                end
              endcase
              case({len_mask, byte_mask})
                4'b0000:
                  cache_slot_data_masked = cache_slot_data;
                4'b0001:
                  cache_slot_data_masked[31:8] = cache_slot_data[23:0];
                4'b0010:
                  cache_slot_data_masked[31:16] = cache_slot_data[15:0];
                4'b0011:
                  cache_slot_data_masked[31:24] = cache_slot_data[7:0];

                4'b0100:
                  cache_slot_data_masked[7:0] = cache_slot_data[7:0];
                4'b0101:
                  cache_slot_data_masked[15:8] = cache_slot_data[7:0];
                4'b0110:
                  cache_slot_data_masked[23:16] = cache_slot_data[7:0];
                4'b0111:
                  cache_slot_data_masked[31:24] = cache_slot_data[7:0];

                4'b1000:
                  cache_slot_data_masked[15:0] = cache_slot_data[15:0];
                4'b1001:
                  cache_slot_data_masked[23:8] = cache_slot_data[15:0];
                4'b1010:
                  cache_slot_data_masked[31:16] = cache_slot_data[15:0];
                4'b1011:
                  cache_slot_data_masked[31:24] = cache_slot_data[7:0];
              endcase
            end
            else
            begin
              cache_slot_data_masked = cache_slot_data_clean;
            end
          end

          wire [1:0] byte_mask;
          assign byte_mask = (addr_sel) ? addr_byte_offset_reg : addr_byte_offset;

          wire cache_way_sel;
          assign cache_way_sel = (state == S_IDLE) ? hit1 : target_reg;

          wire do_cache_data_write;
          assign do_cache_data_write = (state == S_READ_BLOCK) ||
                 (state == S_VICTIM_FILL) ||
                 (((state == S_RESPONSE) || (state == S_WAIT_MEMRESP)) && memreq_msg_type_reg) ||
                 ((state == S_IDLE) && memreq_val && memreq_rdy && memreq_msg_type && hit);


          always @(*)
          begin
            if (do_cache_data_write)
            begin
              case (cache_slot_addr)
                4'd0:
                begin
                  cache_data_wdata[31:0] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[511:32] = cache_data_rdata1[511:32];
                  end
                  else
                  begin
                    cache_data_wdata[511:32] = cache_data_rdata0[511:32];
                  end
                end
                4'd1:
                begin
                  cache_data_wdata[63:32] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[31:0] = cache_data_rdata1[31:0];
                    cache_data_wdata[511:64] = cache_data_rdata1[511:64];
                  end
                  else
                  begin
                    cache_data_wdata[31:0] = cache_data_rdata0[31:0];
                    cache_data_wdata[511:64] = cache_data_rdata0[511:64];
                  end
                end
                4'd2:
                begin
                  cache_data_wdata[95:64] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[63:0] = cache_data_rdata1[63:0];
                    cache_data_wdata[511:96] = cache_data_rdata1[511:96];
                  end
                  else
                  begin
                    cache_data_wdata[63:0] = cache_data_rdata0[63:0];
                    cache_data_wdata[511:96] = cache_data_rdata0[511:96];
                  end
                end
                4'd3:
                begin
                  cache_data_wdata[127:96] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[95:0] = cache_data_rdata1[95:0];
                    cache_data_wdata[511:128] = cache_data_rdata1[511:128];
                  end
                  else
                  begin
                    cache_data_wdata[95:0] = cache_data_rdata0[95:0];
                    cache_data_wdata[511:128] = cache_data_rdata0[511:128];
                  end
                end
                4'd4:
                begin
                  cache_data_wdata[159:128] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[127:0] = cache_data_rdata1[127:0];
                    cache_data_wdata[511:160] = cache_data_rdata1[511:160];
                  end
                  else
                  begin
                    cache_data_wdata[127:0] = cache_data_rdata0[127:0];
                    cache_data_wdata[511:160] = cache_data_rdata0[511:160];
                  end
                end
                4'd5:
                begin
                  cache_data_wdata[191:160] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[159:0] = cache_data_rdata1[159:0];
                    cache_data_wdata[511:192] = cache_data_rdata1[511:192];
                  end
                  else
                  begin
                    cache_data_wdata[159:0] = cache_data_rdata0[159:0];
                    cache_data_wdata[511:192] = cache_data_rdata0[511:192];
                  end
                end
                4'd6:
                begin
                  cache_data_wdata[223:192] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[191:0] = cache_data_rdata1[191:0];
                    cache_data_wdata[511:224] = cache_data_rdata1[511:224];
                  end
                  else
                  begin
                    cache_data_wdata[191:0] = cache_data_rdata0[191:0];
                    cache_data_wdata[511:224] = cache_data_rdata0[511:224];
                  end
                end
                4'd7:
                begin
                  cache_data_wdata[255:224] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[223:0] = cache_data_rdata1[223:0];
                    cache_data_wdata[511:256] = cache_data_rdata1[511:256];
                  end
                  else
                  begin
                    cache_data_wdata[223:0] = cache_data_rdata0[223:0];
                    cache_data_wdata[511:256] = cache_data_rdata0[511:256];
                  end
                end
                4'd8:
                begin
                  cache_data_wdata[287:256] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[255:0] = cache_data_rdata1[255:0];
                    cache_data_wdata[511:288] = cache_data_rdata1[511:288];
                  end
                  else
                  begin
                    cache_data_wdata[255:0] = cache_data_rdata0[255:0];
                    cache_data_wdata[511:288] = cache_data_rdata0[511:288];
                  end
                end
                4'd9:
                begin
                  cache_data_wdata[319:288] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[287:0] = cache_data_rdata1[287:0];
                    cache_data_wdata[511:320] = cache_data_rdata1[511:320];
                  end
                  else
                  begin
                    cache_data_wdata[287:0] = cache_data_rdata0[287:0];
                    cache_data_wdata[511:320] = cache_data_rdata0[511:320];
                  end
                end
                4'd10:
                begin
                  cache_data_wdata[351:320] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[319:0] = cache_data_rdata1[319:0];
                    cache_data_wdata[511:352] = cache_data_rdata1[511:352];
                  end
                  else
                  begin
                    cache_data_wdata[319:0] = cache_data_rdata0[319:0];
                    cache_data_wdata[511:352] = cache_data_rdata0[511:352];
                  end
                end
                4'd11:
                begin
                  cache_data_wdata[383:352] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[351:0] = cache_data_rdata1[351:0];
                    cache_data_wdata[511:384] = cache_data_rdata1[511:384];
                  end
                  else
                  begin
                    cache_data_wdata[351:0] = cache_data_rdata0[351:0];
                    cache_data_wdata[511:384] = cache_data_rdata0[511:384];
                  end
                end
                4'd12:
                begin
                  cache_data_wdata[415:384] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[383:0] = cache_data_rdata1[383:0];
                    cache_data_wdata[511:416] = cache_data_rdata1[511:416];
                  end
                  else
                  begin
                    cache_data_wdata[383:0] = cache_data_rdata0[383:0];
                    cache_data_wdata[511:416] = cache_data_rdata0[511:416];
                  end
                end
                4'd13:
                begin
                  cache_data_wdata[447:416] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[415:0] = cache_data_rdata1[415:0];
                    cache_data_wdata[511:448] = cache_data_rdata1[511:448];
                  end
                  else
                  begin
                    cache_data_wdata[415:0] = cache_data_rdata0[415:0];
                    cache_data_wdata[511:448] = cache_data_rdata0[511:448];
                  end
                end
                4'd14:
                begin
                  cache_data_wdata[479:448] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[447:0] = cache_data_rdata1[447:0];
                    cache_data_wdata[511:480] = cache_data_rdata1[511:480];
                  end
                  else
                  begin
                    cache_data_wdata[447:0] = cache_data_rdata0[447:0];
                    cache_data_wdata[511:480] = cache_data_rdata0[511:480];
                  end
                end
                4'd15:
                begin
                  cache_data_wdata[511:480] = cache_slot_data_masked;
                  if (cache_way_sel)
                  begin
                    cache_data_wdata[479:0] = cache_data_rdata1[479:0];
                  end
                  else
                  begin
                    cache_data_wdata[479:0] = cache_data_rdata0[479:0];
                  end
                end
                default:
                begin
                  if (cache_way_sel)
                  begin
                    cache_data_wdata = cache_data_rdata1;
                  end
                  else
                  begin
                    cache_data_wdata = cache_data_rdata0;
                  end
                end
              endcase
            end
            else
            begin
              if (cache_way_sel)
              begin
                cache_data_wdata = cache_data_rdata1;
              end
              else
              begin
                cache_data_wdata = cache_data_rdata0;
              end
            end
          end


          wire [31:0] hit_data0, hit_data1;
          wire hit0, hit1, hit;
          assign hit0 = valid[cache_raddr0] ? cache_tag_rdata0 == addr_tag : 0;
          assign hit1 = valid[cache_raddr1] ? cache_tag_rdata1 == addr_tag : 0;
          assign hit = hit0 || hit1;
          assign data_wen_p = ((state == S_READ_BLOCK) && cacheresp_val) ||
                 (state == S_VICTIM_FILL) ||
                 ((state == S_RESPONSE) && memreq_msg_type_reg) ||
                 ((state == S_IDLE) && memreq_val && memreq_rdy && memreq_msg_type && hit);
          assign victim_lookup_en = (state == S_IDLE) && memreq_val && memreq_rdy && ~hit;

          wire replace;
          assign replace = old[addr_index];
          wire [BLOCK_ADDR-1:0] addr_replace;
          assign addr_replace = (addr_sel) ? {target_reg, addr_index_reg} : target_addr;

          assign hit_data0 = (addr_slot == 4'd0)  ? cache_data_rdata0[31:0] :
                 (addr_slot == 4'd1)  ? cache_data_rdata0[63:32] :
                 (addr_slot == 4'd2)  ? cache_data_rdata0[95:64] :
                 (addr_slot == 4'd3)  ? cache_data_rdata0[127:96] :
                 (addr_slot == 4'd4)  ? cache_data_rdata0[159:128] :
                 (addr_slot == 4'd5)  ? cache_data_rdata0[191:160] :
                 (addr_slot == 4'd6)  ? cache_data_rdata0[223:192] :
                 (addr_slot == 4'd7)  ? cache_data_rdata0[255:224] :
                 (addr_slot == 4'd8)  ? cache_data_rdata0[287:256] :
                 (addr_slot == 4'd9)  ? cache_data_rdata0[319:288] :
                 (addr_slot == 4'd10) ? cache_data_rdata0[351:320] :
                 (addr_slot == 4'd11) ? cache_data_rdata0[383:352] :
                 (addr_slot == 4'd12) ? cache_data_rdata0[415:384] :
                 (addr_slot == 4'd13) ? cache_data_rdata0[447:416] :
                 (addr_slot == 4'd14) ? cache_data_rdata0[479:448] : cache_data_rdata0[511:480];

          assign hit_data1 = (addr_slot == 4'd0)  ? cache_data_rdata1[31:0] :
                 (addr_slot == 4'd1)  ? cache_data_rdata1[63:32] :
                 (addr_slot == 4'd2)  ? cache_data_rdata1[95:64] :
                 (addr_slot == 4'd3)  ? cache_data_rdata1[127:96] :
                 (addr_slot == 4'd4)  ? cache_data_rdata1[159:128] :
                 (addr_slot == 4'd5)  ? cache_data_rdata1[191:160] :
                 (addr_slot == 4'd6)  ? cache_data_rdata1[223:192] :
                 (addr_slot == 4'd7)  ? cache_data_rdata1[255:224] :
                 (addr_slot == 4'd8)  ? cache_data_rdata1[287:256] :
                 (addr_slot == 4'd9)  ? cache_data_rdata1[319:288] :
                 (addr_slot == 4'd10) ? cache_data_rdata1[351:320] :
                 (addr_slot == 4'd11) ? cache_data_rdata1[383:352] :
                 (addr_slot == 4'd12) ? cache_data_rdata1[415:384] :
                 (addr_slot == 4'd13) ? cache_data_rdata1[447:416] :
                 (addr_slot == 4'd14) ? cache_data_rdata1[479:448] : cache_data_rdata1[511:480];

          reg [3:0] slot2mem_addr, mem2slot_addr;
          wire [3:0] slot2mem_addr_plus1, mem2slot_addr_plus1;
          assign slot2mem_addr_plus1 = (slot2mem_addr == 4'd15) ? 4'd0 : slot2mem_addr + 1;
          assign mem2slot_addr_plus1 = (mem2slot_addr == 4'd15) ? 4'd0 : mem2slot_addr + 1;

          wire [31:0] slot2mem_wdata0, slot2mem_wdata1, flush_wdata;
          assign slot2mem_wdata0  = ((slot2mem_addr + 1) == 4'd0)  ? cache_data_rdata0[31:0] :
                 ((slot2mem_addr + 1) == 4'd1)  ? cache_data_rdata0[63:32] :
                 ((slot2mem_addr + 1) == 4'd2)  ? cache_data_rdata0[95:64] :
                 ((slot2mem_addr + 1) == 4'd3)  ? cache_data_rdata0[127:96] :
                 ((slot2mem_addr + 1) == 4'd4)  ? cache_data_rdata0[159:128] :
                 ((slot2mem_addr + 1) == 4'd5)  ? cache_data_rdata0[191:160] :
                 ((slot2mem_addr + 1) == 4'd6)  ? cache_data_rdata0[223:192] :
                 ((slot2mem_addr + 1) == 4'd7)  ? cache_data_rdata0[255:224] :
                 ((slot2mem_addr + 1) == 4'd8)  ? cache_data_rdata0[287:256] :
                 ((slot2mem_addr + 1) == 4'd9)  ? cache_data_rdata0[319:288] :
                 ((slot2mem_addr + 1) == 4'd10) ? cache_data_rdata0[351:320] :
                 ((slot2mem_addr + 1) == 4'd11) ? cache_data_rdata0[383:352] :
                 ((slot2mem_addr + 1) == 4'd12) ? cache_data_rdata0[415:384] :
                 ((slot2mem_addr + 1) == 4'd13) ? cache_data_rdata0[447:416] :
                 ((slot2mem_addr + 1) == 4'd14) ? cache_data_rdata0[479:448] : cache_data_rdata0[511:480];

          assign slot2mem_wdata1 =  ((slot2mem_addr + 1) == 4'd0)  ? cache_data_rdata1[31:0] :
                 ((slot2mem_addr + 1) == 4'd1)  ? cache_data_rdata1[63:32] :
                 ((slot2mem_addr + 1) == 4'd2)  ? cache_data_rdata1[95:64] :
                 ((slot2mem_addr + 1) == 4'd3)  ? cache_data_rdata1[127:96] :
                 ((slot2mem_addr + 1) == 4'd4)  ? cache_data_rdata1[159:128] :
                 ((slot2mem_addr + 1) == 4'd5)  ? cache_data_rdata1[191:160] :
                 ((slot2mem_addr + 1) == 4'd6)  ? cache_data_rdata1[223:192] :
                 ((slot2mem_addr + 1) == 4'd7)  ? cache_data_rdata1[255:224] :
                 ((slot2mem_addr + 1) == 4'd8)  ? cache_data_rdata1[287:256] :
                 ((slot2mem_addr + 1) == 4'd9)  ? cache_data_rdata1[319:288] :
                 ((slot2mem_addr + 1) == 4'd10) ? cache_data_rdata1[351:320] :
                 ((slot2mem_addr + 1) == 4'd11) ? cache_data_rdata1[383:352] :
                 ((slot2mem_addr + 1) == 4'd12) ? cache_data_rdata1[415:384] :
                 ((slot2mem_addr + 1) == 4'd13) ? cache_data_rdata1[447:416] :
                 ((slot2mem_addr + 1) == 4'd14) ? cache_data_rdata1[479:448] : cache_data_rdata1[511:480];

          assign flush_wdata = (flush_slot_addr == 4'd0)  ? cache_data_rdata0[31:0] :
                 (flush_slot_addr == 4'd1)  ? cache_data_rdata0[63:32] :
                 (flush_slot_addr == 4'd2)  ? cache_data_rdata0[95:64] :
                 (flush_slot_addr == 4'd3)  ? cache_data_rdata0[127:96] :
                 (flush_slot_addr == 4'd4)  ? cache_data_rdata0[159:128] :
                 (flush_slot_addr == 4'd5)  ? cache_data_rdata0[191:160] :
                 (flush_slot_addr == 4'd6)  ? cache_data_rdata0[223:192] :
                 (flush_slot_addr == 4'd7)  ? cache_data_rdata0[255:224] :
                 (flush_slot_addr == 4'd8)  ? cache_data_rdata0[287:256] :
                 (flush_slot_addr == 4'd9)  ? cache_data_rdata0[319:288] :
                 (flush_slot_addr == 4'd10) ? cache_data_rdata0[351:320] :
                 (flush_slot_addr == 4'd11) ? cache_data_rdata0[383:352] :
                 (flush_slot_addr == 4'd12) ? cache_data_rdata0[415:384] :
                 (flush_slot_addr == 4'd13) ? cache_data_rdata0[447:416] :
                 (flush_slot_addr == 4'd14) ? cache_data_rdata0[479:448] : cache_data_rdata0[511:480];

          wire [31:0] miss_resp_data;
          assign miss_resp_data = resp_read_word;


          reg [2:0] state;
          reg [2:0] next_state;
          reg flush_reg;
          assign flush_done = (state == S_FLUSH_DONE) ? 1'b1 : 1'b0;

          reg [BLOCK_ADDR-1:0] flush_addr;
          reg [3:0] flush_slot_addr;

          parameter S_IDLE = 3'd0;
          parameter S_WRITE_BLOCK = 3'd1;
          parameter S_READ_BLOCK = 3'd2;
          parameter S_VICTIM_FILL = 3'd3;
          parameter S_RESPONSE = 3'd4;
          parameter S_WAIT_MEMRESP = 3'd5;
          parameter S_FLUSH = 3'd6;
          parameter S_FLUSH_DONE = 3'd7;

          wire flush_safe = (flush === 1'b1);

          always @(posedge clk)
          begin
            if (reset)
            begin
              state <= S_IDLE;
              flush_reg <= 1'b0;
            end
            else
            begin
              state <= next_state;
              if (state == S_FLUSH_DONE)
              begin
                flush_reg <= 1'b0;
              end
              else if (flush_reg)
              begin
                flush_reg <= 1'b1;
              end
              else
              begin
                flush_reg <= flush_safe;
              end
            end
          end

          always @(*)
          begin
            case (state)
              S_IDLE:
              begin
                if (memreq_val && memreq_rdy)
                begin
                  if (hit)
                    next_state = is_read ? S_IDLE : S_WAIT_MEMRESP;
                  else if (target_dirty)
                    next_state = S_WRITE_BLOCK;
                  else if (victim_lookup_hit)
                    next_state = S_VICTIM_FILL;
                  else
                    next_state = S_READ_BLOCK;
                end
                else
                begin
                  next_state = (flush_reg) ? S_FLUSH : S_IDLE;
                end
              end
              S_WRITE_BLOCK:
              begin
                if (cacheresp_val && (slot2mem_addr == 4'd15))
                begin
                  next_state = writeback_go_victim ? S_VICTIM_FILL : S_READ_BLOCK;
                end
                else
                begin
                  next_state = S_WRITE_BLOCK;
                end
              end
              S_READ_BLOCK:
              begin
                if (cacheresp_val && (mem2slot_addr == 4'd15))
                begin
                  next_state = S_RESPONSE;
                end
                else
                begin
                  next_state = S_READ_BLOCK;
                end
              end
              S_VICTIM_FILL:
              begin
                next_state = (mem2slot_addr == 4'd15) ? S_RESPONSE : S_VICTIM_FILL;
              end
              S_RESPONSE:
              begin
                next_state = S_WAIT_MEMRESP;
              end
              S_WAIT_MEMRESP:
              begin
                if (memresp_val && memresp_rdy)
                begin
                  next_state = S_IDLE;
                end
                else
                begin
                  next_state = S_WAIT_MEMRESP;
                end
              end
              S_FLUSH:
              begin
                if (flush_addr == 6'd63)
                begin
                  if (dirty[flush_addr])
                  begin
                    next_state = (cacheresp_val && flush_slot_addr == 4'd15) ? S_FLUSH_DONE : S_FLUSH;
                  end
                  else
                  begin
                    next_state = S_FLUSH_DONE;
                  end
                end
                else
                begin
                  next_state = S_FLUSH;
                end
              end
              S_FLUSH_DONE:
              begin
                next_state = S_IDLE;
              end
              default:
                next_state = S_IDLE;
            endcase
          end

          always @(posedge clk)
          begin
            if (reset)
            begin
              slot2mem_addr <= 4'd0;
              mem2slot_addr <= 4'd0;
              dirty <= 0;
              valid <= 0;
              old <= 0;
              target_reg <= 0;
              tag_wen_p <= 1'b0;
              memreq_msg_reg <= 0;
              cachereq_val_reg <= 1'b0;
              memreq_msg_reg <= 0;
              victim_hit_reg <= 1'b0;
              victim_way_reg <= 1'b0;
              victim_data_reg <= {BLOCK_SIZE{1'b0}};
              victim_fill_pending <= 1'b0;
              victim_insert_pending <= 1'b0;
              victim_repl_way_reg <= 1'b0;
              evict_valid_reg <= 1'b0;
              evict_dirty_reg <= 1'b0;
              evict_tag_reg <= {TAG_SIZE{1'b0}};
              evict_block_reg <= {BLOCK_SIZE{1'b0}};
              evict_idx_reg <= 5'd0;
              writeback_go_victim <= 1'b0;
              victim_write_en <= 1'b0;
              victim_write_way <= 1'b0;
              victim_write_tag <= {TAG_SIZE{1'b0}};
              victim_write_idx <= 5'd0;
              victim_write_data <= {BLOCK_SIZE{1'b0}};
              victim_write_valid <= 1'b0;
              memresp_val_reg <= 1'b0;
              memresp_msg_reg <= {`VC_MEM_RESP_MSG_SZ(32){1'b0}};
              cachereq_msg_reg <= {`VC_MEM_REQ_MSG_SZ(32,32){1'b0}};
            end
            else
            begin
              if (memresp_val_reg && memresp_rdy)
              begin
                memresp_val_reg <= 1'b0;
              end
              victim_write_en <= 1'b0;
              case (state)
                S_IDLE:
                begin
                  tag_wen_p  <= 1'b0;
                  if (memreq_val && memreq_rdy)
                  begin
                    cachereq_val_reg <= 1'b0;
                    target_reg <= target_way_sel;
                    memreq_msg_reg <= memreq_msg;
                    victim_hit_reg <= victim_lookup_hit && ~hit;
                    victim_way_reg <= victim_lookup_way;
                    victim_data_reg <= victim_lookup_data;
                    victim_repl_way_reg <= victim_repl_way;
                    evict_valid_reg <= target_valid;
                    evict_dirty_reg <= target_dirty;
                    evict_tag_reg <= evict_tag_now;
                    evict_block_reg <= evict_block_now;
                    evict_idx_reg <= addr_index;
                    slot2mem_addr <= 4'd0;
                    mem2slot_addr <= 4'd0;
                    victim_insert_pending <= 1'b0;
                    victim_fill_pending <= victim_lookup_hit && ~hit;
                    writeback_go_victim <= victim_lookup_hit && target_dirty;
                    case ({hit, memreq_msg_type})
                      // Read miss
                      2'b00:
                      begin
                        victim_insert_pending <= target_valid;
                        if (target_dirty)
                        begin
                          cachereq_val_reg <= 1'b1;
                          cachereq_msg_reg <= {1'b1, {evict_tag_now, addr_index, 4'd0, 2'b00}, 2'b0, evict_block_now[31:0]};
                        end
                        else if (~victim_lookup_hit)
                        begin
                          cachereq_val_reg <= 1'b1;
                          cachereq_msg_reg <= {1'b0, {addr_tag, addr_index, 4'd0, 2'b00}, 2'b0, 32'b0};
                        end
                      end
                      // Write miss
                      2'b01:
                      begin
                        victim_insert_pending <= target_valid;
                        if (target_dirty)
                        begin
                          cachereq_val_reg <= 1'b1;
                          cachereq_msg_reg <= {1'b1, {evict_tag_now, addr_index, 4'd0, 2'b00}, 2'b0, evict_block_now[31:0]};
                        end
                        else if (~victim_lookup_hit)
                        begin
                          cachereq_val_reg <= 1'b1;
                          cachereq_msg_reg <= {1'b0, {addr_tag, addr_index, 4'd0, 2'b00}, 2'b0, 32'b0};
                        end
                      end
                      // Read hit
                      2'b10:
                      begin
                        memresp_msg_reg <= hit0
                        ? {1'b0, 2'b0, format_load(hit_data0, len_mask, addr_byte_offset)}
                        : {1'b0, 2'b0, format_load(hit_data1, len_mask, addr_byte_offset)};
                        memresp_val_reg <= 1'b1;
                        old[addr_index] <= (hit0) ? 1'b1 : 1'b0;
                        cachereq_val_reg <= 0;
                        victim_fill_pending <= 1'b0;
                        victim_insert_pending <= 1'b0;
                      end
                      // Write hit
                      2'b11:
                      begin
                        if (hit0)
                        begin
                          dirty[cache_raddr0] <= 1;
                        end
                        else
                        begin
                          dirty[cache_raddr1] <= 1;
                        end
                        old[addr_index] <= (hit0) ? 1'b1 : 1'b0;
                        memresp_msg_reg <= {1'b1, 2'b0, 32'd0};

                        target_reg <= ~hit0;
                        cachereq_val_reg <= 0;
                        victim_fill_pending <= 1'b0;
                        victim_insert_pending <= 1'b0;
                        memresp_val_reg <= 1'b1;
                      end
                    endcase
                  end
                  else if (flush_reg)
                  begin
                    flush_slot_addr <= 0;
                    flush_addr <= 0;
                  end
                end
                S_WRITE_BLOCK:
                begin
                  tag_wen_p  <= 1'b0;
                  if (cachereq_val)
                  begin
                    if (cachereq_rdy)
                    begin
                      cachereq_val_reg <= 1'b0;
                    end
                  end
                  else
                  begin
                    if (cacheresp_val)
                    begin
                      if (slot2mem_addr == 4'd15)
                      begin
                        cachereq_val_reg <= writeback_go_victim ? 1'b0 : 1'b1;
                        if (~writeback_go_victim)
                        begin
                          cachereq_msg_reg <= {1'b0, {addr_tag_reg, addr_index_reg, 4'd0, 2'b00}, 2'b0, 32'b0};
                        end
                      end
                      else
                      begin
                        cachereq_val_reg <= 1'b1;
                        if (target_reg)
                        begin
                          cachereq_msg_reg <= {1'b1, {cache_tag_rdata1, addr_index_reg, slot2mem_addr_plus1, 2'b00}, 2'b0, slot2mem_wdata1};
                        end
                        else
                        begin
                          cachereq_msg_reg <= {1'b1, {cache_tag_rdata0, addr_index_reg, slot2mem_addr_plus1, 2'b00}, 2'b0, slot2mem_wdata0};
                        end
                      end
                      slot2mem_addr <= (slot2mem_addr == 4'd15) ?  4'd0 : slot2mem_addr + 1;
                    end
                    else
                    begin
                      cachereq_val_reg <= 1'b0;
                    end
                  end
                end
                S_READ_BLOCK:
                begin
                  if (cachereq_val)
                  begin
                    if (cachereq_rdy)
                    begin
                      cachereq_val_reg <= 1'b0;
                    end
                  end
                  else
                  begin
                    if (cacheresp_val)
                    begin
                      cachereq_val_reg <= (mem2slot_addr == 4'd15) ? 1'b0 : 1'b1;
                      cachereq_msg_reg <= {1'b0, {addr_tag_reg, addr_index_reg, mem2slot_addr_plus1, 2'b00}, 2'b0, 32'd0};
                      mem2slot_addr <= (mem2slot_addr == 4'd15) ? 4'd0 : mem2slot_addr + 1;
                      tag_wen_p <= (mem2slot_addr == 4'd15) ? 1'b1 : 1'b0;

                      if (target_reg)
                      begin
                        valid[cache_raddr1] <= (mem2slot_addr == 4'd15) ? 1'b1 : valid[cache_raddr1];
                        if (mem2slot_addr == 4'd15 && ~memreq_msg_type_reg)
                          dirty[cache_raddr1] <= 1'b0;
                        cache_tag_wdata <= (mem2slot_addr == 4'd15) ? addr_tag_reg : cache_tag_rdata1;
                      end
                      else
                      begin
                        valid[cache_raddr0] <= (mem2slot_addr == 4'd15) ? 1'b1 : valid[cache_raddr0];
                        if (mem2slot_addr == 4'd15 && ~memreq_msg_type_reg)
                          dirty[cache_raddr0] <= 1'b0;
                        cache_tag_wdata <= (mem2slot_addr == 4'd15) ? addr_tag_reg : cache_tag_rdata0;
                      end

                      slot_addr_reg <= mem2slot_addr;
                    end
                    else
                    begin
                      cachereq_val_reg <= 1'b0;
                    end
                  end
                end
                S_VICTIM_FILL:
                begin
                  cachereq_val_reg <= 1'b0;
                  tag_wen_p <= (mem2slot_addr == 4'd15) ? 1'b1 : 1'b0;
                  slot_addr_reg <= mem2slot_addr;
                  mem2slot_addr <= (mem2slot_addr == 4'd15) ? 4'd0 : mem2slot_addr + 1;

                  if (target_reg)
                  begin
                    valid[cache_raddr1] <= (mem2slot_addr == 4'd15) ? 1'b1 : valid[cache_raddr1];
                    if (mem2slot_addr == 4'd15 && ~memreq_msg_type_reg)
                      dirty[cache_raddr1] <= 1'b0;
                    cache_tag_wdata <= (mem2slot_addr == 4'd15) ? addr_tag_reg : cache_tag_rdata1;
                  end
                  else
                  begin
                    valid[cache_raddr0] <= (mem2slot_addr == 4'd15) ? 1'b1 : valid[cache_raddr0];
                    if (mem2slot_addr == 4'd15 && ~memreq_msg_type_reg)
                      dirty[cache_raddr0] <= 1'b0;
                    cache_tag_wdata <= (mem2slot_addr == 4'd15) ? addr_tag_reg : cache_tag_rdata0;
                  end

                  if (mem2slot_addr == 4'd15)
                  begin
                    victim_write_en <= victim_hit_reg || victim_insert_pending;
                    victim_write_way <= victim_hit_reg ? victim_way_reg : victim_repl_way_reg;
                    victim_write_tag <= evict_tag_reg;
                    victim_write_idx <= evict_idx_reg;
                    victim_write_data <= evict_block_reg;
                    victim_write_valid <= victim_hit_reg ? evict_valid_reg : victim_insert_pending;
                    victim_insert_pending <= 1'b0;
                    victim_fill_pending <= 1'b0;
                    writeback_go_victim <= 1'b0;
                  end
                  else
                  begin
                    victim_write_en <= 1'b0;
                  end
                end
                S_RESPONSE:
                begin
                  if (memreq_msg_type_reg == 1'b0)
                  begin
                    memresp_msg_reg <= {1'b0, 2'b0, format_load(miss_resp_data, len_mask, addr_byte_offset_reg)};
                  end
                  else
                  begin
                    memresp_msg_reg <= {1'b1, 2'b0, 32'b0};
                    if (target_reg)
                    begin
                      dirty[cache_raddr1] <= 1'b1;
                    end
                    else
                    begin
                      dirty[cache_raddr0] <= 1'b1;
                    end
                  end
                  if (victim_insert_pending)
                  begin
                    victim_write_en <= 1'b1;
                    victim_write_way <= victim_repl_way_reg;
                    victim_write_tag <= evict_tag_reg;
                    victim_write_idx <= evict_idx_reg;
                    victim_write_data <= evict_block_reg;
                    victim_write_valid <= evict_valid_reg;
                    victim_insert_pending <= 1'b0;
                  end
                  old[addr_index_reg] <= (~target_reg);
                  tag_wen_p <= 1'b0;
                  memresp_val_reg <= 1'b1;
                end
                S_FLUSH:
                begin
                  if (dirty[flush_addr])
                  begin
                    if (cachereq_val_reg)
                    begin
                      if (cachereq_rdy)
                      begin
                        cachereq_val_reg <= 1'b0;
                      end
                    end
                    else
                    begin
                      if (cacheresp_val)
                      begin
                        cachereq_val_reg <= 1'b1;
                        cachereq_msg_reg <= {1'b1, {cache_tag_rdata0, flush_addr[4:0], flush_slot_addr, 2'b00}, 2'b0, flush_wdata};

                        flush_slot_addr <= (flush_slot_addr == 4'd15) ? 4'd0 : flush_slot_addr + 1;
                        flush_addr <= (flush_slot_addr == 4'd15) ? flush_addr + 1 : flush_addr;
                      end
                      else if (flush_slot_addr == 4'd0)
                      begin
                        cachereq_val_reg <= 1'b1;
                        cachereq_msg_reg <= {1'b1, {cache_tag_rdata0, flush_addr[4:0], flush_slot_addr, 2'b00}, 2'b0, flush_wdata};
                        flush_slot_addr <= flush_slot_addr + 1;
                      end
                      else
                      begin
                        cachereq_val_reg <= 1'b0;
                      end
                    end
                  end
                  else
                  begin
                    flush_addr <= (flush_addr == 6'd63) ? 0 : flush_addr + 1;
                    flush_slot_addr <= 4'd0;
                    cachereq_val_reg <= 1'b0;
                  end
                end
                S_FLUSH_DONE:
                begin
                  cachereq_val_reg <= 1'b0;
                  dirty <= 0;
                end
                S_WAIT_MEMRESP:
                begin
                  tag_wen_p <= 1'b0;
                end
                default:
                begin
                  slot2mem_addr <= 4'd0;
                  mem2slot_addr <= 4'd0;
                end
              endcase
            end
          end

        endmodule


`endif  /* RISCV_CACHE_ALT_V */
