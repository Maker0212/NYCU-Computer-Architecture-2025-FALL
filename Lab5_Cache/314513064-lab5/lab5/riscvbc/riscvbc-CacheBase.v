//=========================================================================
// Cache Base Design
//=========================================================================

`ifndef RISCV_CACHE_BASE_V
`define RISCV_CACHE_BASE_V

`include "vc-RAMs.v"

        module riscv_CacheBase (
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

          parameter  NUM_BLOCK = 32;
          parameter  BLOCK_ADDR = 5;
          parameter  BLOCK_SIZE = 512;
          parameter  SLOT_PER_BLOCK = 16;
          parameter  TAG_SIZE = 21;
          parameter  LEN_SIZE = 2;

          reg  [NUM_BLOCK-1:0] dirty ;
          reg  [NUM_BLOCK-1:0] valid ;
          wire [BLOCK_ADDR-1:0] cache_raddr;
          wire [BLOCK_ADDR-1:0] cache_waddr;

          wire addr_sel;
          assign addr_sel = (state == S_IDLE) ? 1'b0 : 1'b1;
          assign cache_raddr = (state == S_FLUSH) ? flush_addr :(addr_sel ? addr_index_reg : addr_index);
          assign cache_waddr = addr_sel ? addr_index_reg : addr_index;

          wire [TAG_SIZE-1:0] cache_tag_rdata;
          reg  [TAG_SIZE-1:0] cache_tag_wdata;

          reg data_wen_p;
          reg tag_wen_p;

          vc_RAM_1w1r_pf #(
                           .DATA_SZ(TAG_SIZE),
                           .ENTRIES(NUM_BLOCK),
                           .ADDR_SZ(BLOCK_ADDR)
                         ) tag (
                           .clk(clk),
                           .raddr(cache_raddr),
                           .rdata(cache_tag_rdata),
                           .wen_p(tag_wen_p),
                           .waddr_p(cache_waddr),
                           .wdata_p(cache_tag_wdata)
                         );

          wire [BLOCK_SIZE-1:0] cache_data_rdata;
          reg  [BLOCK_SIZE-1:0] cache_data_wdata;
          vc_RAM_1w1r_pf #(
                           .DATA_SZ(BLOCK_SIZE),
                           .ENTRIES(NUM_BLOCK),
                           .ADDR_SZ(BLOCK_ADDR)
                         ) data (
                           .clk(clk),
                           .raddr(cache_raddr),
                           .rdata(cache_data_rdata),

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

          // Req rdy
          assign memreq_rdy = (state == S_IDLE) && ~(memresp_val_reg && ~memresp_rdy);

          // Resp to CPU
          reg [`VC_MEM_RESP_MSG_SZ(32)-1:0] memresp_msg_reg;
          reg memresp_val_reg;

          wire is_read;
          assign is_read = (memreq_val && memreq_rdy) ? ~memreq_msg_type : 0;

          wire hit_resp_en = hit && memreq_val && memreq_rdy;
          wire [`VC_MEM_RESP_MSG_SZ(32)-1:0] hit_resp_msg = memreq_msg_type ? {1'b1, 2'b0, 32'd0}
               : {1'b0, 2'b0, hit_data};

          assign memresp_val = memresp_val_reg;
          assign memresp_msg = memresp_msg_reg;

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
          wire  [3:0]  cache_slot_addr;
          reg   [3:0]  slot_addr_reg;
          assign cache_slot_data = (state == S_IDLE && (memreq_val && memreq_rdy && memreq_msg_type)) ? memreq_msg_data :
                 (state == S_READ_BLOCK) ? cacheresp_msg_data :
                 (state == S_RESPONSE && memreq_msg_type_reg) ? memreq_msg_data_reg : cacheresp_msg_data;
          assign cache_slot_addr = (state == S_IDLE && (memreq_val && memreq_rdy && memreq_msg_type)) ? addr_slot :
                 (state == S_READ_BLOCK) ? slot_addr_reg :
                 (state == S_RESPONSE && memreq_msg_type_reg) ? addr_slot_reg : slot_addr_reg;

          always @(*)
          begin
            if ((state == S_IDLE) || (state == S_RESPONSE) || (state == S_READ_BLOCK))
            begin
              case (cache_slot_addr)
                4'd0:
                begin
                  cache_data_wdata[31:0] = cache_slot_data;
                  cache_data_wdata[511:32] = cache_data_rdata[511:32];
                end
                4'd1:
                begin
                  cache_data_wdata[31:0] = cache_data_rdata[31:0];
                  cache_data_wdata[63:32] = cache_slot_data;
                  cache_data_wdata[511:64] = cache_data_rdata[511:64];
                end
                4'd2:
                begin
                  cache_data_wdata[63:0] = cache_data_rdata[63:0];
                  cache_data_wdata[95:64] = cache_slot_data;
                  cache_data_wdata[511:96] = cache_data_rdata[511:96];
                end
                4'd3:
                begin
                  cache_data_wdata[95:0] = cache_data_rdata[95:0];
                  cache_data_wdata[127:96] = cache_slot_data;
                  cache_data_wdata[511:128] = cache_data_rdata[511:128];
                end
                4'd4:
                begin
                  cache_data_wdata[127:0] = cache_data_rdata[127:0];
                  cache_data_wdata[159:128] = cache_slot_data;
                  cache_data_wdata[511:160] = cache_data_rdata[511:160];
                end
                4'd5:
                begin
                  cache_data_wdata[159:0] = cache_data_rdata[159:0];
                  cache_data_wdata[191:160] = cache_slot_data;
                  cache_data_wdata[511:192] = cache_data_rdata[511:192];
                end
                4'd6:
                begin
                  cache_data_wdata[191:0] = cache_data_rdata[191:0];
                  cache_data_wdata[223:192] = cache_slot_data;
                  cache_data_wdata[511:224] = cache_data_rdata[511:224];
                end
                4'd7:
                begin
                  cache_data_wdata[223:0] = cache_data_rdata[223:0];
                  cache_data_wdata[255:224] = cache_slot_data;
                  cache_data_wdata[511:256] = cache_data_rdata[511:256];
                end
                4'd8:
                begin
                  cache_data_wdata[255:0] = cache_data_rdata[255:0];
                  cache_data_wdata[287:256] = cache_slot_data;
                  cache_data_wdata[511:288] = cache_data_rdata[511:288];
                end
                4'd9:
                begin
                  cache_data_wdata[287:0] = cache_data_rdata[287:0];
                  cache_data_wdata[319:288] = cache_slot_data;
                  cache_data_wdata[511:320] = cache_data_rdata[511:320];
                end
                4'd10:
                begin
                  cache_data_wdata[319:0] = cache_data_rdata[319:0];
                  cache_data_wdata[351:320] = cache_slot_data;
                  cache_data_wdata[511:352] = cache_data_rdata[511:352];
                end
                4'd11:
                begin
                  cache_data_wdata[351:0] = cache_data_rdata[351:0];
                  cache_data_wdata[383:352] = cache_slot_data;
                  cache_data_wdata[511:384] = cache_data_rdata[511:384];
                end
                4'd12:
                begin
                  cache_data_wdata[383:0] = cache_data_rdata[383:0];
                  cache_data_wdata[415:384] = cache_slot_data;
                  cache_data_wdata[511:416] = cache_data_rdata[511:416];
                end
                4'd13:
                begin
                  cache_data_wdata[415:0] = cache_data_rdata[415:0];
                  cache_data_wdata[447:416] = cache_slot_data;
                  cache_data_wdata[511:448] = cache_data_rdata[511:448];
                end
                4'd14:
                begin
                  cache_data_wdata[447:0] = cache_data_rdata[447:0];
                  cache_data_wdata[479:448] = cache_slot_data;
                  cache_data_wdata[511:480] = cache_data_rdata[511:480];
                end
                4'd15:
                begin
                  cache_data_wdata[479:0] = cache_data_rdata[479:0];
                  cache_data_wdata[511:480] = cache_slot_data;
                end
                default:
                  cache_data_wdata = cache_data_rdata;
              endcase
            end
            else
            begin
              cache_data_wdata = cache_data_rdata;
            end
          end

          // Flush
          // reg flush_reg, flush_done_reg;
          // assign flush_done = flush_done_reg;

          // assign flush_done = 1'b1;

          wire [31:0] hit_data;
          wire hit, is_dirty;
          assign hit = valid[addr_index] ? cache_tag_rdata == addr_tag : 0;
          assign is_dirty = (dirty[addr_index] == 1);
          assign hit_data = (addr_slot == 4'd0)  ? cache_data_rdata[31:0] :
                 (addr_slot == 4'd1)  ? cache_data_rdata[63:32] :
                 (addr_slot == 4'd2)  ? cache_data_rdata[95:64] :
                 (addr_slot == 4'd3)  ? cache_data_rdata[127:96] :
                 (addr_slot == 4'd4)  ? cache_data_rdata[159:128] :
                 (addr_slot == 4'd5)  ? cache_data_rdata[191:160] :
                 (addr_slot == 4'd6)  ? cache_data_rdata[223:192] :
                 (addr_slot == 4'd7)  ? cache_data_rdata[255:224] :
                 (addr_slot == 4'd8)  ? cache_data_rdata[287:256] :
                 (addr_slot == 4'd9)  ? cache_data_rdata[319:288] :
                 (addr_slot == 4'd10) ? cache_data_rdata[351:320] :
                 (addr_slot == 4'd11) ? cache_data_rdata[383:352] :
                 (addr_slot == 4'd12) ? cache_data_rdata[415:384] :
                 (addr_slot == 4'd13) ? cache_data_rdata[447:416] :
                 (addr_slot == 4'd14) ? cache_data_rdata[479:448] : cache_data_rdata[511:480];

          reg [3:0] slot2mem_addr, mem2slot_addr;
          wire [3:0] slot2mem_addr_plus1, mem2slot_addr_plus1;
          assign slot2mem_addr_plus1 = (slot2mem_addr == 4'd15) ? 4'd0 : slot2mem_addr + 1;
          assign mem2slot_addr_plus1 = (mem2slot_addr == 4'd15) ? 4'd0 : mem2slot_addr + 1;

          wire [31:0] slot2mem_wdata, flush_wdata;
          assign slot2mem_wdata  = ((slot2mem_addr + 1) == 4'd0)  ? cache_data_rdata[31:0] :
                 ((slot2mem_addr + 1) == 4'd1)  ? cache_data_rdata[63:32] :
                 ((slot2mem_addr + 1) == 4'd2)  ? cache_data_rdata[95:64] :
                 ((slot2mem_addr + 1) == 4'd3)  ? cache_data_rdata[127:96] :
                 ((slot2mem_addr + 1) == 4'd4)  ? cache_data_rdata[159:128] :
                 ((slot2mem_addr + 1) == 4'd5)  ? cache_data_rdata[191:160] :
                 ((slot2mem_addr + 1) == 4'd6)  ? cache_data_rdata[223:192] :
                 ((slot2mem_addr + 1) == 4'd7)  ? cache_data_rdata[255:224] :
                 ((slot2mem_addr + 1) == 4'd8)  ? cache_data_rdata[287:256] :
                 ((slot2mem_addr + 1) == 4'd9)  ? cache_data_rdata[319:288] :
                 ((slot2mem_addr + 1) == 4'd10) ? cache_data_rdata[351:320] :
                 ((slot2mem_addr + 1) == 4'd11) ? cache_data_rdata[383:352] :
                 ((slot2mem_addr + 1) == 4'd12) ? cache_data_rdata[415:384] :
                 ((slot2mem_addr + 1) == 4'd13) ? cache_data_rdata[447:416] :
                 ((slot2mem_addr + 1) == 4'd14) ? cache_data_rdata[479:448] : cache_data_rdata[511:480];

          wire [31:0] response_slot_data;
          assign response_slot_data =  (addr_slot_reg == 4'd0)  ? cache_data_rdata[31:0] :
                 (addr_slot_reg == 4'd1)  ? cache_data_rdata[63:32] :
                 (addr_slot_reg == 4'd2)  ? cache_data_rdata[95:64] :
                 (addr_slot_reg == 4'd3)  ? cache_data_rdata[127:96] :
                 (addr_slot_reg == 4'd4)  ? cache_data_rdata[159:128] :
                 (addr_slot_reg == 4'd5)  ? cache_data_rdata[191:160] :
                 (addr_slot_reg == 4'd6)  ? cache_data_rdata[223:192] :
                 (addr_slot_reg == 4'd7)  ? cache_data_rdata[255:224] :
                 (addr_slot_reg == 4'd8)  ? cache_data_rdata[287:256] :
                 (addr_slot_reg == 4'd9)  ? cache_data_rdata[319:288] :
                 (addr_slot_reg == 4'd10) ? cache_data_rdata[351:320] :
                 (addr_slot_reg == 4'd11) ? cache_data_rdata[383:352] :
                 (addr_slot_reg == 4'd12) ? cache_data_rdata[415:384] :
                 (addr_slot_reg == 4'd13) ? cache_data_rdata[447:416] :
                 (addr_slot_reg == 4'd14) ? cache_data_rdata[479:448] : cache_data_rdata[511:480] ;

          wire [31:0] miss_resp_data;
          assign miss_resp_data =  (addr_slot_reg == 4'd0)  ? cache_data_wdata[31:0] :
                 (addr_slot_reg == 4'd1)  ? cache_data_wdata[63:32] :
                 (addr_slot_reg == 4'd2)  ? cache_data_wdata[95:64] :
                 (addr_slot_reg == 4'd3)  ? cache_data_wdata[127:96] :
                 (addr_slot_reg == 4'd4)  ? cache_data_wdata[159:128] :
                 (addr_slot_reg == 4'd5)  ? cache_data_wdata[191:160] :
                 (addr_slot_reg == 4'd6)  ? cache_data_wdata[223:192] :
                 (addr_slot_reg == 4'd7)  ? cache_data_wdata[255:224] :
                 (addr_slot_reg == 4'd8)  ? cache_data_wdata[287:256] :
                 (addr_slot_reg == 4'd9)  ? cache_data_wdata[319:288] :
                 (addr_slot_reg == 4'd10) ? cache_data_wdata[351:320] :
                 (addr_slot_reg == 4'd11) ? cache_data_wdata[383:352] :
                 (addr_slot_reg == 4'd12) ? cache_data_wdata[415:384] :
                 (addr_slot_reg == 4'd13) ? cache_data_wdata[447:416] :
                 (addr_slot_reg == 4'd14) ? cache_data_wdata[479:448] : cache_data_wdata[511:480] ;

          assign flush_wdata = (flush_slot_addr == 4'd0)  ? cache_data_rdata[31:0] :
                 (flush_slot_addr == 4'd1)  ? cache_data_rdata[63:32] :
                 (flush_slot_addr == 4'd2)  ? cache_data_rdata[95:64] :
                 (flush_slot_addr == 4'd3)  ? cache_data_rdata[127:96] :
                 (flush_slot_addr == 4'd4)  ? cache_data_rdata[159:128] :
                 (flush_slot_addr == 4'd5)  ? cache_data_rdata[191:160] :
                 (flush_slot_addr == 4'd6)  ? cache_data_rdata[223:192] :
                 (flush_slot_addr == 4'd7)  ? cache_data_rdata[255:224] :
                 (flush_slot_addr == 4'd8)  ? cache_data_rdata[287:256] :
                 (flush_slot_addr == 4'd9)  ? cache_data_rdata[319:288] :
                 (flush_slot_addr == 4'd10) ? cache_data_rdata[351:320] :
                 (flush_slot_addr == 4'd11) ? cache_data_rdata[383:352] :
                 (flush_slot_addr == 4'd12) ? cache_data_rdata[415:384] :
                 (flush_slot_addr == 4'd13) ? cache_data_rdata[447:416] :
                 (flush_slot_addr == 4'd14) ? cache_data_rdata[479:448] : cache_data_rdata[511:480];

          reg [2:0] state;
          reg [2:0] next_state;
          reg flush_reg;
          assign flush_done = (state == S_FLUSH_DONE) ? 1'b1 : 1'b0;

          reg [BLOCK_ADDR-1:0] flush_addr;
          reg [3:0] flush_slot_addr;

          parameter S_IDLE = 3'd0;
          parameter S_WRITE_BLOCK = 3'd1;
          parameter S_READ_BLOCK = 3'd2;
          parameter S_RESPONSE = 3'd3;
          parameter S_WAIT_MEMRESP = 3'd4;
          parameter S_FLUSH = 3'd5;
          parameter S_FLUSH_DONE = 3'd6;

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
                  next_state = hit ? (is_read ? S_IDLE : S_WAIT_MEMRESP) :
                    (is_dirty ? S_WRITE_BLOCK : S_READ_BLOCK);
                end
                else
                begin
                  // next_state = S_IDLE;
                  next_state = (flush_reg) ? S_FLUSH : S_IDLE;
                end
              end
              S_WRITE_BLOCK:
              begin
                if (cacheresp_val && (slot2mem_addr == 4'd15))
                begin
                  next_state = S_READ_BLOCK;
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
                if (flush_addr == 5'd31)
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

              memreq_msg_reg <= 0;
              cachereq_val_reg <= 1'b0;
              memreq_msg_reg <= 0;

              data_wen_p <= 1'b0;
              tag_wen_p <= 1'b0;
              memresp_val_reg <= 1'b0;

            end
            else
            begin
              memresp_val_reg <= memresp_val_reg; // hold by default
              case (state)
                S_IDLE:
                begin
                  data_wen_p <= 1'b0;
                  tag_wen_p  <= 1'b0;
                  if (memreq_val && memreq_rdy)
                  begin
                    case ({hit, memreq_msg_type})
                      // Read miss
                      2'b00:
                      begin
                        cachereq_val_reg <= 1'b1;
                        cachereq_msg_reg <= (is_dirty) ? {1'b1, {cache_tag_rdata, addr_index, 4'd0, 2'b00}, 2'b0, cache_data_rdata[31:0]}
                        : {1'b0, {addr_tag, addr_index, 4'd0, 2'b00}, 2'b0, 32'b0};

                        slot2mem_addr <= 4'd0;
                        mem2slot_addr <= 4'd0;
                        cachereq_val_reg <= 1'b1;
                        memreq_msg_reg <= memreq_msg;
                      end
                      // Write miss
                      2'b01:
                      begin
                        cachereq_val_reg <= 1'b1;
                        cachereq_msg_reg <= (is_dirty) ? {1'b1, {cache_tag_rdata, addr_index, 4'd0, 2'b00}, 2'b0, cache_data_rdata[31:0]}
                        : {1'b0, {addr_tag, addr_index, 4'd0, 2'b00}, 2'b0, 32'b0};
                        slot2mem_addr <= 4'd0;
                        mem2slot_addr <= 4'd0;
                        cachereq_val_reg <= 1'b1;

                        memreq_msg_reg <= memreq_msg;
                      end
                      // Read hit
                      2'b10:
                      begin
                        memresp_msg_reg <= {1'b0, 2'b0, hit_data};
                        memresp_val_reg <= 1'b1;
                        tag_wen_p <= 1'b0;
                        data_wen_p <= 1'b0;
                      end
                      // Write hit
                      2'b11:
                      begin
                        dirty[addr_index] <= 1;
                        data_wen_p <= 1'b1;
                        memresp_msg_reg <= {1'b1, 2'b0, 32'd0};
                        memresp_val_reg <= 1'b1;
                        tag_wen_p <= 1'b0;
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
                  tag_wen_p <= 1'b0;
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
                      cachereq_val_reg <= 1'b1;
                      cachereq_msg_reg <= (slot2mem_addr == 4'd15) ? {1'b0, {addr_tag_reg, addr_index_reg, 4'd0, 2'b00}, 2'b0, 32'b0}
                      : {1'b1, {cache_tag_rdata, addr_index_reg, slot2mem_addr_plus1, 2'b00}, 2'b0, slot2mem_wdata};
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
                    data_wen_p <= 1'b0;
                    tag_wen_p <= 1'b0;
                  end
                  else
                  begin
                    if (cacheresp_val)
                    begin
                      cachereq_val_reg <= (mem2slot_addr == 4'd15) ? 1'b0 : 1'b1;
                      cachereq_msg_reg <= {1'b0, {addr_tag_reg, addr_index_reg, mem2slot_addr_plus1, 2'b00}, 2'b0, 32'd0};
                      mem2slot_addr <= (mem2slot_addr == 4'd15) ? 4'd0 : mem2slot_addr + 1;

                      if (mem2slot_addr == 4'd15)
                      begin
                        valid[addr_index_reg] <= 1'b1;
                        if (~memreq_msg_type_reg)
                          dirty[addr_index_reg] <= 1'b0;
                      end
                      cache_tag_wdata <= (mem2slot_addr == 4'd15) ? addr_tag_reg : cache_tag_rdata;

                      data_wen_p <= 1'b1;
                      slot_addr_reg <= mem2slot_addr;
                      tag_wen_p <= (mem2slot_addr == 4'd15) ? 1'b1 : 1'b0;
                      cache_tag_wdata <= (mem2slot_addr == 4'd15) ? addr_tag_reg : cache_tag_rdata;
                    end
                    else
                    begin
                      data_wen_p <= 1'b0;
                      tag_wen_p <= 1'b0;
                      cachereq_val_reg <= 1'b0;
                    end
                  end
                end
                S_RESPONSE:
                begin
                  if (memreq_msg_type_reg == 1'b0)
                  begin
                    memresp_msg_reg <= {1'b0, 2'b0, miss_resp_data};
                    data_wen_p <= 1'b0;
                    tag_wen_p <= 1'b0;
                  end
                  else
                  begin
                    memresp_msg_reg <= {1'b1, 2'b0, 32'b0};
                    dirty[addr_index_reg] <= 1'b1;
                    data_wen_p <= 1'b1;
                    tag_wen_p <= 1'b0;
                  end
                  memresp_val_reg <= 1'b1;
                end
                S_WAIT_MEMRESP:
                begin
                  data_wen_p <= 1'b0;
                  tag_wen_p <= 1'b0;
                  if (memresp_rdy)
                    memresp_val_reg <= 1'b0;
                end
                default:
                begin
                  slot2mem_addr <= 4'd0;
                  mem2slot_addr <= 4'd0;
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
                        cachereq_msg_reg <= {1'b1, {cache_tag_rdata, flush_addr[4:0], flush_slot_addr, 2'b00}, 2'b0, flush_wdata};

                        flush_slot_addr <= (flush_slot_addr == 4'd15) ? 4'd0 : flush_slot_addr + 1;
                        flush_addr <= (flush_slot_addr == 4'd15) ? flush_addr + 1 : flush_addr;
                      end
                      else if (flush_slot_addr == 4'd0)
                      begin
                        cachereq_val_reg <= 1'b1;
                        cachereq_msg_reg <= {1'b1, {cache_tag_rdata, flush_addr[4:0], flush_slot_addr, 2'b00}, 2'b0, flush_wdata};
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
                    flush_addr <= (flush_addr == 5'd31) ? 0 : flush_addr + 1;
                    flush_slot_addr <= 4'd0;
                    cachereq_val_reg <= 1'b0;
                  end
                end
                S_FLUSH_DONE:
                begin
                  cachereq_val_reg <= 1'b0;
                  dirty <= 0;
                end
              endcase
            end
          end

        endmodule


`endif  /* RISCV_CACHE_BASE_V */
