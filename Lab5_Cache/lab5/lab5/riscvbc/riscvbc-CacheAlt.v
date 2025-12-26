//====================================================================================
// Cache Alt Design (Feel free to create your own design if you have a better one)
//====================================================================================

`ifndef RISCV_CACHE_ALT_V
`define RISCV_CACHE_ALT_V

`include "riscvbc-CacheMsg.v"
`include "vc-RAMs.v"
`include "riscvbc-VictimCache.v"

module riscv_CacheAlt (
    input clk,
    input reset,

    input                                  memreq_val,
    output                                 memreq_rdy,
    input  [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] memreq_msg,

    output                                 memresp_val,
    input                                  memresp_rdy,
    output [`VC_MEM_RESP_MSG_SZ(32)-1:0]   memresp_msg,

    output                                 cachereq_val,
    input                                  cachereq_rdy,
    output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] cachereq_msg,

    input                                  cacheresp_val,
    output                                 cacheresp_rdy,
    input  [`VC_MEM_RESP_MSG_SZ(32)-1:0]   cacheresp_msg
);

/* Uncomment the following code when you start working on the D-cache */

// wire       memreq_en;
// wire       tag_wen;
// wire       data_wen;
// wire       write_data_mux_sel;
// wire       miss_en;
// wire       refill_cnt_en;
// wire       refill_cnt_clr;
// wire       wb_cnt_en;
// wire       wb_cnt_clr;
// wire       hit_way_mux_sel;

// wire [2:0] state;
// wire       type;
// wire       tag0_match;
// wire       tag1_match;
// wire       valid0_bit;
// wire       valid1_bit;
// wire       dirty0_bit;
// wire       dirty1_bit;
// wire       victim_way;

// wire [3:0] refill_counter;
// wire [3:0] wb_counter;


// riscv_CacheAltDpath dpath
// (
//     .clk                   (clk),
//     .reset                 (reset),
   
//     .memreq_msg            (memreq_msg),
//     .cacheresp_msg         (cacheresp_msg),
//     .memresp_msg           (memresp_msg),
//     .cachereq_msg          (cachereq_msg),    
   
//     .state                 (state),
//     .memreq_en             (memreq_en),
//     .tag_wen               (tag_wen),
//     .data_wen              (data_wen),    
//     .write_data_mux_sel    (write_data_mux_sel),
//     .miss                  (miss),
//     .refill_cnt_en         (refill_cnt_en),    
//     .refill_cnt_clr        (refill_cnt_clr),
//     .wb_cnt_en             (wb_cnt_en),
//     .wb_cnt_clr            (wb_cnt_clr),                   
//     .hit_way_mux_sel       (hit_way_mux_sel),

//     .type                  (type),
//     .tag0_match            (tag0_match), 
//     .tag1_match            (tag1_match),          
//     .valid0_bit            (valid0_bit),
//     .valid1_bit            (valid1_bit),    
//     .dirty0_bit            (dirty0_bit),
//     .dirty1_bit            (dirty1_bit),    
//     .victim_way            (victim_way),   
//     .refill_counter        (refill_counter),    
//     .wb_counter            (wb_counter)
// );

// riscv_CacheAltCtrl ctrl
// (
//     .clk                   (clk),
//     .reset                 (reset),
      
//     .memreq_val            (memreq_val),
//     .memreq_rdy            (memreq_rdy),  
//     .memresp_val           (memresp_val),
//     .memresp_rdy           (memresp_rdy),    
//     .cachereq_val          (cachereq_val),
//     .cachereq_rdy          (cachereq_rdy),
//     .cacheresp_val         (cacheresp_val),
//     .cacheresp_rdy         (cacheresp_rdy),
         
//     .type                  (type),
//     .tag0_match            (tag0_match), 
//     .tag1_match            (tag1_match),          
//     .valid0_bit            (valid0_bit),
//     .valid1_bit            (valid1_bit),    
//     .dirty0_bit            (dirty0_bit),
//     .dirty1_bit            (dirty1_bit),   
//     .victim_way            (victim_way),           
//     .refill_counter        (refill_counter),      
//     .wb_counter            (wb_counter),    
   
//     .state                 (state),
//     .memreq_en             (memreq_en),
//     .tag_wen               (tag_wen),
//     .data_wen              (data_wen), 
//     .write_data_mux_sel    (write_data_mux_sel),
//     .miss                  (miss),    
//     .refill_cnt_en         (refill_cnt_en),    
//     .refill_cnt_clr        (refill_cnt_clr),
//     .wb_cnt_en             (wb_cnt_en),
//     .wb_cnt_clr            (wb_cnt_clr),     
//     .hit_way_mux_sel       (hit_way_mux_sel)
// );

// endmodule

// //------------------------------------------------------------------------
// // Datapath
// //------------------------------------------------------------------------

// module riscv_CacheAltDpath (
//     input         clk,
//     input         reset,

//     // msg
//     input  [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] memreq_msg,
//     input    [`VC_MEM_RESP_MSG_SZ(32)-1:0] cacheresp_msg,
//     output   [`VC_MEM_RESP_MSG_SZ(32)-1:0] memresp_msg,
//     output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] cachereq_msg,

//     // control singal
//     input      [2:0]  state,               // FSM state
//     input             memreq_en,           // memreq enable
//     input             tag_wen,             // tag cache write enable
//     input             data_wen,            // data cache write enable
//     input             write_data_mux_sel,  // write data is hit data or miss data
//     input             miss,                // cache miss
//     input             refill_cnt_en,       // refill counter enable
//     input             refill_cnt_clr,      // refill counter clear
//     input             wb_cnt_en,           // write back counter enable  
//     input             wb_cnt_clr,          // write back counter clear  
//     input             hit_way_mux_sel,     // which way is hit          
    
//     output            type,                // read or write
//     output            tag0_match,          // way0 tag is match
//     output            tag1_match,          // way1 tag is match
//     output            valid0_bit,          // way0 is valid
//     output            valid1_bit,          // way1 is valid
//     output            dirty0_bit,          // way0 is dirty
//     output            dirty1_bit,          // way1 is dirty
//     output            victim_way,          // which way is victim in main cache
//     output reg [3:0]  refill_counter,      // refill counter 
//     output reg [3:0]  wb_counter           // write back counter
// );

// //------------------------------------------------------------------------
// // Combinational logic
// //------------------------------------------------------------------------

// // message
// wire        memreq_type = // TODO
// wire [31:0] memreq_addr = // TODO   
// wire [1:0]  memreq_len  = // TODO   
// wire [31:0] memreq_data = // TODO 

// wire [31:0] cacheresp_data = cacheresp_msg[31:0];

// // address
// wire [`OFF_BITS-1:0] offset = // TODO
// wire [`IDX_BITS-1:0] index  = // TODO
// wire [`TAG_BITS-1:0] tag    = // TODO

// // extract tag and status bits
// wire [22:0] read_tagS0 = // TODO
// wire [22:0] read_tagS1 = // TODO

// // read way0
// wire [`TAG_BITS-1:0] read_tag0  = // TODO
// assign               valid0_bit = // TODO
// assign               dirty0_bit = // TODO

// // read way1
// wire [`TAG_BITS-1:0] read_tag1  = // TODO
// assign               valid1_bit = // TODO
// assign               dirty1_bit = // TODO

// assign victim_way = // TODO

// // 2-way data
// wire [`BLK_SIZE-1:0] read_data0 = read_data[511:0];
// wire [`BLK_SIZE-1:0] read_data1 = read_data[1023:512];

// // compare tag
// assign tag0_match    = ( tag == /* TODO */ );
// assign tag1_match    = ( tag == /* TODO */ );

// // select victim block in main cache (data/tag)
// wire [`BLK_SIZE-1:0] read_data_vc  = ( victim_way ) ? read_data1 : read_data0;
// wire [22:0]          read_tagS_vc  = ( victim_way ) ? read_tagS1 : read_tagS0;

// // select survivor block in main cache (data/tag)
// wire [`BLK_SIZE-1:0] read_data_surv = ( victim_way ) ? read_data0 : read_data1;
// wire [22:0]          read_tagS_surv = ( victim_way ) ? read_tagS0 : read_tagS1;

// // output request type to control
// assign type = memreq_type_reg;

// // select hit block
// wire [`BLK_SIZE-1:0] hit_block_data = ( hit_way_mux_sel ) ? read_data1 : read_data0;

// // read hit
// reg [31:0] read_data_hit;
// always@ (*) begin
//     case ( memreq_len_reg )
//         2'b01: read_data_hit = // TODO
//         2'b10: read_data_hit = // TODO
//         2'b11: read_data_hit = // TODO
//         2'b00: read_data_hit = // TODO
//     endcase
// end

// // read miss
// wire [`BLK_SIZE-1:0] read_data_mux_out = ( victim_way_reg ) ? read_data1 : read_data0;
// reg  [31:0]          read_data_miss;
// always@ (*) begin
//     case ( memreq_len_reg )
//         2'b01: read_data_miss = // TODO
//         2'b10: read_data_miss = // TODO
//         2'b11: read_data_miss = // TODO
//         2'b00: read_data_miss = // TODO
//     endcase
// end

// // write hit
// reg [`BLK_SIZE-1:0] write_data_hit;
// always@ (*) begin
//     write_data_hit = hit_block_data;
//     case ( memreq_len_reg )
//         2'b01: /* TODO */ = memreq_data_reg[7:0];  
//         2'b10: /* TODO */ = memreq_data_reg[15:0];   
//         2'b11: /* TODO */ = memreq_data_reg[23:0];  
//         2'b00: /* TODO */ = memreq_data_reg[31:0]; 
//     endcase
// end
// wire [`D_SET_SIZE-1:0] write_data_hit_o = ( hit_way_mux_sel ) 
//                                         ? {write_data_hit, read_data0} : {read_data1, write_data_hit};

// // write miss
// reg [`BLK_SIZE-1:0] write_data_miss;
// always@ (*) begin
//     write_data_miss = refill_data;
//     if ( memreq_type_reg == WRITE ) begin
//         case ( memreq_len_reg )
//             2'b01: /* TODO */ = memreq_data_reg[7:0];  
//             2'b10: /* TODO */ = memreq_data_reg[15:0];   
//             2'b11: /* TODO */ = memreq_data_reg[23:0];  
//             2'b00: /* TODO */ = memreq_data_reg[31:0]; 
//         endcase
//     end
// end
// wire [`D_SET_SIZE-1:0] write_data_miss_o = // TODO

// // select write data: hit or miss
// wire [`D_SET_SIZE-1:0] write_data_mux_out = ( write_data_mux_sel ) ? write_data_miss_o : write_data_hit_o;

// // set write tagS
// wire [46:0] write_tagS = // TODO

// // refill address
// wire [31:0] refill_addr = // TODO

// // write back address and data
// wire [31:0] wb_addr  = // TODO
// wire [31:0] wb_data  = // TODO

// //------------------------------------------------------------------------
// // Sequential logic
// //------------------------------------------------------------------------

// // store memreq massage
// reg        memreq_type_reg;
// reg [31:0] memreq_addr_reg;
// reg [1:0]  memreq_len_reg; 
// reg [31:0] memreq_data_reg;

// always@ ( posedge clk ) begin
//     if ( memreq_en ) begin
//         memreq_type_reg <= memreq_type;
//         memreq_addr_reg <= memreq_addr;
//         memreq_len_reg  <= memreq_len;
//         memreq_data_reg <= memreq_data;
//     end
// end

// // store data if missing
// reg [`BLK_SIZE-1:0] read_data_vc_reg;
// reg [`BLK_SIZE-1:0] read_data_surv_reg;
// reg                 victim_way_reg;
// reg [22:0]          read_tagS_vc_reg;
// reg [22:0]          read_tagS_surv_reg;
// always@ ( posedge clk ) begin
//     if ( miss ) begin
//         read_data_vc_reg   <= // TODO
//         read_data_surv_reg <= // TODO
//         victim_way_reg     <= // TODO
//         read_tagS_vc_reg   <= // TODO
//         read_tagS_surv_reg <= // TODO
//     end
// end

// // refill counter 
// always@ ( posedge clk ) begin
//     if ( refill_cnt_clr ) begin
//         refill_counter <= 4'd0;
//     end 
//     else if ( refill_cnt_en ) begin
//         refill_counter <= refill_counter + 1 ;
//     end
// end

// // refill data
// reg [`BLK_SIZE-1:0] refill_data;
// always@ ( posedge clk ) begin
//     if ( reset ) begin
//         refill_data <= 512'b0;
//     end 
//     else if ( refill_cnt_en ) begin
//         // TODO
//     end
// end

// // write back counter 
// always@ ( posedge clk ) begin
//     if ( wb_cnt_clr ) begin
//         wb_counter <= 4'd0;
//     end 
//     else if ( wb_cnt_en ) begin
//         wb_counter <= wb_counter + 1 ;
//     end
// end

// //------------------------------------------------------------------------
// // FSM output massage
// //------------------------------------------------------------------------

// localparam IDLE           = 3'b000;
// localparam READ_CACHE     = 3'b001;
// localparam UPDATE_CACHE   = 3'b010;
// localparam READ_MEM_REQ   = 3'b011;
// localparam READ_MEM_RESP  = 3'b100;
// localparam DONE           = 3'b101;

// reg [34:0] memresp_msg_reg; 
// reg [66:0] cachereq_msg_reg;

// always@ (*) begin

//     memresp_msg_reg   = 35'b0;
//     cachereq_msg_reg  = 67'b0;

//     case ( state )
//         IDLE: begin
//             memresp_msg_reg   = 35'b0;
//             cachereq_msg_reg  = 67'b0;        
//         end

//     // TODO      

//     endcase
// end

// assign memresp_msg   = memresp_msg_reg;
// assign cachereq_msg  = cachereq_msg_reg;

// //------------------------------------------------------------------------
// // D-cache RAM module
// //------------------------------------------------------------------------

// wire [46:0]            read_tagS;
// wire [`D_SET_SIZE-1:0] read_data;

// // 2-way assocative
// vc_RAM_rst_1w1r_pf #(
//     .DATA_SZ     (),
//     .ENTRIES     (),
//     .ADDR_SZ     (),
//     .RESET_VALUE ()
// ) _data (
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
// ) _tag (
//     .clk         (),
//     .reset_p     (),
//     .raddr       (),
//     .rdata       (),
//     .wen_p       (),
//     .waddr_p     (),
//     .wdata_p     ()
// );

// //------------------------------------------------------------------------
// // Part 3: Victim Cache [TODO]
// //------------------------------------------------------------------------

// // Instance riscv_VictimCache in riscv-VictimCache.v

// endmodule


// module riscv_CacheAltCtrl (
//     input         clk,
//     input         reset,

//     // CPU and memory interface
//     input          memreq_val,          
//     output         memreq_rdy,          
//     output         memresp_val,         
//     input          memresp_rdy,         
//     output         cachereq_val,        
//     input          cachereq_rdy,        
//     input          cacheresp_val,       
//     output         cacheresp_rdy,
   
//     input          type,                // read or write
//     input          tag0_match,          // way0 tag is match
//     input          tag1_match,          // way1 tag is match
//     input          valid0_bit,          // way0 is valid
//     input          valid1_bit,          // way1 is valid
//     input          dirty0_bit,          // way0 is dirty
//     input          dirty1_bit,          // way1 is dirty
//     input          victim_way,          // which way is victim in main cache
//     input  [3:0]   refill_counter,      // refill counter
//     input  [3:0]   wb_counter,          // write back counter


//     output [2:0]   state,               // FSM state
//     output         memreq_en,           // memreq enable
//     output         tag_wen,             // tag cache write enable
//     output         data_wen,            // data cache write enable
//     output         write_data_mux_sel,  // write data is hit data or miss data
//     output         miss,                // cache miss
//     output         refill_cnt_en,       // refill counter enable
//     output         refill_cnt_clr,      // refill counter clear
//     output         wb_cnt_en,           // write back counter enable 
//     output         wb_cnt_clr,          // write back counter clear 
//     output         hit_way_mux_sel      // which way is hit          
// );

// // val/rdy enable
// assign memreq_en = memreq_val && memreq_rdy;

// // TODO

// //------------------------------------------------------------------------
// // FSM
// //------------------------------------------------------------------------

// localparam IDLE           = 3'b000;
// localparam READ_CACHE     = 3'b001;
// localparam UPDATE_CACHE   = 3'b010;
// localparam READ_MEM_REQ   = 3'b011;
// localparam READ_MEM_RESP  = 3'b100;
// localparam DONE           = 3'b101;

// reg [2:0] curr_state, next_state;

// always@ ( posedge clk ) begin
//     curr_state <= next_state;
// end

// assign state = curr_state;

// // transition logic
// always@ (*) begin
//     case ( curr_state )
//         IDLE: begin
//             next_state = ( memreq_val ) ? READ_CACHE : IDLE;
//         end

//     // TODO

//     endcase
// end

// reg memreq_rdy_reg;   
// reg memresp_val_reg;  
// reg cachereq_val_reg; 
// reg cacheresp_rdy_reg;

// // val/rdy output
// always@ (*) begin

//     memreq_rdy_reg    = 1'b0;
//     memresp_val_reg   = 1'b0;
//     cachereq_val_reg  = 1'b0;
//     cacheresp_rdy_reg = 1'b0;

//     case ( curr_state )
//         IDLE: begin
//             memreq_rdy_reg    = 1'b1;
//             memresp_val_reg   = 1'b0;
//             cachereq_val_reg  = 1'b0;
//             cacheresp_rdy_reg = 1'b0;          
//         end

//     // TODO

//     endcase
// end

// assign memreq_rdy    = memreq_rdy_reg;
// assign memresp_val   = memresp_val_reg;
// assign cachereq_val  = cachereq_val_reg;
// assign cacheresp_rdy = cacheresp_rdy_reg;

endmodule

`endif  /* RISCV_CACHE_ALT_V */
