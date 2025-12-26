//====================================================================================
// Cache Base Design (Feel free to create your own design if you have a better one)
//====================================================================================

`ifndef RISCV_CACHE_BASE_V
`define RISCV_CACHE_BASE_V

`include "riscvbc-CacheMsg.v"
`include "vc-RAMs.v"

module riscv_CacheBase (
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

/* Uncomment the following code when you start working on the I-cache */

// wire       memreq_en;
// wire       tag_wen;
// wire       data_wen;
// wire       write_data_mux_sel;
// wire       miss;
// wire       refill_cnt_en;
// wire       refill_cnt_clr;

// wire [2:0] state;
// wire       type;
// wire       tag_match;
// wire       valid_bit;
// wire       dirty_bit;
// wire [3:0] refill_counter;

// riscv_CacheBaseDpath dpath
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

//     .type                  (type),
//     .tag_match             (tag_match),      
//     .valid_bit             (valid_bit),
//     .dirty_bit             (dirty_bit),        
//     .refill_counter        (refill_counter)     
// );

// riscv_CacheBaseCtrl ctrl
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
//     .tag_match             (tag_match),          
//     .valid_bit             (valid_bit),
//     .dirty_bit             (dirty_bit),      
//     .refill_counter        (refill_counter),        
      
//     .state                 (state),
//     .memreq_en             (memreq_en),
//     .tag_wen               (tag_wen),
//     .data_wen              (data_wen),    
//     .write_data_mux_sel    (write_data_mux_sel),
//     .miss                  (miss),    
//     .refill_cnt_en         (refill_cnt_en),    
//     .refill_cnt_clr        (refill_cnt_clr)
// );

// endmodule

// //------------------------------------------------------------------------
// // Datapath
// //------------------------------------------------------------------------

// module riscv_CacheBaseDpath (
//     input         clk,
//     input         reset,

//     // msg
//     input  [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] memreq_msg,
//     input  [`VC_MEM_RESP_MSG_SZ(32)-1:0]   cacheresp_msg,
//     output [`VC_MEM_RESP_MSG_SZ(32)-1:0]   memresp_msg,
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

//     output            type,                // read or write
//     output            tag_match,           // tag is match
//     output            valid_bit,           // cache block is valid
//     output            dirty_bit,           // cache block is dirty
//     output reg [3:0]  refill_counter       // refill counter 
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
// wire [`TAG_BITS-1:0] read_tag  = // TODO
// assign               valid_bit = // TODO
// assign               dirty_bit = // TODO

// // compare tag
// assign tag_match = ( tag == /* TODO */ );

// // output request type to control
// assign type = memreq_type_reg;

// // read hit
// reg [31:0] read_data_hit;
// always@ ( * ) begin
//     case ( memreq_len_reg )
//         2'b01: read_data_hit = // TODO
//         2'b10: read_data_hit = // TODO
//         2'b11: read_data_hit = // TODO
//         2'b00: read_data_hit = // TODO
//     endcase
// end

// // read miss
// reg [31:0] read_data_miss;
// always@ ( * ) begin
//     case ( memreq_len_reg )
//         2'b01: read_data_miss = // TODO
//         2'b10: read_data_miss = // TODO
//         2'b11: read_data_miss = // TODO
//         2'b00: read_data_miss = // TODO
//     endcase
// end

// // write hit
// reg [`BLK_SIZE-1:0] write_data_hit;
// always@ ( * ) begin
//     write_data_hit = read_data;
//     case ( memreq_len_reg )
//         2'b01: /* TODO */ = memreq_data_reg[7:0];  
//         2'b10: /* TODO */ = memreq_data_reg[15:0];   
//         2'b11: /* TODO */ = memreq_data_reg[23:0];  
//         2'b00: /* TODO */ = memreq_data_reg[31:0]; 
//     endcase
// end

// // write miss
// reg [`BLK_SIZE-1:0] write_data_miss;
// always@ ( * ) begin
//     write_data_miss = refill_data;
//     if ( memreq_type_reg == `WRITE ) begin
//         case ( memreq_len_reg )
//             2'b01: /* TODO */ = memreq_data_reg[7:0];  
//             2'b10: /* TODO */ = memreq_data_reg[15:0];   
//             2'b11: /* TODO */ = memreq_data_reg[23:0];  
//             2'b00: /* TODO */ = memreq_data_reg[31:0]; 
//         endcase
//     end
// end

// wire [`BLK_SIZE-1:0] write_data_mux_out = ( write_data_mux_sel ) ? write_data_miss : write_data_hit;

// // set write tagS
// wire [22:0] write_tagS = // TODO

// // refill address
// wire [31:0] refill_addr = // TODO;

// //------------------------------------------------------------------------
// // Sequential logic
// //------------------------------------------------------------------------

// // store data if missing
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

// // store data when missing
// reg [`BLK_SIZE-1:0] read_data_reg;
// reg [22:0]          read_tagS_reg;
// always@ ( posedge clk ) begin
//     if ( miss ) begin
//         read_data_reg <= // TODO
//         read_tagS_reg <= // TODO
//     end
// end

// // refill counter
// always@ ( posedge clk ) begin
//     if ( refill_cnt_clr ) begin
//         refill_counter <= 4'd0;
//     end 
//     else if ( refill_cnt_en ) begin
//         refill_counter <= refill_counter + 1;
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

// //------------------------------------------------------------------------
// // FSM output msg
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

//         // TODO

//     endcase
// end

// assign memresp_msg   = memresp_msg_reg;
// assign cachereq_msg  = cachereq_msg_reg;

// //------------------------------------------------------------------------
// // I-cache RAM module
// //------------------------------------------------------------------------

// wire [22:0]          read_tagS;
// wire [`BLK_SIZE-1:0] read_data;

// // direct-mapped
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

// endmodule

// //------------------------------------------------------------------------
// // Control Logic
// //------------------------------------------------------------------------

// module riscv_CacheBaseCtrl (
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
//     input          tag_match,           // tag is match
//     input          valid_bit,           // cache block is valid
//     input          dirty_bit,           // cache block is dirty
//     input   [3:0]  refill_counter,      // refill counter    

//     output  [2:0]  state,               // FSM state
//     output         memreq_en,           // memreq enable
//     output         tag_wen,             // tag cache write enable
//     output         data_wen,            // data cache write enable
//     output         write_data_mux_sel,  // write data is hit data or miss data
//     output         miss,                // cache miss
//     output         refill_cnt_en,       // refill counter enable
//     output         refill_cnt_clr       // refill counter clear
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

//         // TODO

//     endcase
// end

// assign memreq_rdy    = memreq_rdy_reg;
// assign memresp_val   = memresp_val_reg;
// assign cachereq_val  = cachereq_val_reg;
// assign cacheresp_rdy = cacheresp_rdy_reg;

endmodule

`endif  /* RISCV_CACHE_BASE_V */
