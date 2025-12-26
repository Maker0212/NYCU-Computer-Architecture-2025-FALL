//========================================================================
// Lab 1 - Iterative Mul Unit (UNSIGNED 32x32 -> 64)
//   - 保留老師骨架（Top + Dpath + Ctrl）
//   - 改成純無號乘法，外層 wrapper 負責 MULH/MULHSU 的有號處理
//========================================================================

`ifndef RISCV_INT_MUL_ITERATIVE_V
`define RISCV_INT_MUL_ITERATIVE_V

        module imuldiv_IntMulIterative
          (
            input                clk,
            input                reset,

            input  [31:0] mulreq_msg_a,
            input  [31:0] mulreq_msg_b,
            input         mulreq_val,
            output        mulreq_rdy,

            output [63:0] mulresp_msg_result,
            output        mulresp_val,
            input         mulresp_rdy
          );

          wire    [4:0] counter;
          wire          sign;
          wire          b_lsb;
          wire          sign_en;
          wire          result_en;
          wire          cntr_mux_sel;
          wire          a_mux_sel;
          wire          b_mux_sel;
          wire          result_mux_sel;
          wire          add_mux_sel;
          wire          sign_mux_sel;

          imuldiv_IntMulIterativeDpath dpath
                                       (
                                         .clk                (clk),
                                         .reset              (reset),
                                         .mulreq_msg_a       (mulreq_msg_a),
                                         .mulreq_msg_b       (mulreq_msg_b),
                                         .mulresp_msg_result (mulresp_msg_result),
                                         .counter            (counter),
                                         .sign               (sign),
                                         .b_lsb              (b_lsb),
                                         .sign_en            (sign_en),
                                         .result_en          (result_en),
                                         .cntr_mux_sel       (cntr_mux_sel),
                                         .a_mux_sel          (a_mux_sel),
                                         .b_mux_sel          (b_mux_sel),
                                         .result_mux_sel     (result_mux_sel),
                                         .add_mux_sel        (add_mux_sel),
                                         .sign_mux_sel       (sign_mux_sel)
                                       );

          imuldiv_IntMulIterativeCtrl ctrl
                                      (
                                        .clk            (clk),
                                        .reset          (reset),
                                        .mulreq_val     (mulreq_val),
                                        .mulreq_rdy     (mulreq_rdy),
                                        .mulresp_val    (mulresp_val),
                                        .mulresp_rdy    (mulresp_rdy),
                                        .counter        (counter),
                                        .sign           (sign),
                                        .b_lsb          (b_lsb),
                                        .sign_en        (sign_en),
                                        .result_en      (result_en),
                                        .cntr_mux_sel   (cntr_mux_sel),
                                        .a_mux_sel      (a_mux_sel),
                                        .b_mux_sel      (b_mux_sel),
                                        .result_mux_sel (result_mux_sel),
                                        .add_mux_sel    (add_mux_sel),
                                        .sign_mux_sel   (sign_mux_sel)
                                      );

        endmodule

        //------------------------------------------------------------------------
        // Datapath
        //------------------------------------------------------------------------

        module imuldiv_IntMulIterativeDpath
          (
            input                clk,
            input                reset,

            // Operands and Result
            input  [31:0] mulreq_msg_a,
            input  [31:0] mulreq_msg_b,
            output [63:0] mulresp_msg_result,

            // Datapath Outputs
            output  [4:0] counter,
            output        sign,
            output        b_lsb,

            // Control Inputs
            input         sign_en,
            input         result_en,
            input         cntr_mux_sel,
            input         a_mux_sel,
            input         b_mux_sel,
            input         result_mux_sel,
            input         add_mux_sel,
            input         sign_mux_sel
          );

          //----------------------------------------------------------------------
          // Control Definitions
          //----------------------------------------------------------------------

          localparam op_x     = 1'dx;
          localparam op_load  = 1'd0;
          localparam op_next  = 1'd1;

          localparam add_x    = 1'dx;
          localparam add_old  = 1'd0;
          localparam add_next = 1'd1;

          localparam sign_x   = 1'dx;
          localparam sign_u   = 1'd0;
          localparam sign_s   = 1'd1;

          //----------------------------------------------------------------------
          // Pre-Flop Combinational Logic
          //----------------------------------------------------------------------

          // Counter Mux (32 次循環：從 31 數到 0)
          wire [4:0] counter_mux_out
               = ( cntr_mux_sel == op_load ) ? 5'd31
               : ( cntr_mux_sel == op_next ) ? counter_reg - 1'b1
               :                               5'bx;

          assign counter = counter_reg;

          // Sign of Result（保留，但最終不使用，交由外層處理 MULH/MULHSU）
          wire   sign_next = mulreq_msg_a[31] ^ mulreq_msg_b[31];
          assign sign      = sign_reg;

          // Unsigned Operands（關鍵改動：不再取絕對值，直接無號）
          wire [31:0] unsigned_a = mulreq_msg_a;
          wire [31:0] unsigned_b = mulreq_msg_b;

          // Operand Muxes
          wire [63:0] a_mux_out
               = ( a_mux_sel == op_load ) ? { 32'b0, unsigned_a }
               : ( a_mux_sel == op_next ) ? a_shift_out
               :                            64'bx;

          wire [31:0] b_mux_out
               = ( b_mux_sel == op_load ) ? unsigned_b
               : ( b_mux_sel == op_next ) ? b_shift_out
               :                            32'bx;

          //----------------------------------------------------------------------
          // Sequential Logic
          //----------------------------------------------------------------------

          reg  [4:0] counter_reg;
          reg        sign_reg;
          reg [63:0] a_reg;
          reg [31:0] b_reg;
          reg [63:0] result_reg;

          always @ ( posedge clk )
          begin
            if ( sign_en )
            begin
              sign_reg   <= sign_next;
            end

            if ( result_en )
            begin
              result_reg <= result_mux_out;
            end

            counter_reg  <= counter_mux_out;
            a_reg        <= a_mux_out;
            b_reg        <= b_mux_out;
          end

          //----------------------------------------------------------------------
          // Post-Flop Combinational Logic
          //----------------------------------------------------------------------

          // Least Significant Bit of Operand B
          assign b_lsb = b_reg[0];

          // Operand Shifters
          wire [63:0] a_shift_out = a_reg << 1;
          wire [31:0] b_shift_out = b_reg >> 1;

          // Adder（條件累加：若 b_lsb=1 則加上 a_reg）
          wire [63:0] add_out = result_reg + a_reg;

          wire [63:0] add_mux_out
               = ( add_mux_sel == add_old )  ? result_reg
               : ( add_mux_sel == add_next ) ? add_out
               :                               64'bx;

          // Result Mux（啟動時清 0，之後每拍更新）
          wire [63:0] result_mux_out
               = ( result_mux_sel == op_load ) ? 64'b0
               : ( result_mux_sel == op_next ) ? add_mux_out
               :                                 64'bx;

          // Signed Result Mux（關鍵改動：固定走無號）
          wire [63:0] signed_result_mux_out
               = ( sign_mux_sel == sign_u ) ? result_reg
               : ( sign_mux_sel == sign_s ) ? ~result_reg + 1'b1
               :                              64'bx;

          // Final Result（關鍵改動：直接輸出 result_reg 即可）
          // assign mulresp_msg_result = signed_result_mux_out;
          assign mulresp_msg_result = result_reg;

        endmodule

        //------------------------------------------------------------------------
        // Control Logic
        //------------------------------------------------------------------------

        module imuldiv_IntMulIterativeCtrl
          (
            input        clk,
            input        reset,

            // Request val/rdy
            input        mulreq_val,
            output       mulreq_rdy,

            // Response val/rdy
            output       mulresp_val,
            input        mulresp_rdy,

            // Datapath Inputs
            input  [4:0] counter,
            input        sign,
            input        b_lsb,

            // Control Outputs
            output       sign_en,
            output       result_en,
            output       cntr_mux_sel,
            output       a_mux_sel,
            output       b_mux_sel,
            output       result_mux_sel,
            output       add_mux_sel,
            output       sign_mux_sel
          );

          //----------------------------------------------------------------------
          // State Definitions
          //----------------------------------------------------------------------

          localparam STATE_IDLE = 2'd0;
          localparam STATE_CALC = 2'd1;
          localparam STATE_SIGN = 2'd2; // 保留一拍供回應（與骨架一致）

          //----------------------------------------------------------------------
          // State Update
          //----------------------------------------------------------------------

          reg [1:0] state_reg;

          always @ ( posedge clk )
          begin
            if ( reset )
            begin
              state_reg <= STATE_IDLE;
            end
            else
            begin
              state_reg <= state_next;
            end
          end

          //----------------------------------------------------------------------
          // State Transitions
          //----------------------------------------------------------------------

          reg [1:0] state_next;

          always @ ( * )
          begin
            state_next = state_reg;
            case ( state_reg )
              STATE_IDLE:
                if ( mulreq_go )
                  state_next = STATE_CALC;
              STATE_CALC:
                if ( is_calc_done )
                  state_next = STATE_SIGN;
              STATE_SIGN:
                if ( mulresp_go )
                  state_next = STATE_IDLE;
            endcase
          end

          //----------------------------------------------------------------------
          // Control Definitions
          //----------------------------------------------------------------------

          localparam n = 1'd0;
          localparam y = 1'd1;

          localparam op_x    = 1'dx;
          localparam op_load = 1'd0;
          localparam op_next = 1'd1;

          //----------------------------------------------------------------------
          // Output Control Signals
          //----------------------------------------------------------------------

          localparam cs_size = 8;
          reg [cs_size-1:0] cs;

          //                 mulreq mulresp sign result cntr,    a        b        result
          //                 rdy    val     en   en     mux_sel, mux_sel  mux_sel  mux_sel
          always @ ( * )
          begin
            case ( state_reg )
              STATE_IDLE:
                cs = { y,     n,      y,   y,     op_load, op_load, op_load, op_load };
              STATE_CALC:
                cs = { n,     n,      n,   y,     op_next, op_next, op_next, op_next };
              STATE_SIGN:
                cs = { n,     y,      n,   n,     op_x,    op_x,    op_x,    op_x    };
            endcase
          end

          // Signal Parsing
          assign mulreq_rdy     = cs[7];
          assign mulresp_val    = cs[6];
          assign sign_en        = cs[5];
          assign result_en      = cs[4];
          assign cntr_mux_sel   = cs[3];
          assign a_mux_sel      = cs[2];
          assign b_mux_sel      = cs[1];
          assign result_mux_sel = cs[0];

          // 條件加法：當 b 的 LSB 為 1 時才加上 a_reg
          assign add_mux_sel    = b_lsb;

          // 關鍵改動：固定無號路徑（外層 wrapper 負責有號）
          // 原：assign sign_mux_sel = sign;
          assign sign_mux_sel   = 1'b0; // sign_u

          // Transition Triggers
          wire mulreq_go     = mulreq_val && mulreq_rdy;
          wire mulresp_go    = mulresp_val && mulresp_rdy;
          wire is_calc_done  = ( counter == 5'b0 );

        endmodule

`endif
