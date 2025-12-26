//========================================================================
// Lab 1 - Booth Iterative Mul Unit (Radix-4, 17-cycle)
//========================================================================

`ifndef RISCV_INT_MUL_BOOTH_V
`define RISCV_INT_MUL_BOOTH_V

        module imuldiv_IntMulBooth
          (
            input         clk,
            input         reset,

            input  [31:0] mulreq_msg_a,
            input  [31:0] mulreq_msg_b,
            input         mulreq_val,
            output        mulreq_rdy,

            output [63:0] mulresp_msg_result,
            output        mulresp_val,
            input         mulresp_rdy
          );

          // Ctrl <-> Dpath
          wire        dpath_ld_operands;
          wire        dpath_clr_result;
          wire        dpath_shift_en;
          wire        dpath_add_en;
          wire  [2:0] dpath_add_op;
          wire  [2:0] dpath_b_low3;
          wire        dpath_latch_result;

          imuldiv_IntMulBoothCtrl ctrl (
                                    .clk          (clk),
                                    .reset        (reset),
                                    .mulreq_val   (mulreq_val),
                                    .mulreq_rdy   (mulreq_rdy),
                                    .mulresp_val  (mulresp_val),
                                    .mulresp_rdy  (mulresp_rdy),
                                    .b_low3       (dpath_b_low3),
                                    .ld_operands  (dpath_ld_operands),
                                    .clr_result   (dpath_clr_result),
                                    .shift_en     (dpath_shift_en),
                                    .add_en       (dpath_add_en),
                                    .add_op       (dpath_add_op),
                                    .latch_result (dpath_latch_result)
                                  );

          imuldiv_IntMulBoothDpath dpath (
                                     .clk                (clk),
                                     .reset              (reset),
                                     .mulreq_msg_a       (mulreq_msg_a),
                                     .mulreq_msg_b       (mulreq_msg_b),
                                     .ld_operands        (dpath_ld_operands),
                                     .clr_result         (dpath_clr_result),
                                     .shift_en           (dpath_shift_en),
                                     .add_en             (dpath_add_en),
                                     .add_op             (dpath_add_op),
                                     .latch_result       (dpath_latch_result),
                                     .b_low3             (dpath_b_low3),
                                     .mulresp_msg_result (mulresp_msg_result)
                                   );

        endmodule

        //------------------------------------------------------------------------
        // Datapath (Radix-4 Booth) — A 用有號擴展，B 用原始值+0 並做算術右移
        //------------------------------------------------------------------------
        module imuldiv_IntMulBoothDpath
          (
            input         clk,
            input         reset,

            input  [31:0] mulreq_msg_a,
            input  [31:0] mulreq_msg_b,

            input         ld_operands,
            input         clr_result,
            input         shift_en,
            input         add_en,
            input  [2:0]  add_op,
            input         latch_result,

            output [2:0]  b_low3,
            output [63:0] mulresp_msg_result
          );

          // 有號 A、B（Booth 直接處理正負，不做最後 sign 修正）
          reg  [63:0] A_reg;     // {{32{A[31]}}, A}
          reg  [32:0] B_reg;     // {B, 1'b0}
          reg  [63:0] R_reg;     // 累加（有號二補數運算 ok）
          reg  [63:0] out_reg;   // 最終結果鎖存（避免延後一筆）

          // 初始化用的組合線（直接用輸入，不要先寫暫存器再讀）
          wire [63:0] A_init = {{32{mulreq_msg_a[31]}}, mulreq_msg_a};
          wire [32:0] B_init = {mulreq_msg_b, 1'b0};

          // 供 Ctrl 觀察的 B[2:0]
          assign b_low3 = B_reg[2:0];

          // addend 選擇（對 A_reg 做 +/- / *2）
          wire [63:0] add_A  = A_reg;
          wire [63:0] add_2A = A_reg <<< 1;              // 算術左移與邏輯左移等價
          wire [63:0] sub_A  = (~A_reg) + 64'd1;
          wire [63:0] sub_2A = (~(A_reg <<< 1)) + 64'd1;

          reg  [63:0] addend;
          always @(*)
          begin
            case (add_op)
              3'b000:
                addend = 64'd0;     // NOP
              3'b001:
                addend = add_A;     // +A
              3'b010:
                addend = add_2A;    // +2A
              3'b011:
                addend = sub_A;     // -A
              3'b100:
                addend = sub_2A;    // -2A
              default:
                addend = 64'd0;
            endcase
          end

          // 當拍加完的下一值，供最後一輪鎖定
          wire [63:0] R_next = R_reg + addend;

          // 時序邏輯
          always @(posedge clk)
          begin
            if (reset)
            begin
              A_reg   <= 64'd0;
              B_reg   <= 33'd0;
              R_reg   <= 64'd0;
              out_reg <= 64'd0;
            end
            else
            begin
              if (ld_operands)
              begin
                A_reg <= A_init;          // A 有號擴展
                B_reg <= B_init;          // B 原值+0
              end
              if (clr_result)
              begin
                R_reg <= 64'd0;
              end
              if (add_en)
              begin
                R_reg <= R_next;
              end
              if (shift_en)
              begin
                A_reg <= A_reg <<< 2;                        // 左移兩位
                B_reg <= { {2{B_reg[32]}}, B_reg[32:2] };    // ★ 算術右移兩位（保符號）
              end
              if (latch_result)
              begin
                out_reg <= R_next;         // Booth 已處理正負，不需再簽位修正
              end
            end
          end

          assign mulresp_msg_result = out_reg;

        endmodule

        //------------------------------------------------------------------------
        // Control Logic（與你現有一致；保留 latch_result 在最後一輪同拍）
        //  FSM: IDLE -> CALC(16) -> DONE   => 1 + 16 = 17 cycles
        //------------------------------------------------------------------------
        module imuldiv_IntMulBoothCtrl
          (
            input        clk,
            input        reset,

            input        mulreq_val,
            output reg   mulreq_rdy,
            output reg   mulresp_val,
            input        mulresp_rdy,

            input  [2:0] b_low3,

            output reg   ld_operands,
            output reg   clr_result,
            output reg   shift_en,
            output reg   add_en,
            output reg [2:0] add_op,
            output reg   latch_result
          );

          localparam S_IDLE = 2'd0;
          localparam S_CALC = 2'd1;
          localparam S_DONE = 2'd2;

          reg [1:0] state, state_n;
          reg [4:0] cnt; // 16 iterations

          // Booth decode（對 B 視為有號三位窗）
          wire [2:0] booth_op =
               (b_low3 == 3'b000 || b_low3 == 3'b111) ? 3'b000 :
               (b_low3 == 3'b001 || b_low3 == 3'b010) ? 3'b001 :
               (b_low3 == 3'b011)                     ? 3'b010 :
               (b_low3 == 3'b100)                     ? 3'b100 :
               3'b011 ; // 101/110 => -A

          always @(*)
          begin
            ld_operands   = 1'b0;
            clr_result    = 1'b0;
            shift_en      = 1'b0;
            add_en        = 1'b0;
            add_op        = 3'b000;
            latch_result  = 1'b0;

            mulreq_rdy    = 1'b0;
            mulresp_val   = 1'b0;
            state_n       = state;

            case (state)
              S_IDLE:
              begin
                mulreq_rdy = 1'b1;
                if (mulreq_val)
                begin
                  ld_operands = 1'b1;
                  clr_result  = 1'b1;
                  state_n     = S_CALC;
                end
              end
              S_CALC:
              begin
                if (cnt != 0)
                begin
                  add_en   = 1'b1;
                  add_op   = booth_op;
                  shift_en = 1'b1;
                  if (cnt == 5'd1)
                  begin
                    latch_result = 1'b1;   // 最後一輪同拍鎖定
                    state_n      = S_DONE;
                  end
                end
                else
                begin
                  state_n = S_DONE;
                end
              end
              S_DONE:
              begin
                mulresp_val = 1'b1;
                if (mulresp_rdy)
                  state_n = S_IDLE;
              end
            endcase
          end

          always @(posedge clk)
          begin
            if (reset)
            begin
              state <= S_IDLE;
              cnt   <= 5'd0;
            end
            else
            begin
              state <= state_n;
              case (state)
                S_IDLE:
                  if (mulreq_val)
                    cnt <= 5'd16;
                S_CALC:
                  if (cnt != 0)
                    cnt <= cnt - 5'd1;
                default:
                  cnt <= 5'd0;
              endcase
            end
          end

        endmodule

`endif
