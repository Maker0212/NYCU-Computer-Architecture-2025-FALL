//========================================================================
// Lab 1 - Iterative Div Unit (Three-state, exact 33 cycles)
//========================================================================

`ifndef RISCV_INT_DIV_ITERATIVE_V
`define RISCV_INT_DIV_ITERATIVE_V

`include "imuldiv-DivReqMsg.v"

        module imuldiv_IntDivIterative
          (
            input         clk,
            input         reset,

            // 外部請求/回覆介面（照老師給的）
            input         divreq_msg_fn,       // 0: unsigned (divu/remu), 1: signed (div/rem)
            input  [31:0] divreq_msg_a,        // dividend
            input  [31:0] divreq_msg_b,        // divisor
            input         divreq_val,
            output        divreq_rdy,

            output [63:0] divresp_msg_result,  // {remainder, quotient}
            output        divresp_val,
            input         divresp_rdy
          );

          // --------------------------------------------------------------------
          // ctrl <-> dpath 之間的內部連線 (wires)
          // --------------------------------------------------------------------

          // 1) 由 ctrl 驅動 dpath 的控制訊號
          wire        c_ld_inputs;    // S_IDLE: 允許 dpath 把外部 a/b/fn 先鎖進內部輸入暫存
          wire        c_ld_init;      // S_IDLE 成交同拍：載入 A/B/iter_cnt 等初值（併入介面開銷）
          wire        c_step;         // S_CALC: 進行一次「shift/sub/寫入商位」單步
          wire        c_pack;         // S_CALC 最後一步同拍：把 {R,Q} 打包到輸出暫存
          wire        c_signcorr_en;  // S_CALC 最後一步同拍：啟用符號校正（僅 signed 指令）
          wire        c_signed_op;    // 當前操作是否 signed（由 ctrl 解碼）

          // 2) 由 dpath 回報給 ctrl 的狀態/旗標
          wire        d_iter_done;    // 32 次迴圈完成（cnt==0）
          wire        d_busy;         // dpath 是否正在忙碌（可用來 gate）
          wire        d_valid;        // dpath 暫存的「輸入有效」旗標（debug用）
          wire        d_div_zero;     // 除數為 0（僅給 dpath 內部覆蓋結果用）
          wire        d_overflow;     // div 的 INT_MIN / -1 特例（僅給 dpath 內部覆蓋結果用）

          // --------------------------------------------------------------------
          // Datapath instance
          // --------------------------------------------------------------------

          imuldiv_IntDivIterativeDpath dpath
                                       (
                                         .clk                (clk),
                                         .reset              (reset),

                                         // 外部 I/O（直接接頂層）
                                         .divreq_msg_fn      (divreq_msg_fn),
                                         .divreq_msg_a       (divreq_msg_a),
                                         .divreq_msg_b       (divreq_msg_b),
                                         .divreq_val         (divreq_val),
                                         .divreq_rdy         (divreq_rdy),           // 由 ctrl 直接驅動
                                         .divresp_msg_result (divresp_msg_result),   // 由 dpath 打包輸出
                                         .divresp_val        (divresp_val),          // 由 ctrl 直接驅動
                                         .divresp_rdy        (divresp_rdy),

                                         // ctrl → dpath
                                         .c_ld_inputs        (c_ld_inputs),
                                         .c_ld_init          (c_ld_init),
                                         .c_step             (c_step),
                                         .c_pack             (c_pack),
                                         .c_signcorr_en      (c_signcorr_en),
                                         .c_signed_op        (c_signed_op),

                                         // dpath → ctrl
                                         .d_iter_done        (d_iter_done),
                                         .d_div_zero         (d_div_zero),
                                         .d_overflow         (d_overflow),
                                         .d_busy             (d_busy),
                                         .d_valid            (d_valid)
                                       );

          // --------------------------------------------------------------------
          // Control instance (三狀態：IDLE → CALC → DONE)
          // --------------------------------------------------------------------

          imuldiv_IntDivIterativeCtrl ctrl
                                      (
                                        .clk           (clk),
                                        .reset         (reset),

                                        // 外部握手與功能碼（供 ctrl 解碼/決策）
                                        .divreq_val    (divreq_val),
                                        .divresp_rdy   (divresp_rdy),
                                        .divreq_msg_fn (divreq_msg_fn),

                                        // dpath 回報狀態
                                        .d_iter_done   (d_iter_done),
                                        .d_div_zero    (d_div_zero),
                                        .d_overflow    (d_overflow),
                                        .d_busy        (d_busy),
                                        .d_valid       (d_valid),

                                        // ctrl 對外輸出的握手（回灌到頂層 I/O）
                                        .divreq_rdy    (divreq_rdy),
                                        .divresp_val   (divresp_val),

                                        // ctrl 對 dpath 的控制
                                        .c_ld_inputs   (c_ld_inputs),
                                        .c_ld_init     (c_ld_init),
                                        .c_step        (c_step),
                                        .c_pack        (c_pack),
                                        .c_signcorr_en (c_signcorr_en),
                                        .c_signed_op   (c_signed_op)
                                      );

        endmodule

        //------------------------------------------------------------------------
        // Datapath
        //------------------------------------------------------------------------

        module imuldiv_IntDivIterativeDpath
          (
            input         clk,
            input         reset,

            // === 外部 I/O（照老師給的） ===
            input         divreq_msg_fn,      // 0: unsigned, 1: signed
            input  [31:0] divreq_msg_a,       // dividend
            input  [31:0] divreq_msg_b,       // divisor
            input         divreq_val,         // request valid
            output        divreq_rdy,         // request ready  (由 ctrl 驅動；此處高阻避免衝突)
            output [63:0] divresp_msg_result, // {remainder, quotient}
            output        divresp_val,        // response valid (由 ctrl 驅動；此處高阻避免衝突)
            input         divresp_rdy,        // response ready

            // === ctrl → dpath（控制訊號） ===
            input         c_ld_inputs,        // 在 S_IDLE 鎖存輸入 (a/b/fn/val)
            input         c_ld_init,          // 在 S_IDLE 成交同拍：載入 A/B/iter_cnt
            input         c_step,             // 在 S_CALC 執行一次 shift/sub/落商位
            input         c_pack,             // 在 S_CALC 最後一步同拍：打包 {R,Q}
            input         c_signcorr_en,      // 在 S_CALC 最後一步同拍：符號校正
            input         c_signed_op,        // 此次操作是否為 signed（由 ctrl 解碼）

            // === dpath → ctrl（狀態訊號） ===
            output        d_iter_done,        // 迭代完成（cnt==0）
            output        d_div_zero,         // 除數為 0
            output        d_overflow,         // div 的 INT_MIN / -1 特例（僅 signed）
            output        d_busy,             // dpath 忙碌旗標
            output        d_valid             // dpath 內部暫存之「輸入有效」旗標
          );

          // --------------------------------------------------------------------
          // 與 ctrl 的外部握手腳位：改為高阻，由 ctrl 單一來源驅動頂層
          // --------------------------------------------------------------------
          assign divreq_rdy  = 1'bz;
          assign divresp_val = 1'bz;

          // --------------------------------------------------------------------
          // 內部輸入暫存：原始 A/B、指令型別
          // --------------------------------------------------------------------
          reg [31:0] in_a_r, in_b_r;
          reg        in_fn_signed_r;   // 1: signed(div/rem), 0: unsigned(divu/remu)
          reg        valid_r;          // 已鎖到一筆輸入

          // 例外旗標（初始化當拍依輸入決定；不早退，PACK 當拍覆蓋結果以保 33 cycles）
          reg div_zero_r;
          reg overflow_r;

          // --------------------------------------------------------------------
          // 迭代暫存器：A(65)、B(65)、Q(32)、cnt(5)、忙碌旗標
          // --------------------------------------------------------------------
          reg  [64:0] a_reg;           // {33b, 32b}  65-bit 以保留溢位位元供 diff[64] 判斷
          reg  [64:0] b_reg;           // {1’b0, unsigned_b, 32’b0}
          reg  [31:0] q_reg;           // 商
          reg   [4:0] cnt_reg;         // 31..0 共 32 次
          reg         busy_r;          // 進入運算忙碌

          // --------------------------------------------------------------------
          // 結果暫存與輸出
          // --------------------------------------------------------------------
          reg [63:0] result_r;   // {remainder[31:0], quotient[31:0]}
          // c_pack 那拍直接輸出組合結果；非 c_pack 時輸出暫存的舊結果
          assign divresp_msg_result = c_pack ? pack_result_w : result_r;


          // --------------------------------------------------------------------
          // 符號處理號誌與「無號版」操作數
          // --------------------------------------------------------------------
          wire a_neg = in_a_r[31];
          wire b_neg = in_b_r[31];

          // signed_op 時取絕對值；unsigned_op 時原樣
          wire [31:0] abs_a = a_neg ? (~in_a_r + 32'd1) : in_a_r;
          wire [31:0] abs_b = b_neg ? (~in_b_r + 32'd1) : in_b_r;

          wire [31:0] unsigned_a = (c_signed_op) ? abs_a : in_a_r;
          wire [31:0] unsigned_b = (c_signed_op) ? abs_b : in_b_r;

          // 商/餘的號誌（僅 signed）
          wire quo_neg = (c_signed_op) ? (a_neg ^ b_neg) : 1'b0;
          wire rem_neg = (c_signed_op) ?  a_neg          : 1'b0;

          // --------------------------------------------------------------------
          // dpath → ctrl 狀態輸出
          // --------------------------------------------------------------------
          assign d_valid     = valid_r;
          assign d_busy      = busy_r;
          assign d_div_zero  = div_zero_r;
          assign d_overflow  = overflow_r;
          assign d_iter_done = (cnt_reg == 5'd0);  // cnt==0 視為完成（但最後一步仍需執行）

          // --------------------------------------------------------------------
          // 組合：本拍候選的 next 值與「是否最後一步」
          // --------------------------------------------------------------------
          wire [64:0] a_shl1   = { a_reg[63:0], 1'b0 };   // A << 1
          wire [64:0] diff     = a_shl1 - b_reg;          // A<<1 - B (65-bit)
          wire        diff_neg = diff[64];                // 1: 負 → 丟棄差值、商位=0；0: 正 → 採用差值、商位=1

          wire [64:0] next_a   = diff_neg ? a_shl1 : diff;
          wire [31:0] next_q   = diff_neg ? q_reg  : (q_reg | (32'h1 << cnt_reg));

          wire        last_step = c_step && busy_r && (cnt_reg == 5'd0);

          // 基礎值：若是最後一步，用 next_*；否則用當前暫存
          wire [31:0] rem_base = last_step ? next_a[63:32] : a_reg[63:32];
          wire [31:0] quo_base = last_step ? next_q        : q_reg;

          reg [31:0] rem_pack, quo_pack;

          wire a_neg_seed = divreq_msg_a[31];
          wire b_neg_seed = divreq_msg_b[31];

          wire [31:0] seed_a_abs =
               (divreq_msg_fn && a_neg_seed) ? (~divreq_msg_a + 32'd1) : divreq_msg_a;
          wire [31:0] seed_b_abs =
               (divreq_msg_fn && b_neg_seed) ? (~divreq_msg_b + 32'd1) : divreq_msg_b;

          // ---- 打包結果的組合邏輯：同拍可用，避免一拍延遲 ----
          wire [31:0] rem_base_w = last_step ? next_a[63:32] : a_reg[63:32];
          wire [31:0] quo_base_w = last_step ? next_q        : q_reg;

          wire [31:0] rem_exn_w  = div_zero_r  ? in_a_r        :
               overflow_r   ? 32'd0        :
               rem_base_w;

          wire [31:0] quo_exn_w  = div_zero_r  ? 32'hFFFF_FFFF :
               overflow_r   ? 32'h8000_0000 :
               quo_base_w;

          // 符號校正（僅在 signed 且非例外；注意：用 combinational 版本）
          wire do_signcorr = c_signcorr_en && c_signed_op && !div_zero_r && !overflow_r;

          wire [31:0] rem_sc_w = do_signcorr && rem_neg ? (~rem_exn_w) + 32'd1 : rem_exn_w;
          wire [31:0] quo_sc_w = do_signcorr && quo_neg ? (~quo_exn_w) + 32'd1 : quo_exn_w;

          wire [63:0] pack_result_w = { rem_sc_w, quo_sc_w };


          // --------------------------------------------------------------------
          // 同步邏輯
          // --------------------------------------------------------------------
          always @(posedge clk or posedge reset)
          begin
            if (reset)
            begin
              in_a_r         <= 32'd0;
              in_b_r         <= 32'd0;
              in_fn_signed_r <= 1'b0;
              valid_r        <= 1'b0;

              div_zero_r     <= 1'b0;
              overflow_r     <= 1'b0;

              a_reg          <= 65'd0;
              b_reg          <= 65'd0;
              q_reg          <= 32'd0;
              cnt_reg        <= 5'd0;
              busy_r         <= 1'b0;

              result_r       <= 64'd0;
            end
            else
            begin
              // ------------------------------
              // S_IDLE: 鎖存輸入（由 ctrl 在 IDLE 成交同拍 assert）
              // ------------------------------
              if (c_ld_inputs)
              begin
                in_a_r         <= divreq_msg_a;
                in_b_r         <= divreq_msg_b;
                in_fn_signed_r <= divreq_msg_fn;   // 保留原始 fn
                valid_r        <= 1'b1;
              end

              // ------------------------------
              // S_IDLE 成交同拍：初始化（併入介面開銷那 1 拍）
              // ------------------------------
              if (c_ld_init)
              begin
                div_zero_r <= (divreq_msg_b == 32'd0);
                overflow_r <= (divreq_msg_fn &&
                               (divreq_msg_a == 32'h8000_0000) &&
                               (divreq_msg_b == 32'hFFFF_FFFF));

                // 初始化 A/B
                a_reg   <= { 33'd0, seed_a_abs };
                b_reg   <= { 1'b0, seed_b_abs, 32'd0 };

                q_reg   <= 32'd0;
                cnt_reg <= 5'd31;
                busy_r  <= 1'b1;
              end

              // ------------------------------
              // S_CALC: 迭代（每拍一步；最後一步仍需執行，cnt==0 那拍）
              // ------------------------------
              if (c_step)
              begin
                if (busy_r)
                begin
                  a_reg <= next_a;
                  q_reg <= next_q;
                  if (cnt_reg != 5'd0)
                    cnt_reg <= cnt_reg - 5'd1;
                end
              end

              // ------------------------------
              // S_CALC 最後一步同拍：打包 + 例外覆蓋 +（如需）符號校正
              // ------------------------------
              if (c_pack)
              begin
                result_r <= pack_result_w; // 同拍也寄存，供 S_DONE/hold 使用
                busy_r   <= 1'b0;
              end


              // valid_r 的生命週期交由 ctrl 握手節奏決定；此處不自動清
            end
          end

        endmodule

        //------------------------------------------------------------------------
        // Control Logic (Three-state; pack-on-last-step; exact 33 cycles)
        //------------------------------------------------------------------------
        module imuldiv_IntDivIterativeCtrl
          (
            input  clk,
            input  reset,

            // 外部握手與功能碼（供 ctrl 解碼/決策）
            input  divreq_val,
            input  divresp_rdy,
            input  divreq_msg_fn,   // 0: unsigned, 1: signed

            // dpath 回報狀態
            input  d_iter_done,     // cnt==0（最後一步當拍仍會 step）
            input  d_div_zero,      // 僅供觀察/除錯，不影響三態流程
            input  d_overflow,      // 僅供觀察/除錯，不影響三態流程
            input  d_busy,
            input  d_valid,

            // ctrl 對外部 I/O 的握手輸出
            output reg divreq_rdy,
            output reg divresp_val,

            // ctrl → dpath 的控制
            output reg c_ld_inputs,
            output reg c_ld_init,
            output reg c_step,
            output reg c_pack,
            output reg c_signcorr_en,
            output     c_signed_op
          );

          // 三狀態
          parameter S_IDLE = 2'd0;
          parameter S_CALC = 2'd1;
          parameter S_DONE = 2'd2;

          reg [1:0] state, state_n;

          // 針對本筆交易鎖存是否為 signed 操作
          reg signed_op_r;
          assign c_signed_op = signed_op_r;

          // 狀態暫存
          always @(posedge clk or posedge reset)
          begin
            if (reset)
            begin
              state       <= S_IDLE;
              signed_op_r <= 1'b0;
            end
            else
            begin
              state <= state_n;
              if (state == S_IDLE && divreq_val && divreq_rdy)
                signed_op_r <= divreq_msg_fn; // 1:signed，0:unsigned
            end
          end

          // 次態轉移 & 輸出控制（組合）
          always @*
          begin
            // 預設值
            state_n       = state;

            divreq_rdy    = 1'b0;
            divresp_val   = 1'b0;

            c_ld_inputs   = 1'b0;
            c_ld_init     = 1'b0;
            c_step        = 1'b0;
            c_pack        = 1'b0;
            c_signcorr_en = 1'b0;

            case (state)
              // --------------------------------------------------
              // S_IDLE：等待一筆請求；成交同拍即完成「鎖入 + 初始化」
              // 這一拍即為介面開銷的 1 cycle
              // --------------------------------------------------
              S_IDLE:
              begin
                divreq_rdy  = 1'b1;               // 我方準備好收
                if (divreq_val && divreq_rdy)
                begin
                  c_ld_inputs = 1'b1;             // 請 dpath 鎖入 a/b/fn
                  c_ld_init   = 1'b1;             // 初始化 A/B/Q/cnt/busy
                  state_n     = S_CALC;
                end
              end

              // --------------------------------------------------
              // S_CALC：每拍一步；當 d_iter_done（cnt==0）那拍：
              // - 同拍下達 c_pack
              // - 若 signed，本拍 c_signcorr_en=1 做號誌校正
              // - 同拍 divresp_val=1，若對方 ready 則直接回 IDLE
              // --------------------------------------------------
              S_CALC:
              begin
                c_step = 1'b1;
                if (d_iter_done)
                begin
                  c_pack        = 1'b1;          // 同拍打包
                  c_signcorr_en = signed_op_r;   // 僅 signed 才校正
                  // divresp_val 不要在這拍拉高
                  state_n       = S_DONE;        // 一律進 DONE
                end
              end

              // --------------------------------------------------
              // S_DONE：若上一拍外面沒 ready，就在這裡等；ready 後回 IDLE
              // --------------------------------------------------
              S_DONE:
              begin
                divresp_val = 1'b1;              // 這拍才對外宣告結果有效
                if (divresp_rdy)
                  state_n = S_IDLE;
              end
              default:
              begin
                state_n = S_IDLE;
              end
            endcase
          end

        endmodule

`endif
