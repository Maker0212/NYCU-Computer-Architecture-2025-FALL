//========================================================================
// Lab 1/2 - Iterative Mul/Div Unit (with MULH/MULHSU/MULHU support)
//========================================================================

`ifndef RISCV_INT_MULDIV_ITERATIVE_V
`define RISCV_INT_MULDIV_ITERATIVE_V

`include "imuldiv-MulDivReqMsg.v"
`include "imuldiv-IntMulIterative.v"
`include "imuldiv-IntDivIterative.v"

        module imuldiv_IntMulDivIterative
          (
            input         clk,
            input         reset,

            // req
            input   [2:0] muldivreq_msg_fn,
            input  [31:0] muldivreq_msg_a,
            input  [31:0] muldivreq_msg_b,
            input         muldivreq_val,
            output        muldivreq_rdy,

            // resp
            output [63:0] muldivresp_msg_result,
            output        muldivresp_val,
            input         muldivresp_rdy
          );

          //----------------------------------------------------------------------
          // Function decode
          //----------------------------------------------------------------------

          localparam MD_MUL    = 3'd0;
          localparam MD_DIV    = 3'd1;
          localparam MD_DIVU   = 3'd2;
          localparam MD_REM    = 3'd3;
          localparam MD_REMU   = 3'd4;
          localparam MD_MULH   = 3'd5; // signed × signed -> high
          localparam MD_MULHSU = 3'd6; // signed × unsigned -> high
          localparam MD_MULHU  = 3'd7; // unsigned × unsigned -> high

          wire is_mul_family =
               (muldivreq_msg_fn==MD_MUL)    ||
               (muldivreq_msg_fn==MD_MULH)   ||
               (muldivreq_msg_fn==MD_MULHSU) ||
               (muldivreq_msg_fn==MD_MULHU);

          // 將乘法與除法串接為互斥請求，保證同拍只打一個子單元
          // 讓 req_rdy = mul_rdy && div_rdy 確保 backpressure 正確傳回上游
          wire mulreq_rdy, divreq_rdy;
          wire mulresp_val, divresp_val;

          // 只有當另一方 ready 才允許這一方發 req，避免同拍競爭
          wire mulreq_val = is_mul_family && muldivreq_val && divreq_rdy;
          wire divreq_val = (!is_mul_family) && muldivreq_val && mulreq_rdy;

          assign muldivreq_rdy  = mulreq_rdy && divreq_rdy;
          assign muldivresp_val = mulresp_val || divresp_val;

          //----------------------------------------------------------------------
          // 有/無號處理（僅針對 MULH 與 MULHSU 需要「絕對值×絕對值 + 事後符號修正」）
          //   - MUL  / 低半部：無須處理（低 32 位對 signed/unsigned 相同）
          //   - MULHU：無號×無號，直接當正數相乘
          //   - MULH ：有號×有號，對兩邊取絕對值相乘，最後若符號不同再整體取負
          //   - MULHSU：有號×無號，只看 A 的符號取絕對值與最後取負
          //----------------------------------------------------------------------

          wire a_neg_raw = muldivreq_msg_a[31];
          wire b_neg_raw = muldivreq_msg_b[31];

          // 取絕對值（two's complement）
          wire [31:0] a_abs = a_neg_raw ? (~muldivreq_msg_a + 32'd1) : muldivreq_msg_a;
          wire [31:0] b_abs = b_neg_raw ? (~muldivreq_msg_b + 32'd1) : muldivreq_msg_b;

          // 依功能決定送進乘法器的操作數（皆視為「非負的 32-bit 數值」）
          reg [31:0] mul_a_pre, mul_b_pre;
          reg        mul_negate_result;  // 乘積是否需要最後整體取負（two's complement）

          always @(*)
          begin
            mul_a_pre = muldivreq_msg_a;
            mul_b_pre = muldivreq_msg_b;
            mul_negate_result = 1'b0;

            case (muldivreq_msg_fn)
              MD_MUL:
              begin
                // 低半部；signed/unsigned 低 32 位相同，不需特別處理
                mul_a_pre = muldivreq_msg_a;
                mul_b_pre = muldivreq_msg_b;
                mul_negate_result = 1'b0;
              end

              MD_MULHU:
              begin
                // 無號×無號：直接以「非負」解讀
                mul_a_pre = muldivreq_msg_a; // 其實等同 zero-extend 32->32
                mul_b_pre = muldivreq_msg_b;
                mul_negate_result = 1'b0;
              end

              MD_MULH:
              begin
                // 有號×有號：用絕對值相乘；若 a、b 其中之一為負則最後取負
                mul_a_pre = a_abs;
                mul_b_pre = b_abs;
                mul_negate_result = (a_neg_raw ^ b_neg_raw);
              end

              MD_MULHSU:
              begin
                // 有號×無號：只看 A 的符號
                mul_a_pre = a_abs;               // A 取絕對值
                mul_b_pre = muldivreq_msg_b;     // B 視為無號，不取絕對值
                mul_negate_result = a_neg_raw;   // 若 A 原本為負，最後取負
              end

              default:
              begin
                // 其它（DIV/REM）不使用這兩個暫存值
                mul_a_pre = muldivreq_msg_a;
                mul_b_pre = muldivreq_msg_b;
                mul_negate_result = 1'b0;
              end
            endcase
          end

          // 為了在握手成功時鎖定「是否需要最後取負」的旗標（避免之後功能碼變動）
          wire mul_fire = mulreq_val && mulreq_rdy;

          reg negate_result_q;
          always @(posedge clk)
          begin
            if (reset)
            begin
              negate_result_q <= 1'b0;
            end
            else if (mul_fire)
            begin
              negate_result_q <= mul_negate_result;
            end
          end

          //----------------------------------------------------------------------
          // 子模組：乘法/除法
          // 乘法器輸出為「非負」乘積（64-bit）。若需要有號結果，之後以 two's complement 取負。
          //----------------------------------------------------------------------

          wire [63:0] mulresp_msg_result_raw; // 乘法器原始 64-bit 乘積
          wire [63:0] divresp_msg_result;     // 除法器 64-bit 結果（依功能內部打包）

          imuldiv_IntMulIterative imul
                                  (
                                    .clk                (clk),
                                    .reset              (reset),
                                    .mulreq_msg_a       (mul_a_pre),
                                    .mulreq_msg_b       (mul_b_pre),
                                    .mulreq_val         (mulreq_val),
                                    .mulreq_rdy         (mulreq_rdy),
                                    .mulresp_msg_result (mulresp_msg_result_raw),
                                    .mulresp_val        (mulresp_val),
                                    .mulresp_rdy        (muldivresp_rdy)
                                  );

          // 除法/餘數：維持原本行為（div 模組內部已有有/無號選擇）
          wire divreq_msg_fn_signed =
               (muldivreq_msg_fn==MD_DIV) || (muldivreq_msg_fn==MD_REM);

          imuldiv_IntDivIterative idiv
                                  (
                                    .clk                (clk),
                                    .reset              (reset),
                                    .divreq_msg_fn      (divreq_msg_fn_signed),
                                    .divreq_msg_a       (muldivreq_msg_a),
                                    .divreq_msg_b       (muldivreq_msg_b),
                                    .divreq_val         (divreq_val),
                                    .divreq_rdy         (divreq_rdy),
                                    .divresp_msg_result (divresp_msg_result),
                                    .divresp_val        (divresp_val),
                                    .divresp_rdy        (muldivresp_rdy)
                                  );

          //----------------------------------------------------------------------
          // 乘法結果的最後符號修正（僅在 MULH/MULHSU 可能需要）
          //   - 若 negate_result_q 為 1，對 64-bit 乘積做 two's complement
          //----------------------------------------------------------------------

          wire [63:0] mulresp_msg_result_fixed =
               negate_result_q ? (~mulresp_msg_result_raw + 64'd1) : mulresp_msg_result_raw;

          //----------------------------------------------------------------------
          // 輸出多工：優先輸出乘法結果；否則輸出除法結果
          //----------------------------------------------------------------------

          assign muldivresp_msg_result =
                 mulresp_val ? mulresp_msg_result_fixed :
                 divresp_val ? divresp_msg_result      :
                 64'bx;

        endmodule

`endif
