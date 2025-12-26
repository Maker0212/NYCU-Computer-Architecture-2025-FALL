//========================================================================
// Lab1 - IntMulThreeInput (Verilog 2005)
// 三輸入整數乘法：輸出為完整 96 位 (signed-correct)
// 僅使用兩顆 32x32→64 Booth 乘法器 (imuldiv-IntMulBooth)
// Phase 1: mul0 計算 AB (signed 64-bit)
// Phase 2: 併行送出 L*C (mul0) 與 H*C (mul1)
// 最終：final96 = (L_signed*C_signed) + (L[31]? (C<<32):0) + ((H_signed*C_signed)<<32)
//        以上每一項一律先做成 96 位再相加，確保高位正確
//========================================================================

`ifndef RISCV_INT_MULDIV_THREEINPUT_V
`define RISCV_INT_MULDIV_THREEINPUT_V

`include "imuldiv-ThreeMulReqMsg.v"
`include "imuldiv-IntMulBooth.v"

        module imuldiv_IntMulThreeInput
          (
            input         clk,
            input         reset,

            // req
            input   [2:0] muldivreq_msg_fn,  // 保留不用（僅 mul）
            input  [31:0] muldivreq_msg_a,
            input  [31:0] muldivreq_msg_b,
            input  [31:0] muldivreq_msg_c,
            input         muldivreq_val,
            output        muldivreq_rdy,

            // resp
            output [95:0] muldivresp_msg_result, // 完整 96 位（不再強制高 32 = 0）
            output        muldivresp_val,
            input         muldivresp_rdy
          );

          // ---------------------------------------------------------------------
          // FSM
          // ---------------------------------------------------------------------
          localparam S_IDLE        = 3'd0;
          localparam S_MUL_AB_REQ  = 3'd1;
          localparam S_MUL_AB_WAIT = 3'd2;
          localparam S_P2_REQ      = 3'd3;
          localparam S_P2_WAIT     = 3'd4;
          localparam S_ACCUM       = 3'd5;
          localparam S_RESP        = 3'd6;

          reg [2:0] state, state_n;

          // ---------------------------------------------------------------------
          // 暫存/中間結果（為了與 Booth 相容，內部皆以無號向量表達，但語意為 signed）
          // ---------------------------------------------------------------------
          reg [31:0] a_q, b_q, c_q;   // request 暫存
          reg [63:0] ab_q;            // A*B (signed 64，Booth 輸出)
          reg [63:0] lc_q;            // L_signed * C_signed（Booth 輸出，稍後修正為 L_unsigned*C）
          reg [63:0] hc_q;            // H_signed * C_signed（Booth 輸出）

          wire [31:0] L = ab_q[31:0];
          wire [31:0] H = ab_q[63:32];

          // ---- 96-bit 擴展與校正項（全部在 always 外宣告/運算） ----

          // L*C（Booth 算的是 L_signed*C_signed），先 sign-extend 到 96
          wire [95:0] lc_ss96   = { {32{lc_q[63]}}, lc_q };

          // 校正：若 L[31]=1，需加上 (C << 32)，且以 96-bit 表示
          wire [95:0] c_shift96 = { {64{c_q[31]}}, c_q, 32'b0 };
          wire [95:0] lc_fix96  = lc_ss96 + ( L[31] ? c_shift96 : 96'd0 );

          // H*C（Booth 算的是 H_signed*C_signed），先 sign-extend 到 96，再左移 32
          wire [95:0] hc_ss96   = { {32{hc_q[63]}}, hc_q };
          wire [95:0] hc_sll32  = { hc_ss96[63:0], 32'b0 }; // 等價於 (hc_ss96 <<< 32)

          // 最終 96 位結果：全部在 96 位寬上相加
          wire [95:0] final96   = lc_fix96 + hc_sll32;

          // ---------------------------------------------------------------------
          // 兩顆 Booth 乘法器介面
          // ---------------------------------------------------------------------
          // mul0
          reg         mul0_req_val;
          wire        mul0_req_rdy;
          reg  [31:0] mul0_req_a;
          reg  [31:0] mul0_req_b;

          wire        mul0_resp_val;
          reg         mul0_resp_rdy;
          wire [63:0] mul0_resp_res;

          // mul1
          reg         mul1_req_val;
          wire        mul1_req_rdy;
          reg  [31:0] mul1_req_a;
          reg  [31:0] mul1_req_b;

          wire        mul1_resp_val;
          reg         mul1_resp_rdy;
          wire [63:0] mul1_resp_res;

          imuldiv_IntMulBooth mul0
                              (
                                .clk                (clk),
                                .reset              (reset),
                                .mulreq_msg_a       (mul0_req_a),
                                .mulreq_msg_b       (mul0_req_b),
                                .mulreq_val         (mul0_req_val),
                                .mulreq_rdy         (mul0_req_rdy),
                                .mulresp_msg_result (mul0_resp_res),
                                .mulresp_val        (mul0_resp_val),
                                .mulresp_rdy        (mul0_resp_rdy)
                              );

          imuldiv_IntMulBooth mul1
                              (
                                .clk                (clk),
                                .reset              (reset),
                                .mulreq_msg_a       (mul1_req_a),
                                .mulreq_msg_b       (mul1_req_b),
                                .mulreq_val         (mul1_req_val),
                                .mulreq_rdy         (mul1_req_rdy),
                                .mulresp_msg_result (mul1_resp_res),
                                .mulresp_val        (mul1_resp_val),
                                .mulresp_rdy        (mul1_resp_rdy)
                              );

          // Phase 2 sent/got 旗標
          reg p2_sent0, p2_sent1;
          reg p2_got0,  p2_got1;

          // ---------------------------------------------------------------------
          // 對外握手
          // ---------------------------------------------------------------------
          assign muldivreq_rdy = (state == S_IDLE);

          reg resp_val_q;
          assign muldivresp_val        = resp_val_q;
          assign muldivresp_msg_result = final96; // 直接輸出 96 位結果

          // ---------------------------------------------------------------------
          // 次態邏輯
          // ---------------------------------------------------------------------
          always @(*)
          begin
            state_n = state;
            case (state)
              S_IDLE:
                if (muldivreq_val && muldivreq_rdy)
                  state_n = S_MUL_AB_REQ;
              S_MUL_AB_REQ:
                if (mul0_req_val && mul0_req_rdy)
                  state_n = S_MUL_AB_WAIT;
              S_MUL_AB_WAIT:
                if (mul0_resp_val)
                  state_n = S_P2_REQ;
              S_P2_REQ:
                if (p2_sent0 && p2_sent1)
                  state_n = S_P2_WAIT;
              S_P2_WAIT:
                if (p2_got0 && p2_got1)
                  state_n = S_ACCUM;
              S_ACCUM:
                state_n = S_RESP;
              S_RESP:
                if (muldivresp_val && muldivresp_rdy)
                  state_n = S_IDLE;
              default:
                state_n = S_IDLE;
            endcase
          end

          // ---------------------------------------------------------------------
          // 狀態暫存
          // ---------------------------------------------------------------------
          always @(posedge clk)
          begin
            if (reset)
              state <= S_IDLE;
            else
              state <= state_n;
          end

          // ---------------------------------------------------------------------
          // Booth req/resp 的組合預設
          // ---------------------------------------------------------------------
          always @(*)
          begin
            // 預設：不送 req，resp 可隨時接
            mul0_req_val  = 1'b0;
            mul0_req_a    = 32'd0;
            mul0_req_b    = 32'd0;
            mul0_resp_rdy = 1'b1;

            mul1_req_val  = 1'b0;
            mul1_req_a    = 32'd0;
            mul1_req_b    = 32'd0;
            mul1_resp_rdy = 1'b1;

            case (state)
              // Phase 1：送 A*B 到 mul0
              S_MUL_AB_REQ:
              begin
                mul0_req_val = 1'b1;
                mul0_req_a   = a_q;
                mul0_req_b   = b_q;
              end

              // Phase 2：同時送 L*C（mul0）與 H*C（mul1），若尚未送過才送，避免重送
              S_P2_REQ:
              begin
                if (!p2_sent0)
                begin
                  mul0_req_val = 1'b1;
                  mul0_req_a   = L;    // L_signed
                  mul0_req_b   = c_q;  // C_signed
                end
                if (!p2_sent1)
                begin
                  mul1_req_val = 1'b1;
                  mul1_req_a   = H;    // H_signed
                  mul1_req_b   = c_q;  // C_signed
                end
              end
            endcase
          end

          // ---------------------------------------------------------------------
          // 序向邏輯：資料鎖存、旗標、resp_valid
          // ---------------------------------------------------------------------
          always @(posedge clk)
          begin
            if (reset)
            begin
              a_q <= 32'd0;
              b_q <= 32'd0;
              c_q <= 32'd0;
              ab_q <= 64'd0;
              lc_q <= 64'd0;
              hc_q <= 64'd0;
              p2_sent0 <= 1'b0;
              p2_sent1 <= 1'b0;
              p2_got0  <= 1'b0;
              p2_got1  <= 1'b0;
              resp_val_q <= 1'b0;
            end
            else
            begin
              // 讀入 A/B/C
              if (state == S_IDLE && muldivreq_val && muldivreq_rdy)
              begin
                // muldivreq_msg_fn 保留不用
                a_q <= muldivreq_msg_a;
                b_q <= muldivreq_msg_b;
                c_q <= muldivreq_msg_c;
              end

              // 取得 AB
              if (state == S_MUL_AB_WAIT && mul0_resp_val)
              begin
                ab_q <= mul0_resp_res; // 64-bit（signed 語意）
              end

              // 進入 P2_REQ 清 sent/got
              if (state_n == S_P2_REQ && state != S_P2_REQ)
              begin
                p2_sent0 <= 1'b0;
                p2_sent1 <= 1'b0;
                p2_got0  <= 1'b0;
                p2_got1  <= 1'b0;
                // 也可以在此清暫存（非必要，僅避免波形殘值誤導）
                lc_q <= 64'd0;
                hc_q <= 64'd0;
              end

              // 記錄送出
              if (state == S_P2_REQ)
              begin
                if (!p2_sent0 && mul0_req_val && mul0_req_rdy)
                  p2_sent0 <= 1'b1;
                if (!p2_sent1 && mul1_req_val && mul1_req_rdy)
                  p2_sent1 <= 1'b1;
              end

              // 收到 P2 結果
              if (state == S_P2_WAIT)
              begin
                if (mul0_resp_val)
                begin
                  lc_q   <= mul0_resp_res; // L_signed*C_signed
                  p2_got0 <= 1'b1;
                end
                if (mul1_resp_val)
                begin
                  hc_q   <= mul1_resp_res; // H_signed*C_signed
                  p2_got1 <= 1'b1;
                end
              end

              // RESP valid：在 S_RESP 維持為 1，直到 rdy
              if (state_n == S_RESP && state != S_RESP)
              begin
                resp_val_q <= 1'b1;
              end
              else if (state == S_RESP && muldivresp_rdy)
              begin
                resp_val_q <= 1'b0;
              end
            end
          end

          // ---------------------------------------------------------------------
          // （可選）模擬期防呆：在 RESP 檢查 P2 兩路都已就緒
          // ---------------------------------------------------------------------
          always @(*)
          begin
            if (state == S_RESP)
            begin
              if (!(p2_got0 && p2_got1))
              begin
                // 若看到這行表示 FSM 流程有問題（應該先等到兩路皆 valid 再進 RESP）
                // $display("WARN: RESP without both P2 results ready!");
              end
            end
          end

        endmodule

`endif



