//========================================================================
// Lab 1 - Iterative Mul Unit
//========================================================================

`ifndef RISCV_INT_MUL_ITERATIVE_V
`define RISCV_INT_MUL_ITERATIVE_V

        module imuldiv_IntMulIterative
          (

            input                clk,
            input                reset,

            // val/rdy request
            input  [31:0] mulreq_msg_a,
            input  [31:0] mulreq_msg_b,
            input         mulreq_val,
            output        mulreq_rdy,

            // val/rdy response
            output [63:0] mulresp_msg_result,
            output        mulresp_val,
            input         mulresp_rdy
          );

          // -------------------------
          // Control <-> Datapath wires
          // -------------------------

          // control outputs to datapath
          wire        a_mux_sel;        // 0: load {32'b0, unsigned_a}; 1: shift result of A
          wire        a_shift_en;       // enable A <<= 1
          wire        b_mux_sel;        // 0: load unsigned_b; 1: shift result of B
          wire        b_shift_en;       // enable B >>= 1
          wire        result_mux_sel;   // 0: hold; 1: add_mux_out
          wire        result_en;        // enable Result update
          wire        counter_en;       // enable counter update
          wire        counter_mux_sel;  // 0: init to 31; 1: decrement
          wire        sign_en;          // enable sign_reg write
          wire        sign_mux_sel;     // 0: compute sign from inputs; 1: hold
          wire        add_mux_sel;      // 0: pass Result; 1: Result + A

          // datapath status to control
          wire        b0;               // b_reg[0]
          wire        counter_is_zero;  // counter == 0

          // handshake wires driven by ctrl
          wire req_rdy_o;   // ctrl decides when to accept a new request
          wire resp_val_o;  // ctrl decides when result is valid



          // -------------------------
          // Datapath
          // -------------------------
          imuldiv_IntMulIterativeDpath dpath (
                                         .clk(clk),
                                         .reset(reset),
                                         // request / response ports
                                         .mulreq_msg_a(mulreq_msg_a),
                                         .mulreq_msg_b(mulreq_msg_b),
                                         .mulreq_val  (mulreq_val),
                                         .mulreq_rdy  (mulreq_rdy),
                                         .mulresp_msg_result(mulresp_msg_result),
                                         .mulresp_val (mulresp_val),
                                         .mulresp_rdy (mulresp_rdy),

                                         // control signals (from ctrl)
                                         .a_mux_sel   (a_mux_sel),
                                         .a_shift_en  (a_shift_en),
                                         .b_mux_sel   (b_mux_sel),
                                         .b_shift_en  (b_shift_en),
                                         .result_mux_sel(result_mux_sel),
                                         .result_en   (result_en),
                                         .counter_en  (counter_en),
                                         .counter_mux_sel(counter_mux_sel),
                                         .sign_en     (sign_en),
                                         .sign_mux_sel(sign_mux_sel),
                                         .add_mux_sel (add_mux_sel),

                                         // status signals (to ctrl)
                                         .b0          (b0),
                                         .counter_is_zero(counter_is_zero),

                                         // handshake from ctrl
                                         .req_rdy_i        (req_rdy_o),
                                         .resp_val_i       (resp_val_o)
                                       );


          // -------------------------
          // Control
          // -------------------------
          imuldiv_IntMulIterativeCtrl ctrl (
                                        .clk(clk),
                                        .reset(reset),

                                        // status from datapath
                                        .b0(b0),
                                        .counter_is_zero(counter_is_zero),

                                        // control outputs to datapath
                                        .a_mux_sel(a_mux_sel),
                                        .a_shift_en(a_shift_en),
                                        .b_mux_sel(b_mux_sel),
                                        .b_shift_en(b_shift_en),
                                        .result_mux_sel(result_mux_sel),
                                        .result_en(result_en),
                                        .counter_en(counter_en),
                                        .counter_mux_sel(counter_mux_sel),
                                        .sign_en(sign_en),
                                        .sign_mux_sel(sign_mux_sel),
                                        .add_mux_sel(add_mux_sel),

                                        // handshake outputs
                                        .req_rdy_o        (req_rdy_o),
                                        .resp_val_o       (resp_val_o),
                                        .mulreq_val_i     (mulreq_val),
                                        .mulresp_rdy_i    (mulresp_rdy)
                                      );

        endmodule

        //------------------------------------------------------------------------
        // Datapath (skeleton only; no algorithm yet)
        // - Registers: A, B, Result, Sign, Counter
        // - MUX/shift wiring controlled by ctrl signals
        // - Status to ctrl: b0 (B LSB), counter_is_zero
        // - Handshake: driven by ctrl (dpath just passes through)
        //------------------------------------------------------------------------
        module imuldiv_IntMulIterativeDpath
          (
            input         clk,
            input         reset,

            // val/rdy request (payload comes from upstream)
            input  [31:0] mulreq_msg_a,
            input  [31:0] mulreq_msg_b,
            input         mulreq_val,
            output        mulreq_rdy,

            // val/rdy response (payload goes to downstream)
            output [63:0] mulresp_msg_result,
            output        mulresp_val,
            input         mulresp_rdy,

            // -------- control inputs from ctrl --------
            input         a_mux_sel,        // 0: load {32'b0, unsigned_a}; 1: take a_shift_out
            input         a_shift_en,       // enable A <<= 1 (effective when a_mux_sel==1)
            input         b_mux_sel,        // 0: load unsigned_b; 1: take b_shift_out
            input         b_shift_en,       // enable B >>= 1 (effective when b_mux_sel==1)
            input         result_mux_sel,   // 0: hold; 1: take add_mux_out
            input         result_en,        // result_reg write enable
            input         counter_en,       // counter write enable
            input         counter_mux_sel,  // 0: init to 31; 1: decrement
            input         sign_en,          // sign_reg write enable
            input         sign_mux_sel,     // 0: compute from inputs; 1: hold previous
            input         add_mux_sel,      // 0: pass Result; 1: Result + A

            // -------- handshake pass-through from ctrl --------
            input         req_rdy_i,        // ctrl-computed mulreq_rdy
            input         resp_val_i,       // ctrl-computed mulresp_val

            // -------- status outputs to ctrl ----------
            output        b0,               // B LSB
            output        counter_is_zero   // counter==0
          );

          //----------------------------------------------------------------------
          // Unsigning logic (Figure 6: 'unsign' blocks) — 只做資料整備，不改演算法
          //----------------------------------------------------------------------

          wire sign_bit_a   = mulreq_msg_a[31];
          wire sign_bit_b   = mulreq_msg_b[31];

          // two's complement if negative
          wire [31:0] unsigned_a = sign_bit_a ? (~mulreq_msg_a + 32'd1) : mulreq_msg_a;
          wire [31:0] unsigned_b = sign_bit_b ? (~mulreq_msg_b + 32'd1) : mulreq_msg_b;

          //----------------------------------------------------------------------
          // Registers
          //  - A: 64b, 初始載入 {32'b0, unsigned_a}
          //  - B: 32b, 初始載入 unsigned_b
          //  - Result: 64b, 初始 0
          //  - Sign: 1b,   記錄結果正負
          //  - Counter: 5b, 由 ctrl 控制 init=31、之後遞減
          //----------------------------------------------------------------------

          reg  [63:0] a_reg;
          reg  [31:0] b_reg;
          reg  [63:0] result_reg;
          reg         sign_reg;
          reg  [5:0]  counter_reg;

          //----------------------------------------------------------------------
          // Shift paths & next values
          //----------------------------------------------------------------------

          wire [63:0] a_shift_out = a_reg << 1;      // for iterative shift-and-add
          wire [31:0] b_shift_out = b_reg >> 1;

          // A next: select load vs shift
          wire [63:0] a_load_val  = {32'b0, unsigned_a};
          wire [63:0] a_next_mux  = (a_mux_sel) ? a_shift_out : a_load_val;

          // B next: select load vs shift
          wire [31:0] b_load_val  = unsigned_b;
          wire [31:0] b_next_mux  = (b_mux_sel) ? b_shift_out : b_load_val;

          // Add mux: Result or (Result + A)
          wire [63:0] add_mux_out = (add_mux_sel) ? (result_reg + a_reg) : result_reg;

          // Result next
          wire [63:0] result_next = (result_mux_sel) ? add_mux_out : result_reg;

          // Counter next
          wire [5:0]  counter_dec = counter_reg - 6'd1;
          wire [5:0]  counter_init= 6'd32;
          wire [5:0]  counter_next= (counter_mux_sel) ? counter_dec : counter_init;

          reg captured_sign;

          // 在握手那一拍鎖住本次運算的 sign（避免下一拍上游換向量）
          always @(posedge clk)
          begin
            if (reset)
            begin
              captured_sign <= 1'b0;
            end
            else if (req_fire)
            begin
              captured_sign <= (sign_bit_a ^ sign_bit_b);
            end
          end

          // 在「握手當拍」直接用當前輸入的符號；否則用前一拍鎖住的 captured_sign
          wire sign_load_immediate = (sign_bit_a ^ sign_bit_b);
          wire sign_load_val       = req_fire ? sign_load_immediate : captured_sign;
          wire sign_next           = (sign_mux_sel) ? sign_reg : sign_load_val;



          wire req_fire = mulreq_val & req_rdy_i;

          //----------------------------------------------------------------------
          // Sequential updates
          //----------------------------------------------------------------------

          always @(posedge clk)
          begin
            if (reset)
            begin
              a_reg       <= 64'd0;
              b_reg       <= 32'd0;
              result_reg  <= 64'd0;
              sign_reg    <= 1'b0;
              counter_reg <= 6'd0;
            end
            else
            begin
              // -------- A register --------
              // 只有在「負載路徑且握手成功」才載入；只有在「shift 路徑且允許 shift」才移位；否則保持
              if (!a_mux_sel && req_fire)
                a_reg <= a_load_val;   // LOAD {0, unsigned_a}
              else if (a_mux_sel && a_shift_en)
                a_reg <= a_shift_out;  // SHIFT (<<1)
              // else hold

              // -------- B register --------
              if (!b_mux_sel && req_fire)
                b_reg <= b_load_val;   // LOAD unsigned_b
              else if (b_mux_sel && b_shift_en)
                b_reg <= b_shift_out;  // SHIFT (>>1)
              // else hold

              // -------- Counter --------
              if (counter_en)
                counter_reg <= counter_next;     // init=32 或遞減由 counter_mux_sel 決定

              // -------- Result --------
              // 修正：在「counter 初始化那拍」清 0（counter_mux_sel==0 表示選 init）
              if (counter_en && (counter_mux_sel == 1'b0))
                result_reg <= 64'd0;
              else if (result_en)
                result_reg <= result_next;

              // -------- Sign --------
              if (sign_en)
                sign_reg <= sign_next;
            end
          end

          //----------------------------------------------------------------------
          // Outputs: status to ctrl, and final signed result to downstream
          //----------------------------------------------------------------------

          assign b0               = b_reg[0];
          assign counter_is_zero  = (counter_reg == 6'd1);

          // 將 sign 套用到 Result；實際何時套用由 ctrl 排程，這裡給出已簽名結果
          wire [63:0] signed_result = sign_reg ? (~result_reg + 64'd1) : result_reg;

          assign mulresp_msg_result = signed_result;

          //----------------------------------------------------------------------
          // Handshake lines (由 ctrl 決定；dpath 僅轉接)
          //----------------------------------------------------------------------

          assign mulreq_rdy  = req_rdy_i;   // ctrl 決定什麼時候接受新請求
          assign mulresp_val = resp_val_i;  // ctrl 決定什麼時候宣告結果有效

        endmodule


        //------------------------------------------------------------------------
        // Control Logic
        //------------------------------------------------------------------------

        module imuldiv_IntMulIterativeCtrl
          (
            input         clk,
            input         reset,

            // status from datapath
            input         b0,
            input         counter_is_zero,

            // control outputs to datapath
            output reg        a_mux_sel,
            output reg        a_shift_en,
            output reg        b_mux_sel,
            output reg        b_shift_en,
            output reg        result_mux_sel,
            output reg        result_en,
            output reg        counter_en,
            output reg        counter_mux_sel,
            output reg        sign_en,
            output reg        sign_mux_sel,
            output reg        add_mux_sel,

            // handshake to/from top/dpath
            output reg        req_rdy_o,     // drives dpath.mulreq_rdy
            output reg        resp_val_o,    // drives dpath.mulresp_val
            input             mulreq_val_i,  // from top (upstream)
            input             mulresp_rdy_i  // from top (downstream)
          );

          // FSM encoding (Verilog-2005 friendly)
          localparam S_IDLE = 2'd0;
          localparam S_RUN  = 2'd1;
          localparam S_RESP = 2'd2;


          reg [1:0] cs, ns;

          // state register
          always @(posedge clk)
          begin
            if (reset)
              cs <= S_IDLE;
            else
              cs <= ns;
          end

          // default outputs and next-state logic
          always @*
          begin
            // defaults
            a_mux_sel       = 1'b0;
            a_shift_en      = 1'b0;
            b_mux_sel       = 1'b0;
            b_shift_en      = 1'b0;
            result_mux_sel  = 1'b0;
            result_en       = 1'b0;
            counter_en      = 1'b0;
            counter_mux_sel = 1'b0;
            sign_en         = 1'b0;
            sign_mux_sel    = 1'b0;
            add_mux_sel     = 1'b0;

            req_rdy_o       = 1'b0;
            resp_val_o      = 1'b0;

            ns = cs;

            case (cs)

              // ---------------- IDLE ----------------
              S_IDLE:
              begin
                req_rdy_o = 1'b1;
                if (mulreq_val_i && req_rdy_o)
                begin
                  // ★ 把你原本 S_LOAD 的內容逐行搬進來（數值保持一致）
                  // 例如（名字可能和你檔案略有不同，請以你原本 S_LOAD 為準）：
                  a_mux_sel        = 1'b0;
                  b_mux_sel        = 1'b0;
                  result_en        = 1'b1;          // 若你原本用 counter_init 清 result，也照原樣
                  counter_en       = 1'b1;
                  counter_mux_sel  = 1'b0;
                  sign_en          = 1'b1;
                  sign_mux_sel     = 1'b0;

                  ns = S_RUN;
                end
              end


              // ---------------- RUN (32 cycles) ----------------
              S_RUN:
              begin
                // shift A/B every cycle
                a_mux_sel  = 1'b1;
                a_shift_en = 1'b1;  // A <<= 1
                b_mux_sel  = 1'b1;
                b_shift_en = 1'b1;  // B >>= 1

                // conditional add when LSB of B is 1
                if (b0)
                begin
                  result_en       = 1'b1;
                  result_mux_sel  = 1'b1;               // take add_mux_out
                  add_mux_sel     = 1'b1;               // Result + A
                end

                // counter--
                counter_en      = 1'b1;
                counter_mux_sel = 1'b1;

                if (counter_is_zero)
                  ns = S_RESP;
              end

              // ---------------- RESP ----------------
              S_RESP:
              begin
                resp_val_o = 1'b1;               // present a valid result
                if (mulresp_rdy_i)
                  ns = S_IDLE;
              end

              default:
                ns = S_IDLE;
            endcase
          end

        endmodule



`endif
