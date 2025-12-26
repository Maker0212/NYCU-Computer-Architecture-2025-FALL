`ifndef RISCV_CORE_SCOREBOARD_V
`define RISCV_CORE_SCOREBOARD_V

        module riscv_CoreScoreboard
          (
            input            clk,
            input            reset,

            input            inst_val_Dhl,

            input      [4:0] src00, input src00_en,
            input      [4:0] src01, input src01_en,
            input      [4:0] src10, input src10_en,
            input      [4:0] src11, input src11_en,

            output           stall_0_hazard,
            output           stall_1_hazard,

            output reg [3:0] src00_byp_mux_sel,
            output reg [3:0] src01_byp_mux_sel,
            output reg [3:0] src10_byp_mux_sel,
            output reg [3:0] src11_byp_mux_sel,

            input      [4:0] dstA,
            input            dstA_en,
            input            stall_A_Dhl,
            input            is_muldiv_A,
            input            is_load_A,

            input      [4:0] dstB,
            input            dstB_en,
            input            stall_B_Dhl,
            input            is_muldiv_B,
            input            is_load_B,

            input            stall_X0hl,
            input            stall_X1hl,

            input            wbA_wen,
            input [4:0] wbA_dst,
            input            wbB_wen,
            input [4:0] wbB_dst
          );

          localparam [3:0] BYP_RF   = 4'd0;

          localparam [3:0] BYP_A_X0 = 4'd1;
          localparam [3:0] BYP_A_X1 = 4'd2;
          localparam [3:0] BYP_A_X2 = 4'd3;
          localparam [3:0] BYP_A_X3 = 4'd4;
          localparam [3:0] BYP_A_W  = 4'd5;

          localparam [3:0] BYP_B_X0 = 4'd6;
          localparam [3:0] BYP_B_X1 = 4'd7;
          localparam [3:0] BYP_B_X2 = 4'd8;
          localparam [3:0] BYP_B_X3 = 4'd9;
          localparam [3:0] BYP_B_W  = 4'd10;

          reg [4:0] stage    [31:0];
          reg       is_ld    [31:0];
          reg       is_md    [31:0];
          reg       prod_isB [31:0];

          integer i;

          function is_at_stage;
            input [4:0] stage_bits;
            input [2:0] stage_num;
            begin
              is_at_stage = stage_bits[stage_num];
            end
          endfunction

          function [2:0] get_highest_stage;
            input [4:0] stage_bits;
            begin
              case (1'b1)
                stage_bits[4]:
                  get_highest_stage = 3'd4;
                stage_bits[3]:
                  get_highest_stage = 3'd3;
                stage_bits[2]:
                  get_highest_stage = 3'd2;
                stage_bits[1]:
                  get_highest_stage = 3'd1;
                stage_bits[0]:
                  get_highest_stage = 3'd0;
                default:
                  get_highest_stage = 3'd7;
              endcase
            end
          endfunction

          function has_pending_regs;
            input [4:0] stage_bits;
            begin
              has_pending_regs = (stage_bits != 5'b0);
            end
          endfunction

          function can_bypass_to_destination;
            input [4:0] stage_bits;
            input is_load;
            input is_muldiv;
            reg [2:0] from_stage;
            begin
              from_stage = get_highest_stage(stage_bits);
              case (from_stage)
                3'd0:
                  can_bypass_to_destination = (!is_load && !is_muldiv);
                3'd1:
                  can_bypass_to_destination = (!is_muldiv);
                3'd2:
                  can_bypass_to_destination = (!is_muldiv);
                3'd3:
                  can_bypass_to_destination = 1'b1;
                3'd4:
                  can_bypass_to_destination = 1'b1;
                default:
                  can_bypass_to_destination = 1'b0;
              endcase
            end
          endfunction

          function is_in_writeback_stage;
            input [4:0] stage_bits;
            begin
              is_in_writeback_stage = stage_bits[4];
            end
          endfunction

          task scan_pending_registers;
            input [4:0] check_reg;
            output reg [4:0] found_stage;
            output reg found_valid;
            begin
              found_valid = 1'b0;
              found_stage = 5'b0;
              case (check_reg != 5'd0 && stage[check_reg] != 5'b0)
                1'b1:
                begin
                  found_valid = 1'b1;
                  found_stage = stage[check_reg];
                end
              endcase
            end
          endtask

          task insert_into_X0 (
              input [4:0] rd,
              input       rd_en,
              input       isload,
              input       ismuldiv,
              input       isB_producer
            );
            begin
              case (rd_en && (rd!=5'd0))
                1'b1:
                begin
                  stage[rd]    <= 5'b00001;
                  is_ld[rd]    <= isload;
                  is_md[rd]    <= ismuldiv;
                  prod_isB[rd] <= isB_producer;
                end
              endcase
            end
          endtask

          reg [4:0] cur;
          reg [4:0] nxt;
          integer j;

          always @(posedge clk or posedge reset)
          begin
            case (reset)
              1'b1:
              begin
                for (i=0; i<32; i=i+1)
                begin
                  stage[i]    <= 5'b0;
                  is_ld[i]    <= 1'b0;
                  is_md[i]    <= 1'b0;
                  prod_isB[i] <= 1'b0;
                end
              end
              1'b0:
              begin
                for (j=0; j<32; j=j+1)
                begin
                  cur = stage[j];
                  nxt = 5'b0;

                  if (cur[3])
                    nxt[4] = 1'b1;

                  if (cur[2])
                    nxt[3] = 1'b1;

                  if (cur[1])
                  begin
                    case (stall_X1hl)
                      1'b0:
                        nxt[2] = 1'b1;
                      1'b1:
                        nxt[1] = 1'b1;
                    endcase
                  end

                  if (cur[0])
                  begin
                    case ({stall_X1hl, stall_X0hl})
                      2'b00:
                        nxt[1] = 1'b1;
                      default:
                        nxt[0] = 1'b1;
                    endcase
                  end

                  stage[j] <= nxt;

                  case (nxt==5'b0)
                    1'b1:
                    begin
                      is_ld[j]    <= 1'b0;
                      is_md[j]    <= 1'b0;
                      prod_isB[j] <= 1'b0;
                    end
                  endcase
                end

                case (!stall_A_Dhl && !stall_X0hl && inst_val_Dhl && dstA_en && dstA!=5'd0 && (stage[dstA]==5'b0))
                  1'b1:
                    insert_into_X0(dstA, 1'b1, is_load_A, is_muldiv_A, 1'b0);
                endcase

                case (!stall_B_Dhl && !stall_X0hl && inst_val_Dhl && dstB_en && dstB!=5'd0 && (stage[dstB]==5'b0))
                  1'b1:
                    insert_into_X0(dstB, 1'b1, is_load_B, is_muldiv_B, 1'b1);
                endcase
              end
            endcase
          end

          function can_bypass_from_X0;
            input isload;
            input ismuldiv;
            begin
              can_bypass_from_X0 = (!isload && !ismuldiv);
            end
          endfunction
          function can_bypass_from_X1;
            input isload;
            input ismuldiv;
            begin
              can_bypass_from_X1 = (!ismuldiv);
            end
          endfunction
          function can_bypass_from_X2;
            input isload;
            input ismuldiv;
            begin
              can_bypass_from_X2 = (!ismuldiv);
            end
          endfunction
          function can_bypass_from_X3;
            input isload;
            input ismuldiv;
            begin
              can_bypass_from_X3 = 1'b1;
            end
          endfunction

          function [3:0] get_bypass_mux_for_stage;
            input [2:0] stage_idx;
            input isBprod;
            reg [3:0] a_lane_sel [4:0];
            reg [3:0] b_lane_sel [4:0];
            begin
              a_lane_sel[0] = BYP_A_X0;
              a_lane_sel[1] = BYP_A_X1;
              a_lane_sel[2] = BYP_A_X2;
              a_lane_sel[3] = BYP_A_X3;
              a_lane_sel[4] = BYP_A_W;

              b_lane_sel[0] = BYP_B_X0;
              b_lane_sel[1] = BYP_B_X1;
              b_lane_sel[2] = BYP_B_X2;
              b_lane_sel[3] = BYP_B_X3;
              b_lane_sel[4] = BYP_B_W;

              case (stage_idx < 5)
                1'b1:
                  get_bypass_mux_for_stage = isBprod ? b_lane_sel[stage_idx] : a_lane_sel[stage_idx];
                1'b0:
                  get_bypass_mux_for_stage = BYP_RF;
              endcase
            end
          endfunction

          function [3:0] bypass_mux_X0;
            input isBprod;
            begin
              bypass_mux_X0 = get_bypass_mux_for_stage(3'd0, isBprod);
            end
          endfunction

          function [3:0] bypass_mux_X1;
            input isBprod;
            begin
              bypass_mux_X1 = get_bypass_mux_for_stage(3'd1, isBprod);
            end
          endfunction

          function [3:0] bypass_mux_X2;
            input isBprod;
            begin
              bypass_mux_X2 = get_bypass_mux_for_stage(3'd2, isBprod);
            end
          endfunction

          function [3:0] bypass_mux_X3;
            input isBprod;
            begin
              bypass_mux_X3 = get_bypass_mux_for_stage(3'd3, isBprod);
            end
          endfunction

          function [3:0] bypass_mux_W;
            input isBprod;
            begin
              bypass_mux_W = get_bypass_mux_for_stage(3'd4, isBprod);
            end
          endfunction

          function [4:0] resolve_X3_bypass;
            input isload;
            input ismd;
            input isBprod;
            reg stall;
            reg [3:0] byp;
            begin
              case (can_bypass_from_X3(isload, ismd))
                1'b1:
                begin
                  stall = 1'b0;
                  byp = get_bypass_mux_for_stage(3'd3, isBprod);
                end
                1'b0:
                begin
                  stall = 1'b1;
                  byp = BYP_RF;
                end
              endcase
              resolve_X3_bypass = {stall, byp};
            end
          endfunction

          function [4:0] resolve_X2_bypass;
            input isload;
            input ismd;
            input isBprod;
            reg stall;
            reg [3:0] byp;
            begin
              case (can_bypass_from_X2(isload, ismd))
                1'b1:
                begin
                  stall = 1'b0;
                  byp = bypass_mux_X2(isBprod);
                end
                1'b0:
                begin
                  stall = 1'b1;
                  byp = BYP_RF;
                end
              endcase
              resolve_X2_bypass = {stall, byp};
            end
          endfunction

          function [4:0] resolve_X1_bypass;
            input isload;
            input ismd;
            input isBprod;
            reg stall;
            reg [3:0] byp;
            begin
              case (can_bypass_from_X1(isload, ismd))
                1'b1:
                begin
                  stall = 1'b0;
                  byp = bypass_mux_X1(isBprod);
                end
                1'b0:
                begin
                  stall = 1'b1;
                  byp = BYP_RF;
                end
              endcase
              resolve_X1_bypass = {stall, byp};
            end
          endfunction

          function [4:0] resolve_X0_bypass;
            input isload;
            input ismd;
            input isBprod;
            reg stall;
            reg [3:0] byp;
            begin
              case (can_bypass_from_X0(isload, ismd))
                1'b1:
                begin
                  stall = 1'b0;
                  byp = bypass_mux_X0(isBprod);
                end
                1'b0:
                begin
                  stall = 1'b1;
                  byp = BYP_RF;
                end
              endcase
              resolve_X0_bypass = {stall, byp};
            end
          endfunction

          function [4:0] resolve_src_f;
            input        valid;
            input  [4:0] stg;
            input        isload;
            input        ismd;
            input        isBprod;
            reg          stall;
            reg  [3:0]   byp;
            begin
              stall = 1'b0;
              byp   = BYP_RF;

              case (valid)
                1'b0:
                begin
                  resolve_src_f = {stall, byp};
                end
                1'b1:
                begin
                  case (1'b1)
                    stg[4]:
                    begin
                      stall = 1'b0;
                      byp   = bypass_mux_W(isBprod);
                    end
                    stg[3]:
                    begin
                      {stall, byp} = resolve_X3_bypass(isload, ismd, isBprod);
                    end
                    stg[2]:
                    begin
                      {stall, byp} = resolve_X2_bypass(isload, ismd, isBprod);
                    end
                    stg[1]:
                    begin
                      {stall, byp} = resolve_X1_bypass(isload, ismd, isBprod);
                    end
                    stg[0]:
                    begin
                      {stall, byp} = resolve_X0_bypass(isload, ismd, isBprod);
                    end
                    default:
                    begin
                      stall = 1'b0;
                      byp   = BYP_RF;
                    end
                  endcase

                  resolve_src_f = {stall, byp};
                end
              endcase
            end
          endfunction

          function [3:0] compress_sel;
            input [3:0] raw;
            begin
              case (raw)
                BYP_A_W,  BYP_B_W:
                  compress_sel = 4'd1;
                BYP_A_X1, BYP_A_X2, BYP_B_X1, BYP_B_X2:
                  compress_sel = 4'd2;
                BYP_A_X0, BYP_A_X3, BYP_B_X0, BYP_B_X3:
                  compress_sel = 4'd3;
                default:
                  compress_sel = 4'd0;
              endcase
            end
          endfunction

          task update_stg_for_A_producer;
            inout [4:0] stg;
            inout       isB;
            inout       ld;
            inout       md;
            input [4:0] check_reg;
            input       valid;
            begin
              case (valid && a_will_issue && (dstA!=5'd0) && (check_reg==dstA))
                1'b1:
                begin
                  stg = 5'b00001;
                  isB = 1'b0;
                  ld = is_load_A;
                  md = is_muldiv_A;
                end
              endcase
            end
          endtask

          task update_stg_for_B_producer;
            inout [4:0] stg;
            inout       isB;
            inout       ld;
            inout       md;
            input [4:0] check_reg;
            input       valid;
            begin
              case (valid && b_will_issue && (dstB!=5'd0) && (check_reg==dstB))
                1'b1:
                begin
                  stg = 5'b00001;
                  isB = 1'b1;
                  ld = is_load_B;
                  md = is_muldiv_B;
                end
              endcase
            end
          endtask

          task apply_wb_instant_hit;
            inout        stall;
            inout [3:0]  byp;
            input        hitWB;
            input        wbA_valid;
            input [4:0]  wbA_dst;
            input        wbB_valid;
            input [4:0]  wbB_dst;
            input [4:0]  src_reg;
            begin
              case (hitWB)
                1'b1:
                begin
                  stall = 1'b0;
                  case (wbA_valid && (wbA_dst==src_reg))
                    1'b1:
                      byp = BYP_A_W;
                    1'b0:
                    case (wbB_valid && (wbB_dst==src_reg))
                      1'b1:
                        byp = BYP_B_W;
                    endcase
                  endcase
                end
              endcase
            end
          endtask

          function check_wb_instant_hit;
            input        src_valid;
            input [4:0]  src_reg;
            input        wbA_wen;
            input [4:0]  wbA_dst;
            input        wbB_wen;
            input [4:0]  wbB_dst;
            begin
              check_wb_instant_hit = (src_valid && (src_reg!=5'd0)) &&
                                   ((wbA_wen && (wbA_dst==src_reg)) ||
                                    (wbB_wen && (wbB_dst==src_reg)));
            end
          endfunction

          function detect_waw_conflict;
            input a_will_issue;
            input b_will_issue;
            input [4:0] dstA;
            input [4:0] dstB;
            begin
              detect_waw_conflict = a_will_issue && b_will_issue &&
                                  (dstA!=5'd0) && (dstA==dstB);
            end
          endfunction

          function detect_raw1_conflict;
            input a_will_issue;
            input [4:0] dstA;
            input src10_valid;
            input [4:0] src10;
            input src11_valid;
            input [4:0] src11;
            begin
              detect_raw1_conflict = a_will_issue && (dstA!=5'd0) &&
                                   ((src10_valid && (src10==dstA)) ||
                                    (src11_valid && (src11==dstA)));
            end
          endfunction

          function detect_raw0_conflict;
            input b_will_issue;
            input [4:0] dstB;
            input src00_valid;
            input [4:0] src00;
            input src01_valid;
            input [4:0] src01;
            begin
              detect_raw0_conflict = b_will_issue && (dstB!=5'd0) &&
                                   ((src00_valid && (src00==dstB)) ||
                                    (src01_valid && (src01==dstB)));
            end
          endfunction

          function is_dst_busy;
            input [4:0] dst;
            input dst_en;
            input [4:0] stage_bits;
            begin
              is_dst_busy = (dst_en && (dst!=5'd0) && (stage_bits!=5'b0));
            end
          endfunction

          function dst_conflicts_with_A;
            input b_will_issue;
            input a_will_issue;
            input [4:0] dstB;
            input [4:0] dstA;
            begin
              dst_conflicts_with_A = b_will_issue && a_will_issue &&
                                   (dstB==dstA) && (dstB!=5'd0);
            end
          endfunction

          reg         stall00, stall01, stall10, stall11;
          reg  [3:0]  byp00,   byp01,   byp10,   byp11;

          reg  [4:0]  stg00, stg01, stg10, stg11;
          reg         ld00,  ld01,  ld10,  ld11;
          reg         md00,  md01,  md10,  md11;
          reg         isB00, isB01, isB10, isB11;

          wire a_will_issue = inst_val_Dhl && dstA_en && !stall_A_Dhl && !stall_X0hl;
          wire b_will_issue = inst_val_Dhl && dstB_en && !stall_B_Dhl && !stall_X0hl;

          wire pair_waw  = a_will_issue && b_will_issue && (dstA!=5'd0) && (dstA==dstB);
          wire pair_raw1 = a_will_issue && (dstA!=5'd0) &&
               ((src10_en && (src10==dstA)) || (src11_en && (src11==dstA)));
          wire pair_raw0 = b_will_issue && (dstB!=5'd0) &&
               ((src00_en && (src00==dstB)) || (src01_en && (src01==dstB)));

          wire hitWB00 = (src00_en && (src00!=5'd0)) &&
               ((wbA_wen && (wbA_dst==src00)) || (wbB_wen && (wbB_dst==src00)));
          wire hitWB01 = (src01_en && (src01!=5'd0)) &&
               ((wbA_wen && (wbA_dst==src01)) || (wbB_wen && (wbB_dst==src01)));
          wire hitWB10 = (src10_en && (src10!=5'd0)) &&
               ((wbA_wen && (wbA_dst==src10)) || (wbB_wen && (wbB_dst==src10)));
          wire hitWB11 = (src11_en && (src11!=5'd0)) &&
               ((wbA_wen && (wbA_dst==src11)) || (wbB_wen && (wbB_dst==src11)));

          wire dstA_busy_now = (dstA_en && dstA!=5'd0 && stage[dstA]!=5'b0);
          wire dstB_busy_now = (dstB_en && dstB!=5'd0 && stage[dstB]!=5'b0);
          wire dstB_conflict_with_A_issue = b_will_issue && a_will_issue && (dstB==dstA) && (dstB!=5'd0);

          wire dstA_hazard = a_will_issue && dstA_busy_now;
          wire dstB_hazard = b_will_issue && (dstB_busy_now || dstB_conflict_with_A_issue);

          always @*
          begin
            stg00 = (src00_en && (src00!=5'd0)) ? stage[src00]    : 5'b0;
            stg01 = (src01_en && (src01!=5'd0)) ? stage[src01]    : 5'b0;
            stg10 = (src10_en && (src10!=5'd0)) ? stage[src10]    : 5'b0;
            stg11 = (src11_en && (src11!=5'd0)) ? stage[src11]    : 5'b0;

            ld00  = (src00_en && (src00!=5'd0)) ? is_ld[src00]    : 1'b0;
            ld01  = (src01_en && (src01!=5'd0)) ? is_ld[src01]    : 1'b0;
            ld10  = (src10_en && (src10!=5'd0)) ? is_ld[src10]    : 1'b0;
            ld11  = (src11_en && (src11!=5'd0)) ? is_ld[src11]    : 1'b0;

            md00  = (src00_en && (src00!=5'd0)) ? is_md[src00]    : 1'b0;
            md01  = (src01_en && (src01!=5'd0)) ? is_md[src01]    : 1'b0;
            md10  = (src10_en && (src10!=5'd0)) ? is_md[src10]    : 1'b0;
            md11  = (src11_en && (src11!=5'd0)) ? is_md[src11]    : 1'b0;

            isB00 = (src00_en && (src00!=5'd0)) ? prod_isB[src00] : 1'b0;
            isB01 = (src01_en && (src01!=5'd0)) ? prod_isB[src01] : 1'b0;
            isB10 = (src10_en && (src10!=5'd0)) ? prod_isB[src10] : 1'b0;
            isB11 = (src11_en && (src11!=5'd0)) ? prod_isB[src11] : 1'b0;

            case (1'b1)
              (src00_en && (src00!=5'd0) && inst_val_Dhl && a_will_issue && (dstA!=5'd0) && (src00==dstA)):
              begin
                stg00 = 5'b00001;
                isB00 = 1'b0;
                ld00 = is_load_A;
                md00 = is_muldiv_A;
              end
              (src00_en && (src00!=5'd0) && inst_val_Dhl && b_will_issue && (dstB!=5'd0) && (src00==dstB)):
              begin
                stg00 = 5'b00001;
                isB00 = 1'b1;
                ld00 = is_load_B;
                md00 = is_muldiv_B;
              end
            endcase

            case (1'b1)
              (src01_en && (src01!=5'd0) && inst_val_Dhl && a_will_issue && (dstA!=5'd0) && (src01==dstA)):
              begin
                stg01 = 5'b00001;
                isB01 = 1'b0;
                ld01 = is_load_A;
                md01 = is_muldiv_A;
              end
              (src01_en && (src01!=5'd0) && inst_val_Dhl && b_will_issue && (dstB!=5'd0) && (src01==dstB)):
              begin
                stg01 = 5'b00001;
                isB01 = 1'b1;
                ld01 = is_load_B;
                md01 = is_muldiv_B;
              end
            endcase

            case (1'b1)
              (src10_en && (src10!=5'd0) && inst_val_Dhl && a_will_issue && (dstA!=5'd0) && (src10==dstA)):
              begin
                stg10 = 5'b00001;
                isB10 = 1'b0;
                ld10 = is_load_A;
                md10 = is_muldiv_A;
              end
              (src10_en && (src10!=5'd0) && inst_val_Dhl && b_will_issue && (dstB!=5'd0) && (src10==dstB)):
              begin
                stg10 = 5'b00001;
                isB10 = 1'b1;
                ld10 = is_load_B;
                md10 = is_muldiv_B;
              end
            endcase

            case (1'b1)
              (src11_en && (src11!=5'd0) && inst_val_Dhl && a_will_issue && (dstA!=5'd0) && (src11==dstA)):
              begin
                stg11 = 5'b00001;
                isB11 = 1'b0;
                ld11 = is_load_A;
                md11 = is_muldiv_A;
              end
              (src11_en && (src11!=5'd0) && inst_val_Dhl && b_will_issue && (dstB!=5'd0) && (src11==dstB)):
              begin
                stg11 = 5'b00001;
                isB11 = 1'b1;
                ld11 = is_load_B;
                md11 = is_muldiv_B;
              end
            endcase

            {stall00, byp00} = resolve_src_f(src00_en && (src00!=5'd0), stg00, ld00, md00, isB00);
            {stall01, byp01} = resolve_src_f(src01_en && (src01!=5'd0), stg01, ld01, md01, isB01);
            {stall10, byp10} = resolve_src_f(src10_en && (src10!=5'd0), stg10, ld10, md10, isB10);
            {stall11, byp11} = resolve_src_f(src11_en && (src11!=5'd0), stg11, ld11, md11, isB11);

            case (1'b1)
              hitWB00:
              begin
                stall00 = 1'b0;
                byp00   = (wbA_wen && (wbA_dst==src00)) ? BYP_A_W : BYP_B_W;
              end
            endcase

            case (1'b1)
              hitWB01:
              begin
                stall01 = 1'b0;
                byp01   = (wbA_wen && (wbA_dst==src01)) ? BYP_A_W : BYP_B_W;
              end
            endcase

            case (1'b1)
              hitWB10:
              begin
                stall10 = 1'b0;
                byp10   = (wbA_wen && (wbA_dst==src10)) ? BYP_A_W : BYP_B_W;
              end
            endcase

            case (1'b1)
              hitWB11:
              begin
                stall11 = 1'b0;
                byp11   = (wbA_wen && (wbA_dst==src11)) ? BYP_A_W : BYP_B_W;
              end
            endcase

            src00_byp_mux_sel = byp00;
            src01_byp_mux_sel = byp01;
            src10_byp_mux_sel = byp10;
            src11_byp_mux_sel = byp11;
          end

          assign stall_0_hazard = (stall00 | stall01) | pair_raw0 | dstA_hazard;
          assign stall_1_hazard = (stall10 | stall11) | pair_waw | pair_raw1 | dstB_hazard;

        endmodule
`endif
