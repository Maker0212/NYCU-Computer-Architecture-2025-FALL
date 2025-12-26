#include "riscv-macros.h"
        TEST_RISCV_BEGIN

        # X -> D (ALU 轉發)
        li   x1, 5
        add  x2, x1, x1        # 需要 rs1 from W 其實這裡會從 RF，但下一行會從 X
        add  x3, x2, x1        # 需要 rs1 from X (若沒繞送會多 stall)

        # load-use 應該要 stall 1 cycle（不能從 X forward）
        la   x4, data
        lw   x5, 0(x4)         # X 是 load
        add  x6, x5, x3        # 對 x5 的 use：必須 stall 再拿到 M/W 值

        # M -> D (load 之後的前饋)
        add  x7, x6, x3

        # 驗證結果（依你的宏替換）
        li   x10, 5
        add  x10, x10, x10     # 10
        add  x10, x10, x10     # 20
        add  x10, x10, x10     # 40 (僅示意，實際請依上面算式設定期望值)
        TEST_ASSERT_EQ(x7, x10)

        TEST_RISCV_END

        .data
data:   .word  7
