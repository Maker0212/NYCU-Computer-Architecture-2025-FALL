# mulhu：兩個都無號，做 64-bit 乘積後取高半

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        # 基本值
        TEST_RR_OP( mulhu, 0x00000000, 0x00000000, 0x00000000 )  # 0*0 → hi=0
        TEST_RR_OP( mulhu, 0x00000001, 0x00000001, 0x00000000 )  # 1*1 → hi=0

        # 重要邊界：max 與 2^31
        TEST_RR_OP( mulhu, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE )  # max^2 → hi=0xFFFFFFFE
        TEST_RR_OP( mulhu, 0x80000000, 0x80000000, 0x40000000 )  # 2^31*2^31=2^62 → hi=0x40000000

        # 跨 32-bit 的乾淨例子
        TEST_RR_OP( mulhu, 0x00010000, 0x00010000, 0x00000001 )  # 2^16*2^16 → hi=1

        # 小心邊界：接近溢位但仍可心算
        TEST_RR_OP( mulhu, 0xFFFFFFFF, 0x00000002, 0x00000001 )  # max*2=0x1_FFFFFFFE → hi=1
        TEST_RR_OP( mulhu, 0x7FFFFFFF, 0x00000002, 0x00000000 )  # (2^31-1)*2 → hi=0

        # 隨機常數（可交換：mulhu 交換操作數結果相同）
        TEST_RR_OP( mulhu, 0x12345678, 0x9ABCDEF0, 0x0B00EA4E )  # 雜湊大數 → hi=0x0B00EA4E
        TEST_RR_OP( mulhu, 0x9ABCDEF0, 0x12345678, 0x0B00EA4E )  # 交換後一致

        # 其他隨機大數
        TEST_RR_OP( mulhu, 0xFFFFFFFF, 0x00000001, 0x00000000 )  # max*1 → hi=0
        TEST_RR_OP( mulhu, 0xFFFFFFFE, 0x00000003, 0x00000002 )  # (2^32-2)*3 = 0x2_FFFFFFFA → hi=2
        TEST_RR_OP( mulhu, 0xDEADBEEF, 0xC001D00D, 0xA703E2D9 )  # 隨機大數 → hi=0xA703E2D9




        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x00000001, 0x00000000 )

        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x00000010, 0x00000008 )

        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x00000100, 0x00000080 )

        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x00001000, 0x00000800 )

        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x00010000, 0x00008000 )

        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x00100000, 0x00080000 )

        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x01000000, 0x00800000 )

        # 2^31 * bit_k
        TEST_RR_OP( mulhu, 0x80000000, 0x10000000, 0x08000000 )

        # 隨機案例
        TEST_RR_OP( mulhu, 0x386ECBE0, 0x72FF5D2A, 0x1959A1B0 )

        # 隨機案例
        TEST_RR_OP( mulhu, 0x96DA1DAC, 0x47378190, 0x29F7336F )

        # 隨機案例
        TEST_RR_OP( mulhu, 0xCF36D58B, 0xDE8A774B, 0xB421A14C )

        # 隨機案例
        TEST_RR_OP( mulhu, 0x01A9E71F, 0xC241330B, 0x01432D9E )

        # 隨機案例
        TEST_RR_OP( mulhu, 0xCE4A2BBD, 0x28DF6EC4, 0x20EFA2B3 )

        # 隨機案例
        TEST_RR_OP( mulhu, 0xB2B9437A, 0x6C307511, 0x4B87FCEE )

        # 隨機案例
        TEST_RR_OP( mulhu, 0x571AA876, 0x47229389, 0x18342876 )

        # 隨機案例
        TEST_RR_OP( mulhu, 0x27CD8130, 0x371ECD7B, 0x0891F0C9 )

        TEST_RISCV_END
