# mulhsu：左操作數有號、右操作數無號，做 64-bit 乘積後取高半

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        # 常見邊界（負 × 大無號）
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x80000000, 0xFFFFFFFF )  # (-1)*2^31u = 0xFFFFFFFF_80000000 → hi=0xFFFFFFFF
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF )  # (-1)*maxu = 0xFFFFFFFF_FFFFFFFF → hi=0xFFFFFFFF
        TEST_RR_OP( mulhsu, 0xFFFFFFFE, 0x00000003, 0xFFFFFFFF )  # (-2)*3u = 0xFFFFFFFF_FFFFFFFA → hi=0xFFFFFFFF

        # 0、1 的 sanity check
        TEST_RR_OP( mulhsu, 0x00000000, 0x075BCD15, 0x00000000 )  # 0*X → hi=0
        TEST_RR_OP( mulhsu, 0x00000000, 0x00000000, 0x00000000 )  # 0*0 → hi=0
        TEST_RR_OP( mulhsu, 0x00000001, 0xFFFFFFFF, 0x00000000 )  # 1*maxu → 0x00000000_FFFFFFFF → hi=0

        # 關鍵大邊界：(-2^31) × (2^31u)
        TEST_RR_OP( mulhsu, 0x80000000, 0x80000000, 0xC0000000 )  # = 0xC000000000000000 → hi=0xC0000000

        # 其他邊界與易錯例
        TEST_RR_OP( mulhsu, 0x7FFFFFFF, 0x00000002, 0x00000000 )  # (2^31-1)*2u → hi=0
        TEST_RR_OP( mulhsu, 0x40000000, 0x00000002, 0x00000000 )  # 2^30*2u=2^31 → hi=0
        TEST_RR_OP( mulhsu, 0x00010000, 0x00010000, 0x00000001 )  # 2^16*2^16 → hi=1

        # 單位無號 × 極端負數
        TEST_RR_OP( mulhsu, 0x80000000, 0x00000001, 0xFFFFFFFF )  # (-2^31)*1u → hi=0xFFFFFFFF
        TEST_RR_OP( mulhsu, 0x80000000, 0x00000002, 0xFFFFFFFF )  # (-2^31)*2u=-2^32 → hi=0xFFFFFFFF
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00000001, 0xFFFFFFFF )  # (-1)*1u=-1 → hi=0xFFFFFFFF
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00000000, 0x00000000 )  # (-1)*0u=0 → hi=0

        # 大無號 × 大正數（正值結果，方便對照）
        TEST_RR_OP( mulhsu, 0x7FFFFFFF, 0xFFFFFFFF, 0x7FFFFFFE )  # (2^31-1)*maxu → hi=0x7FFFFFFE




        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00000001, 0xFFFFFFFF )

        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00000010, 0xFFFFFFFF )

        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00000100, 0xFFFFFFFF )

        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00001000, 0xFFFFFFFF )

        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00010000, 0xFFFFFFFF )

        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x00100000, 0xFFFFFFFF )

        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x01000000, 0xFFFFFFFF )

        # (-1)*bit_k(u)
        TEST_RR_OP( mulhsu, 0xFFFFFFFF, 0x10000000, 0xFFFFFFFF )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0x6C031199, 0x0822E8F3, 0x036ED33E )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0x07A0CA6E, 0x17FC695A, 0x00B6F79A )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0x37F8A88B, 0x3B8FAA18, 0x0D05B7EE )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0x815EF6D1, 0x9A1DE644, 0xB3C45655 )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0x06CB0FB3, 0x8FADC1A6, 0x03D00A22 )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0x32E70629, 0xB74D0FB1, 0x24727802 )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0xA65ED389, 0xB38A088C, 0xC123FA08 )

        # 隨機案例
        TEST_RR_OP( mulhsu, 0x8B8148F6, 0x6B65A6A4, 0xCF20C7AB )
        TEST_RISCV_END
