# mulh：把兩個 32-bit 當有號數相乘，得到 64-bit 兩補數乘積，取 prod[63:32]

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        # 基本值與邊界
        TEST_RR_OP( mulh, 0x00000000, 0x00000000, 0x00000000 )  # 0*0 → hi=0
        TEST_RR_OP( mulh, 0x00000001, 0x00000001, 0x00000000 )  # 1*1 → 1 → hi=0
        TEST_RR_OP( mulh, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000 )  # (-1)*(-1)=1 → hi=0
        TEST_RR_OP( mulh, 0xFFFFFFFF, 0x00000001, 0xFFFFFFFF )  # (-1)*(+1)=-1 → hi=0xFFFFFFFF

        # 重要邊界：-2^31 與大數
        TEST_RR_OP( mulh, 0x80000000, 0x80000000, 0x40000000 )  # (-2^31)^2=2^62 → hi=0x40000000
        TEST_RR_OP( mulh, 0x80000000, 0x00000002, 0xFFFFFFFF )  # (-2^31)*2=-2^32 → hi=0xFFFFFFFF
        TEST_RR_OP( mulh, 0x7FFFFFFF, 0x00000002, 0x00000000 )  # (2^31-1)*2=0x00000000_FFFFFFFE → hi=0

        # 乘積跨 32-bit 邊界的乾淨例子（方便心算）
        TEST_RR_OP( mulh, 0x00010000, 0x00010000, 0x00000001 )  # 2^16*2^16=2^32 → hi=1
        TEST_RR_OP( mulh, 0x7FFFFFFF, 0x7FFFFFFF, 0x3FFFFFFF )  # (2^31-1)^2 → hi=0x3FFFFFFF

        # 隨機常數（對稱性：mulh 是可交換的，交換操作數應得相同 hi）
        TEST_RR_OP( mulh, 0x12345678, 0x9ABCDEF0, 0xF8CC93D6 )  # 雜湊大數 → hi=0xF8CC93D6
        TEST_RR_OP( mulh, 0x9ABCDEF0, 0x12345678, 0xF8CC93D6 )  # 交換後結果相同

        # 符號測試
        TEST_RR_OP( mulh, 0xFFFFFFFE, 0x00000003, 0xFFFFFFFF )  # (-2)*3=-6 → hi=0xFFFFFFFF
        TEST_RR_OP( mulh, 0x00000003, 0xFFFFFFFE, 0xFFFFFFFF )  # 交換符號 → 結果相同


        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x00000001, 0xFFFFFFFF )

        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x00000010, 0xFFFFFFF8 )

        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x00000100, 0xFFFFFF80 )

        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x00001000, 0xFFFFF800 )

        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x00010000, 0xFFFF8000 )

        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x00100000, 0xFFF80000 )

        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x01000000, 0xFF800000 )

        # (-2^31)*bit_k
        TEST_RR_OP( mulh, 0x80000000, 0x10000000, 0xF8000000 )

        # 隨機案例
        TEST_RR_OP( mulh, 0xA3B1799D, 0x1C80317F, 0xF5B93031 )

        # 隨機案例
        TEST_RR_OP( mulh, 0x06671AD1, 0xBDD640FB, 0xFE585FCB )

        # 隨機案例
        TEST_RR_OP( mulh, 0x46685257, 0x3EB13B90, 0x113E0273 )

        # 隨機案例
        TEST_RR_OP( mulh, 0x392456DE, 0x23B8C1E9, 0x07F93547 )

        # 隨機案例
        TEST_RR_OP( mulh, 0xBC8960A9, 0x1A3D1FA7, 0xF915D835 )

        # 隨機案例
        TEST_RR_OP( mulh, 0xAD3C2D6D, 0xBD9C66B3, 0x1576AF94 )

        # 隨機案例
        TEST_RR_OP( mulh, 0xE465E150, 0x8B9D2434, 0x0C8C7E9A )

        # 隨機案例
        TEST_RR_OP( mulh, 0x16419F82, 0x972A8469, 0xF6E2C7DA )

        TEST_RISCV_END
