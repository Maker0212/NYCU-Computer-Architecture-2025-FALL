//========================================================================
// riscv-mulh.S
//========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //--------------------------------------------------------------------
        // Arithmetic tests
        //--------------------------------------------------------------------

        // Zero and one operands

        TEST_RR_OP(mulh, 0x00000000, 0x00000000, 0x00000000 );
        TEST_RR_OP(mulh, 0x00000001, 0x00000001, 0x00000000 );
        TEST_RR_OP(mulh, 0x00000003, 0x00000007, 0x00000000 );

        TEST_RR_OP(mulh, 0x00000000, 0xffff8000, 0x00000000 );
        TEST_RR_OP(mulh, 0x80000000, 0x00000000, 0x00000000 );
        TEST_RR_OP(mulh, 0x80000000, 0x00000000, 0x00000000 );

        TEST_RR_OP(mulh, 0xaaaaaaab, 0x0002fe7d, 0xffff0081 );
        TEST_RR_OP(mulh, 0x0002fe7d, 0xaaaaaaab, 0xffff0081 );

        TEST_RR_OP(mulh, 0xff000000, 0xff000000, 0x00010000 );

        TEST_RR_OP(mulh, 0xffffffff, 0xffffffff, 0x00000000 );
        TEST_RR_OP(mulh, 0xffffffff, 0x00000001, 0xffffffff );
        TEST_RR_OP(mulh, 0x00000001, 0xffffffff, 0xffffffff );            

        // Positive operands — result fits in 32-bit, high part is 0
        TEST_RR_OP( mulh,    4200000,   1300000,  0x4f7)
        TEST_RR_OP( mulh,   71600000,   8900000, 0x24390 )
        TEST_RR_OP( mulh, 201540000, 83300000, 0x3ba4da )

        // Negative operands — result high part depends on sign extension
        TEST_RR_OP( mulh,    10,   -5, -1 )
        TEST_RR_OP( mulh,  -2,     2, -1 )
        TEST_RR_OP( mulh, -100, -100, 0 )

        // Mixed high-value tests

        TEST_RR_OP( mulh, 0x0deadbee, 0x10000000, 0xdeadbe )
        TEST_RR_OP( mulh, 0xdeadbeef, 0x10000000, 0xfdeadbee )

        //--------------------------------------------------------------------
        // Source/Destination tests
        //--------------------------------------------------------------------

        TEST_RR_SRC0_EQ_DEST( mulh, 4200000, 1300000, 0x4f7 )
        TEST_RR_SRC1_EQ_DEST( mulh, 71600000, 8900000, 0x24390 )
        TEST_RR_SRC0_EQ_SRC1( mulh, 0x10000, 0x1 )
        TEST_RR_SRCS_EQ_DEST( mulh, 0x100000, 0x100 )

        TEST_RISCV_END
