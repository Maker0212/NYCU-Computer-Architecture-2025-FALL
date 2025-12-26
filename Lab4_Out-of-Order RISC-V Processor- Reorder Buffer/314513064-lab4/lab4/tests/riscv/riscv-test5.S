//=========================================================================
// riscv-test5.S -- Fill every ROB entry before the head can drain
//=========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //-----------------------------------------------------------------
        // A long-latency divide (after two quick immediates) holds the ROB
        // head, while the next fifteen independent writes (x4-x18) allocate
        // the remaining slots in the 16-entry buffer.
        //-----------------------------------------------------------------

        li      x1, 120
        li      x2, 3
        div     x3, x1, x2

        addi    x4,  x0, 1
        addi    x5,  x0, 2
        addi    x6,  x0, 3
        addi    x7,  x0, 4
        addi    x8,  x0, 5
        addi    x9,  x0, 6
        addi    x10, x0, 7
        addi    x11, x0, 8
        addi    x12, x0, 9
        addi    x13, x0, 10
        addi    x14, x0, 11
        addi    x15, x0, 12
        addi    x16, x0, 13
        addi    x17, x0, 14
        addi    x18, x0, 15

        TEST_CHECK_EQ( x3, 40 )
        TEST_CHECK_EQ( x18, 15 )

        TEST_RISCV_END
