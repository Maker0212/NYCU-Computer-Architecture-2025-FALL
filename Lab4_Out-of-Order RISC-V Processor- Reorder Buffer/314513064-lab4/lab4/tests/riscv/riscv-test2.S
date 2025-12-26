//=========================================================================
// riscv-test2.S -- ROB bypass needed for consumers before commit
//=========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //-----------------------------------------------------------------
        // The divide pins the ROB head so later instructions cannot commit.
        // x6 must read x5's value directly from the ROB entry.
        //-----------------------------------------------------------------

        li      x10, 63
        li      x11, 7
        div     x12, x10, x11

        addi    x5, x0, 7
        add     x6, x5, x5

        TEST_CHECK_EQ( x6, 14 )

        TEST_RISCV_END
