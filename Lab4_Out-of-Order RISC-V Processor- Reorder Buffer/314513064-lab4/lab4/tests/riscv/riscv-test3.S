//=========================================================================
// riscv-test3.S -- Safe WAW thanks to long delay between the writes
//=========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //-----------------------------------------------------------------
        // The busy loop burns enough cycles for the first write to commit
        // before the second reaches decode, so both cores run correctly.
        //-----------------------------------------------------------------

        li      x5, 0
        li      x6, 5

        addi    x5, x0, 11

1:      addi    x6, x6, -1
        bne     x6, x0, 1b

        addi    x5, x0, 22

        TEST_CHECK_EQ( x5, 22 )

        TEST_RISCV_END
