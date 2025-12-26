//=========================================================================
// riscv-test4.S -- Branch thrash where in-order beats speculative OOO
//=========================================================================
// riscvlong never speculates past a branch, but riscvooo does. This loop
// alternates the branch outcome every iteration, so riscvooo repeatedly
// mispredicts, wastes fetch/issue bandwidth, and can end up with a lower
// IPC even though the work is identical.
//=========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        li      x4, 0          // iterations counted on even branch path
        li      x5, 0          // iterations counted on odd branch path
        li      x6, 8          // total iterations (keep < 30 inst overall)
        li      x7, 0          // toggles branch direction

1:      xori    x7, x7, 1
        bne     x7, x0, 2f
        addi    x4, x4, 1
        j       3f
2:      addi    x5, x5, 1
3:      addi    x6, x6, -1
        bne     x6, x0, 1b

        add     x8, x4, x5
        TEST_CHECK_EQ( x8, 8 )

        TEST_RISCV_END
