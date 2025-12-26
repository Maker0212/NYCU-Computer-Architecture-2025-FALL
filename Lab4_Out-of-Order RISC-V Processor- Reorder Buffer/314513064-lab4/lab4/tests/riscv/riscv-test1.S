//=========================================================================
// riscv-test1.S -- Demonstrate why ROBs must commit in order
//=========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //-----------------------------------------------------------------
        // Older long-latency write followed by a short write to same rd.
        // Without a ROB, the divide completes late and overwrites x4.
        //-----------------------------------------------------------------

        li      x2, 40
        li      x3, 5
        div     x4, x2, x3
        addi    x4, x0, 42

        TEST_CHECK_EQ( x4, 42 )

        TEST_RISCV_END
