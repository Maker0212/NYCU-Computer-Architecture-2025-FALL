//========================================================================
// riscv-mulhu.S
//========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //--------------------------------------------------------------------
        // Arithmetic tests
        //--------------------------------------------------------------------

        TEST_RR_OP( mulhu, 0x00000000, 0x00000000, 0x00000000 );
        TEST_RR_OP( mulhu, 0x00000001, 0x00000001, 0x00000000 );
        TEST_RR_OP( mulhu, 0x00000003, 0x00000007, 0x00000000 );

        TEST_RR_OP( mulhu, 0x00000000, 0xffff8000, 0x00000000 );
        TEST_RR_OP( mulhu, 0x80000000, 0x00000000, 0x00000000);
        TEST_RR_OP( mulhu, 0x80000000, 0xffff8000, 0x7fffc000 );

        TEST_RR_OP( mulhu, 0xaaaaaaab, 0x0002fe7d, 0x0001fefe );
        TEST_RR_OP( mulhu, 0x0002fe7d, 0xaaaaaaab, 0x0001fefe ); 
        TEST_RR_OP( mulhu, 0xff000000, 0xff000000, 0xfe010000 ); 
        TEST_RR_OP( mulhu, 0xffffffff, 0xffffffff, 0xfffffffe );
        TEST_RR_OP( mulhu, 0xffffffff, 0x00000001, 0x00000000 );
        TEST_RR_OP( mulhu, 0x00000001, 0xffffffff, 0x00000000 );

        //--------------------------------------------------------------------
        // Source/Destination tests
        //--------------------------------------------------------------------


        TEST_RR_SRC0_EQ_DEST( mulhu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC1_EQ_DEST( mulhu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRCS_EQ_DEST( mulhu, 13<<20, 43264 );


        //-------------------------------------------------------------
        // Bypassing tests
        //-------------------------------------------------------------

        TEST_RR_DEST_BYP( 0, mulhu, 13<<20, 11<<20, 36608 );
        TEST_RR_DEST_BYP( 1, mulhu, 14<<20, 11<<20, 39424 );
        TEST_RR_DEST_BYP( 2, mulhu, 15<<20, 11<<20, 42240 );

        TEST_RR_SRC01_BYP( 0, 0, mulhu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC01_BYP( 0, 1, mulhu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC01_BYP( 0, 2, mulhu, 15<<20, 11<<20, 42240 );
        TEST_RR_SRC01_BYP( 1, 0, mulhu, 13<<20, 11<<20, 36608);
        TEST_RR_SRC01_BYP( 1, 1, mulhu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC01_BYP( 2, 0, mulhu, 15<<20, 11<<20, 42240 );

        TEST_RR_SRC10_BYP( 0, 0, mulhu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC10_BYP( 0, 1, mulhu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC10_BYP( 0, 2, mulhu, 15<<20, 11<<20, 42240 );
        TEST_RR_SRC10_BYP( 1, 0, mulhu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC10_BYP( 1, 1, mulhu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC10_BYP( 2, 0, mulhu, 15<<20, 11<<20, 42240 );


        TEST_RISCV_END
