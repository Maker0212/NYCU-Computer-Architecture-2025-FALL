//========================================================================
// riscv-mulhsu.S
//========================================================================

#include "riscv-macros.h"

        TEST_RISCV_BEGIN

        //--------------------------------------------------------------------
        // Arithmetic tests
        //--------------------------------------------------------------------

        TEST_RR_OP(mulhsu, 0x00000000, 0x00000000, 0x00000000 );
        TEST_RR_OP(mulhsu, 0x00000001, 0x00000001, 0x00000000 );
        TEST_RR_OP(mulhsu, 0x00000003, 0x00000007, 0x00000000 );

        TEST_RR_OP(mulhsu, 0x00000000, 0xffff8000, 0x00000000 );
        TEST_RR_OP(mulhsu, 0x80000000, 0x00000000, 0x00000000 );
        TEST_RR_OP(mulhsu, 0x80000000, 0xffff8000, 0x80004000);

        TEST_RR_OP(mulhsu, 0xaaaaaaab, 0x0002fe7d, 0xffff0081);
        TEST_RR_OP(mulhsu, 0x0002fe7d, 0xaaaaaaab, 0x0001fefe );

        TEST_RR_OP(mulhsu, 0xff000000, 0xff000000, 0xff010000 );

        TEST_RR_OP(mulhsu, 0xffffffff, 0xffffffff, 0xffffffff );
        TEST_RR_OP(mulhsu, 0xffffffff, 0x00000001, 0xffffffff );
        TEST_RR_OP(mulhsu, 0x00000001, 0xffffffff, 0x00000000 );


        //--------------------------------------------------------------------
        // Source/Destination tests
        //--------------------------------------------------------------------

        TEST_RR_SRC0_EQ_DEST( mulhsu, 13<<20, 11<<20, 36608 )
        TEST_RR_SRC1_EQ_DEST( mulhsu, 14<<20, 11<<20, 39424 )
        TEST_RR_SRCS_EQ_DEST( mulhsu, 0x100000, 0x100 )

        TEST_RR_DEST_BYP( 0, mulhsu, 13<<20, 11<<20, 36608 );
        TEST_RR_DEST_BYP( 1, mulhsu, 14<<20, 11<<20, 39424 );
        TEST_RR_DEST_BYP( 2, mulhsu, 15<<20, 11<<20, 42240 );


        //-------------------------------------------------------------
        // Bypassing tests
        //-------------------------------------------------------------        

        TEST_RR_SRC01_BYP( 0, 0, mulhsu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC01_BYP( 0, 1, mulhsu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC01_BYP( 0, 2, mulhsu, 15<<20, 11<<20, 42240 );
        TEST_RR_SRC01_BYP( 1, 0, mulhsu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC01_BYP( 1, 1, mulhsu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC01_BYP( 2, 0, mulhsu, 15<<20, 11<<20, 42240 );

        TEST_RR_SRC10_BYP( 0, 0, mulhsu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC10_BYP( 0, 1, mulhsu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC10_BYP( 0, 2, mulhsu, 15<<20, 11<<20, 42240 );
        TEST_RR_SRC10_BYP( 1, 0, mulhsu, 13<<20, 11<<20, 36608 );
        TEST_RR_SRC10_BYP( 1, 1, mulhsu, 14<<20, 11<<20, 39424 );
        TEST_RR_SRC10_BYP( 2, 0, mulhsu, 15<<20, 11<<20, 42240 );        

        TEST_RISCV_END
