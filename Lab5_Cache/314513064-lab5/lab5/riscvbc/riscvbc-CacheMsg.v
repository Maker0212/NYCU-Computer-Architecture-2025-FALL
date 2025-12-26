`ifndef RISCV_CACHE_MSG_V
`define RISCV_CACHE_MSG_V

        //------------------------------------------------------------------------
        // Read or Write
        //------------------------------------------------------------------------
`define READ  1'b0
`define WRITE 1'b1

        //------------------------------------------------------------------------
        // Bit field definitions for cache addressing
        //------------------------------------------------------------------------
`define OFF_BITS 6
`define IDX_BITS 5
`define TAG_BITS 21

        //------------------------------------------------------------------------
        // Cache parameter
        //------------------------------------------------------------------------
`define BLK_SIZE   512
`define D_SET_SIZE 32

`define WAY0 1'b0
`define WAY1 1'b1

`endif
