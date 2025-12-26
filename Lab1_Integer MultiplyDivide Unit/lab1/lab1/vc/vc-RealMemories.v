//========================================================================
// Verilog Components: Synthesizable Real Memories
//========================================================================

`ifndef VC_MEMORIES_V
`define VC_MEMORIES_V

//------------------------------------------------------------------------
// Real 1-Cycle Latency Dual Port Mem
//------------------------------------------------------------------------

module vc_Mem_real2port
#(
  parameter MEM_SZ       = 8,  // Size of physical memory address in bits
  parameter ADDR_SZ      = 8,  // Size of request address in bits
  parameter DATA_SZ      = 32, // Size of data in bits
  parameter ADDR_SHIFT   = 2   // Right shift amount addr before indexing
)(
  input clk, reset,

  // Memory Request Input Interface (Port 0)
  input                    memreq0_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq0_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq0_bits_data,  // Req data
  input                    memreq0_val,        // Req address is valid
  output                   memreq0_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 0)
  output     [DATA_SZ-1:0] memresp0_bits_data, // Response data
  output                   memresp0_val,       // Response is valid

  // Memory Request Input Interface (Port 1)
  input                    memreq1_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq1_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq1_bits_data,  // Req data
  input                    memreq1_val,        // Req address is valid
  output                   memreq1_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 1)
  output     [DATA_SZ-1:0] memresp1_bits_data, // Response data
  output                   memresp1_val        // Response is valid
);

  //----------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------
  reg [DATA_SZ-1:0] m[(1 << MEM_SZ)-1:0];

  reg               memreq0_rdy;
  reg               memreq1_rdy;

  //----------------------------------------------------------------------
  // M0: Address Calculation
  //----------------------------------------------------------------------
  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M0
    = memreq0_bits_addr[ADDR_SZ-1:ADDR_SHIFT];

  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M0
    = memreq1_bits_addr[ADDR_SZ-1:ADDR_SHIFT];

  // Pipeline to next stage
  reg                          memreq0_bits_rw_M1;
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M1;

  reg                          memreq1_bits_rw_M1;
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M1;

  // val/rdy go signal
  wire                         memreq0_go_M0 = ( memreq0_val & memreq0_rdy );
  wire                         memreq1_go_M0 = ( memreq1_val & memreq1_rdy );

  //----------------------------------------------------------------------
  // M1 <- M0: Writes and Pipeline
  //----------------------------------------------------------------------

  always @ ( posedge clk ) begin
    if ( reset ) begin
      memreq0_bits_rw_M1   <= 0;
      phys_addr0_M1        <= 0;

      memreq1_bits_rw_M1   <= 0;
      phys_addr1_M1        <= 0;
    end

    // Writes
    if ( memreq0_bits_rw ) begin
      m[phys_addr0_M0] = memreq0_bits_data;
    end
    if ( memreq1_bits_rw ) begin
      m[phys_addr1_M0] = memreq1_bits_data;
    end

    // Pipeline M1 <- M0
    if ( memreq0_go_M0 ) begin
      memreq0_bits_rw_M1   <= memreq0_bits_rw;
      phys_addr0_M1        <= phys_addr0_M0;
    end

    if ( memreq1_go_M0 ) begin
      memreq1_bits_rw_M1   <= memreq1_bits_rw;
      phys_addr1_M1        <= phys_addr1_M0;
    end
  end

  //----------------------------------------------------------------------
  // M1: Read Operations
  //----------------------------------------------------------------------

  assign memresp0_val = !memreq0_bits_rw_M1;
  assign memresp1_val = !memreq1_bits_rw_M1;

  // Read
  assign memresp0_bits_data = m[phys_addr0_M1];
  assign memresp1_bits_data = m[phys_addr1_M1];

  // Set req rdy signals
  always @ ( posedge clk ) begin
    if ( reset ) begin
      memreq0_rdy <= 1'b1;
      memreq1_rdy <= 1'b1;
    end
  end

endmodule

//------------------------------------------------------------------------
// Real Random Latency Dual Port Mem
//------------------------------------------------------------------------

module vc_Mem_rand2port
#(
  parameter MEM_SZ       = 8,  // Size of physical memory address in bits
  parameter ADDR_SZ      = 8,  // Size of request address in bits
  parameter DATA_SZ      = 32, // Size of data in bits
  parameter ADDR_SHIFT   = 2,  // Right shift amount addr before indexing
  parameter RANDOM_DELAY = 0   // Max num cycles to randomly delay response
)(
  input clk, reset,

  // Memory Request Input Interface (Port 0)
  input                    memreq0_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq0_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq0_bits_data,  // Req data
  input                    memreq0_val,        // Req address is valid
  output                   memreq0_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 0)
  output     [DATA_SZ-1:0] memresp0_bits_data, // Response data
  output                   memresp0_val,       // Response is valid

  // Memory Request Input Interface (Port 1)
  input                    memreq1_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq1_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq1_bits_data,  // Req data
  input                    memreq1_val,        // Req address is valid
  output                   memreq1_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 1)
  output     [DATA_SZ-1:0] memresp1_bits_data, // Response data
  output                   memresp1_val        // Response is valid
);

  //----------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------
  reg  [DATA_SZ-1:0] m[(1 << MEM_SZ)-1:0];

  // Random delay signals

  reg   [31:0]       rand_delay;

  wire  [31:0]       rand_delay_next = ( rand_delay == 0 ) ? ( {$random} % RANDOM_DELAY )
                                     :                       ( rand_delay - 1 );
  wire               rand_delay_en   = ( RANDOM_DELAY > 0 );

  //----------------------------------------------------------------------
  // M0: Address Calculation
  //----------------------------------------------------------------------
  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M0
    = memreq0_bits_addr[ADDR_SZ-1:ADDR_SHIFT];

  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M0
    = memreq1_bits_addr[ADDR_SZ-1:ADDR_SHIFT];

  // Req val/rdy go signal
  wire                         memreq0_go_M0   = ( memreq0_val & memreq0_rdy );
  wire                         memreq1_go_M0   = ( memreq1_val & memreq1_rdy );

  // Resp val signal
  wire                         memresp0_val_M0 = !memreq0_bits_rw;
  wire                         memresp1_val_M0 = !memreq1_bits_rw;

  //----------------------------------------------------------------------
  // M1 <- M0: Writes and Pipeline
  //----------------------------------------------------------------------
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M1;
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M1;

  reg                          memresp0_val_M1;
  reg                          memresp1_val_M1;

  reg                          bubble_M1;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      memresp0_val_M1      <= 0;
      phys_addr0_M1        <= 0;

      memresp1_val_M1      <= 0;
      phys_addr1_M1        <= 0;

      rand_delay           <= 0;

      bubble_M1            <= 0;
    end
    else begin
      rand_delay <= rand_delay_next;
    end

    if ( !rand_delay_en || rand_delay == 0 ) begin
      // Writes
      if ( memreq0_bits_rw ) begin
        m[phys_addr0_M0]   <= memreq0_bits_data;
      end
      if ( memreq1_bits_rw ) begin
        m[phys_addr1_M0]   <= memreq1_bits_data;
      end

      // Pipeline M1 <- M0
      if ( memreq0_go_M0 ) begin
        memresp0_val_M1    <= memresp0_val_M0;
        phys_addr0_M1      <= phys_addr0_M0;
      end
      if ( memreq1_go_M0 ) begin
        memresp1_val_M1    <= memresp1_val_M1;
        phys_addr1_M1      <= phys_addr1_M0;
      end

      bubble_M1            <= 1'b0;
    end
    else begin
      bubble_M1            <= 1'b1;
    end

  end

  //----------------------------------------------------------------------
  // M1: Read Operations
  //----------------------------------------------------------------------

  assign memresp0_val = ( !bubble_M1 && memresp0_val_M1 );
  assign memresp1_val = ( !bubble_M1 && memresp1_val_M1 );

  // Read
  assign memresp0_bits_data = m[phys_addr0_M1];
  assign memresp1_bits_data = m[phys_addr1_M1];

  // Set req rdy signals
  assign memreq0_rdy        = ( reset || !rand_delay_en || rand_delay == 0 );
  assign memreq1_rdy        = ( reset || !rand_delay_en || rand_delay == 0 );

endmodule


//------------------------------------------------------------------------
// Real Random Latency Dual Port Mem with Subword Support
//------------------------------------------------------------------------

module vc_Mem_randsub2port
#(
  parameter MEM_SZ       = 8,  // Size of physical memory address in bits
  parameter ADDR_SZ      = 8,  // Size of request address in bits
  parameter DATA_SZ      = 32, // Size of data in bits
  parameter ADDR_SHIFT   = 2,  // Right shift amount addr before indexing
  parameter RANDOM_DELAY = 0   // Max num cycles to randomly delay response
)(
  input clk, reset,

  // Memory Request Input Interface (Port 0)
  input                    memreq0_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq0_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq0_bits_data,  // Req data
  input                    memreq0_val,        // Req address is valid
  output                   memreq0_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 0)
  output     [DATA_SZ-1:0] memresp0_bits_data, // Response data
  output                   memresp0_val,       // Response is valid

  // Memory Request Input Interface (Port 1)
  input                    memreq1_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq1_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq1_bits_data,  // Req data
  input                    subword_en,         // Enable subword ops
  input                    subword_sign,       // 1 = signed, 0 = unsigned;
  input                    subword_type,       // 1 = halfword, 0 = byte;
  input                    memreq1_val,        // Req address is valid
  output                   memreq1_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 1)
  output     [DATA_SZ-1:0] memresp1_bits_data, // Response data
  output                   memresp1_val,       // Response is valid
  output                   memresp1_sack       // Store ack
);

  //----------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------
  reg  [DATA_SZ-1:0] m[(1 << MEM_SZ)-1:0];

  // Random delay signals

  reg   [31:0]       rand_delay;

  wire  [31:0]       rand_delay_next = ( rand_delay == 0 ) ? ( {$random} % RANDOM_DELAY )
                                     :                       ( rand_delay - 1 );
  wire               rand_delay_en   = ( RANDOM_DELAY > 0 );

  //----------------------------------------------------------------------
  // M0: Address Calculation
  //----------------------------------------------------------------------
  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M0
    = memreq0_bits_addr[ADDR_SZ-1:ADDR_SHIFT];

  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M0
    = memreq1_bits_addr[ADDR_SZ-1:ADDR_SHIFT];
  wire [ADDR_SHIFT-1:0]         sub_addr1_M0 = memreq1_bits_addr[ADDR_SHIFT-1:0];

  // Req val/rdy go signal
  wire                         memreq0_go_M0   = ( memreq0_val & memreq0_rdy );
  wire                         memreq1_go_M0   = ( memreq1_val & memreq1_rdy );

  // Resp val signal
  wire                         memresp0_val_M0 = !memreq0_bits_rw;
  wire                         memresp1_val_M0 = !memreq1_bits_rw;

  //----------------------------------------------------------------------
  // M1 <- M0: Writes and Pipeline
  //----------------------------------------------------------------------
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M1;
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M1;

  reg [ADDR_SHIFT-1:0]         sub_addr1_M1;
  reg                          subword_en_M1;
  reg                          subword_sign_M1;
  reg                          subword_type_M1;

  reg                          memresp0_val_M1;
  reg                          memresp1_val_M1;

  reg                          memresp1_sack_M1;

  reg                          bubble_M1;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      memresp0_val_M1      <= 0;
      phys_addr0_M1        <= 0;

      memresp1_val_M1      <= 0;
      phys_addr1_M1        <= 0;

      memresp1_sack_M1     <= 0;

      sub_addr1_M1         <= 0;
      subword_en_M1        <= 0;
      subword_sign_M1      <= 0;
      subword_type_M1      <= 0;

      rand_delay           <= 0;

      bubble_M1            <= 0;
    end
    else begin
      rand_delay <= rand_delay_next;
    end

    if ( !rand_delay_en || rand_delay == 0 ) begin
      // Writes
      if ( memreq0_bits_rw && memreq0_go_M0 ) begin
        m[phys_addr0_M0]   <= memreq0_bits_data;
      end
      if ( memreq1_bits_rw  && memreq1_go_M0 ) begin
        if ( !subword_en ) begin
          m[phys_addr1_M0]   <= memreq1_bits_data;
        end

        // halfword stores
        else if ( subword_type ) begin
          if ( !sub_addr1_M0[1] ) begin
            m[phys_addr1_M0][(DATA_SZ>>1)-1:0] <= memreq1_bits_data[15:0];
          end
          else if ( sub_addr1_M0[1] ) begin
            m[phys_addr1_M0][DATA_SZ-1:DATA_SZ>>1] <= memreq1_bits_data[15:0];
          end
        end

        // byte stores
        else if ( !subword_type ) begin
          if ( sub_addr1_M0 == 2'd0 ) begin
            m[phys_addr1_M0][(DATA_SZ>>2)-1:0] <= memreq1_bits_data[7:0];
          end
          else if ( sub_addr1_M0 == 2'd1 ) begin
            m[phys_addr1_M0][(DATA_SZ>>1)-1:DATA_SZ>>2] <= memreq1_bits_data[7:0];
          end
          else if ( sub_addr1_M0 == 2'd2 ) begin
            m[phys_addr1_M0][DATA_SZ-(DATA_SZ>>2)-1:DATA_SZ>>1] <= memreq1_bits_data[7:0];
          end
          else if ( sub_addr1_M0 == 2'd3 ) begin
            m[phys_addr1_M0][DATA_SZ-1:DATA_SZ-(DATA_SZ>>2)] <= memreq1_bits_data[7:0];
          end
        end

        memresp1_sack_M1 <= 1'b1;
      end
      else begin
        memresp1_sack_M1 <= 1'b0;
      end

      // Pipeline M1 <- M0
      if ( memreq0_go_M0 ) begin
        memresp0_val_M1    <= memresp0_val_M0;
        phys_addr0_M1      <= phys_addr0_M0;
      end
      if ( memreq1_go_M0 ) begin
        memresp1_val_M1    <= memresp1_val_M0;
        phys_addr1_M1      <= phys_addr1_M0;
        sub_addr1_M1       <= sub_addr1_M0;
        subword_en_M1      <= subword_en;
        subword_sign_M1    <= subword_sign;
        subword_type_M1    <= subword_type;
      end

      bubble_M1            <= 1'b0;
    end
    else begin
      bubble_M1            <= 1'b1;
      memresp1_sack_M1     <= 1'b0;
    end

  end

  //----------------------------------------------------------------------
  // M1: Read Operations
  //----------------------------------------------------------------------

  assign memresp0_val = ( !bubble_M1 && memresp0_val_M1 );
  assign memresp1_val = ( !bubble_M1 && memresp1_val_M1 );

  assign memresp1_sack = memresp1_sack_M1;

  // halfword load raw
  wire [15:0] half_memresp1_bits_data_M1;
  assign half_memresp1_bits_data_M1 =
         ( !sub_addr1_M1[1] ) ? m[phys_addr1_M1][(DATA_SZ>>1)-1:0]
       : ( sub_addr1_M1[1] )  ? m[phys_addr1_M1][DATA_SZ-1:DATA_SZ>>1]
       :                        16'bx;

  // halfword load with signs
  wire [31:0] sign_half_memresp1_bits_data_M1;
  assign sign_half_memresp1_bits_data_M1 =
         ( subword_sign_M1 )  ? { {16{half_memresp1_bits_data_M1[15]}}, half_memresp1_bits_data_M1 }
       : ( !subword_sign_M1 ) ? { 16'b0, half_memresp1_bits_data_M1 }
       :                        32'bx;

  // byte load raw
  wire [7:0]  byte_memresp1_bits_data_M1;
  assign byte_memresp1_bits_data_M1 =
         ( sub_addr1_M1 == 2'd0 ) ? m[phys_addr1_M1][(DATA_SZ>>2)-1:0]
       : ( sub_addr1_M1 == 2'd1 ) ? m[phys_addr1_M1][(DATA_SZ>>1)-1:DATA_SZ>>2]
       : ( sub_addr1_M1 == 2'd2 ) ? m[phys_addr1_M1][DATA_SZ-(DATA_SZ>>2)-1:DATA_SZ>>1]
       : ( sub_addr1_M1 == 2'd3 ) ? m[phys_addr1_M1][DATA_SZ-1:DATA_SZ-(DATA_SZ>>2)]
       :                            8'bx;

  // byte load with signs
  wire [31:0] sign_byte_memresp1_bits_data_M1;
  assign sign_byte_memresp1_bits_data_M1 =
         ( subword_sign_M1 )  ? { {24{byte_memresp1_bits_data_M1[7]}}, byte_memresp1_bits_data_M1 }
       : ( !subword_sign_M1 ) ? { 24'b0, byte_memresp1_bits_data_M1 }
       :                        32'bx;

  // Read
  assign memresp0_bits_data = m[phys_addr0_M1];
  assign memresp1_bits_data = ( !subword_en_M1 )   ? m[phys_addr1_M1]
                            : ( subword_type_M1 )  ? sign_half_memresp1_bits_data_M1
                            : ( !subword_type_M1 ) ? sign_byte_memresp1_bits_data_M1
                            :                        32'bx;

  // Set req rdy signals
  assign memreq0_rdy        = ( reset || !rand_delay_en || rand_delay == 0 );
  assign memreq1_rdy        = ( reset || !rand_delay_en || rand_delay == 0 );

endmodule

//------------------------------------------------------------------------
// 2 Read 3 Write Ports
//------------------------------------------------------------------------

module vc_Mem_amo2port
#(
  parameter MEM_SZ       = 8,  // Size of physical memory address in bits
  parameter ADDR_SZ      = 8,  // Size of request address in bits
  parameter DATA_SZ      = 32, // Size of data in bits
  parameter ADDR_SHIFT   = 2,  // Right shift amount addr before indexing
  parameter RANDOM_DELAY = 0   // Max num cycles to randomly delay response
)(
  input clk, reset,

  // Memory Request Input Interface (Port 0)
  input                    memreq0_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq0_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq0_bits_data,  // Req data
  input                    memreq0_val,        // Req address is valid
  output                   memreq0_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 0)
  output     [DATA_SZ-1:0] memresp0_bits_data, // Response data
  output                   memresp0_val,       // Response is valid

  // Memory Request Input Interface (Port 1)
  input                    memreq1_bits_rw,    // Req type (0=read,1=write)
  input      [ADDR_SZ-1:0] memreq1_bits_addr,  // Req address
  input      [DATA_SZ-1:0] memreq1_bits_data,  // Req data
  input                    subword_en,         // Enable subword ops
  input                    subword_sign,       // 1 = signed, 0 = unsigned;
  input                    subword_type,       // 1 = halfword, 0 = byte;
  input                    memreq1_val,        // Req address is valid
  output                   memreq1_rdy,        // Mem ready to accept req

  // Memory Response Output Interface (Port 1)
  output     [DATA_SZ-1:0] memresp1_bits_data, // Response data
  output                   memresp1_val,       // Response is valid
  output                   memresp1_sack,      // Store ack

  // Memory Write Request Input Interface (Port 2)
  input      [ADDR_SZ-1:0] memreq2_bits_addr,
  input      [DATA_SZ-1:0] memreq2_bits_data,
  input                    memreq2_val,
  output                   memreq2_rdy
);

  //----------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------
  reg  [DATA_SZ-1:0] m[(1 << MEM_SZ)-1:0];

  // Random delay signals

  reg   [31:0]       rand_delay;

  wire  [31:0]       rand_delay_next = ( rand_delay == 0 ) ? ( {$random} % RANDOM_DELAY )
                                     :                       ( rand_delay - 1 );
  wire               rand_delay_en   = ( RANDOM_DELAY > 0 );

  //----------------------------------------------------------------------
  // M0: Address Calculation
  //----------------------------------------------------------------------
  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M0
    = memreq0_bits_addr[ADDR_SZ-1:ADDR_SHIFT];

  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M0
    = memreq1_bits_addr[ADDR_SZ-1:ADDR_SHIFT];
  wire [ADDR_SHIFT-1:0]         sub_addr1_M0 = memreq1_bits_addr[ADDR_SHIFT-1:0];

  wire [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr2_M0
    = memreq2_bits_addr[ADDR_SZ-1:ADDR_SHIFT];

  // Req val/rdy go signal
  wire                         memreq0_go_M0   = ( memreq0_val & memreq0_rdy );
  wire                         memreq1_go_M0   = ( memreq1_val & memreq1_rdy );
  wire                         memreq2_go_M0   = ( memreq2_val & memreq2_rdy );

  // Resp val signal
  wire                         memresp0_val_M0 = !memreq0_bits_rw;
  wire                         memresp1_val_M0 = !memreq1_bits_rw;

  //----------------------------------------------------------------------
  // M1 <- M0: Writes and Pipeline
  //----------------------------------------------------------------------
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr0_M1;
  reg [ADDR_SZ-ADDR_SHIFT-1:0] phys_addr1_M1;

  reg [ADDR_SHIFT-1:0]         sub_addr1_M1;
  reg                          subword_en_M1;
  reg                          subword_sign_M1;
  reg                          subword_type_M1;

  reg                          memresp0_val_M1;
  reg                          memresp1_val_M1;

  reg                          memresp1_sack_M1;

  reg                          bubble_M1;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      memresp0_val_M1      <= 0;
      phys_addr0_M1        <= 0;

      memresp1_val_M1      <= 0;
      phys_addr1_M1        <= 0;

      memresp1_sack_M1     <= 0;

      sub_addr1_M1         <= 0;
      subword_en_M1        <= 0;
      subword_sign_M1      <= 0;
      subword_type_M1      <= 0;

      rand_delay           <= 0;

      bubble_M1            <= 0;
    end
    else begin
      rand_delay <= rand_delay_next;
    end

    if ( !rand_delay_en || rand_delay == 0 ) begin
      // Writes
      if ( memreq0_bits_rw && memreq0_go_M0 ) begin
        m[phys_addr0_M0]   <= memreq0_bits_data;
      end
      if ( memreq1_bits_rw  && memreq1_go_M0 ) begin
        if ( !subword_en ) begin
          m[phys_addr1_M0]   <= memreq1_bits_data;
        end

        // halfword stores
        else if ( subword_type ) begin
          if ( !sub_addr1_M0[1] ) begin
            m[phys_addr1_M0][(DATA_SZ>>1)-1:0] <= memreq1_bits_data[15:0];
          end
          else if ( sub_addr1_M0[1] ) begin
            m[phys_addr1_M0][DATA_SZ-1:DATA_SZ>>1] <= memreq1_bits_data[15:0];
          end
        end

        // byte stores
        else if ( !subword_type ) begin
          if ( sub_addr1_M0 == 2'd0 ) begin
            m[phys_addr1_M0][(DATA_SZ>>2)-1:0] <= memreq1_bits_data[7:0];
          end
          else if ( sub_addr1_M0 == 2'd1 ) begin
            m[phys_addr1_M0][(DATA_SZ>>1)-1:DATA_SZ>>2] <= memreq1_bits_data[7:0];
          end
          else if ( sub_addr1_M0 == 2'd2 ) begin
            m[phys_addr1_M0][DATA_SZ-(DATA_SZ>>2)-1:DATA_SZ>>1] <= memreq1_bits_data[7:0];
          end
          else if ( sub_addr1_M0 == 2'd3 ) begin
            m[phys_addr1_M0][DATA_SZ-1:DATA_SZ-(DATA_SZ>>2)] <= memreq1_bits_data[7:0];
          end
        end

        memresp1_sack_M1 <= 1'b1;
      end
      else begin
        memresp1_sack_M1 <= 1'b0;
      end

      // Port 2 write port, always assumed to be write if valid
      if ( memreq2_go_M0 ) begin
        m[phys_addr2_M0] <= memreq2_bits_data;
      end

      // Pipeline M1 <- M0
      if ( memreq0_go_M0 ) begin
        memresp0_val_M1    <= memresp0_val_M0;
        phys_addr0_M1      <= phys_addr0_M0;
      end
      if ( memreq1_go_M0 ) begin
        memresp1_val_M1    <= memresp1_val_M0;
        phys_addr1_M1      <= phys_addr1_M0;
        sub_addr1_M1       <= sub_addr1_M0;
        subword_en_M1      <= subword_en;
        subword_sign_M1    <= subword_sign;
        subword_type_M1    <= subword_type;
      end

      bubble_M1            <= 1'b0;
    end
    else begin
      bubble_M1            <= 1'b1;
      memresp1_sack_M1     <= 1'b0;
    end

  end

  //----------------------------------------------------------------------
  // M1: Read Operations
  //----------------------------------------------------------------------

  assign memresp0_val = ( !bubble_M1 && memresp0_val_M1 );
  assign memresp1_val = ( !bubble_M1 && memresp1_val_M1 );

  assign memresp1_sack = memresp1_sack_M1;

  // halfword load raw
  wire [15:0] half_memresp1_bits_data_M1;
  assign half_memresp1_bits_data_M1 =
         ( !sub_addr1_M1[1] ) ? m[phys_addr1_M1][(DATA_SZ>>1)-1:0]
       : ( sub_addr1_M1[1] )  ? m[phys_addr1_M1][DATA_SZ-1:DATA_SZ>>1]
       :                        16'bx;

  // halfword load with signs
  wire [31:0] sign_half_memresp1_bits_data_M1;
  assign sign_half_memresp1_bits_data_M1 =
         ( subword_sign_M1 )  ? { {16{half_memresp1_bits_data_M1[15]}}, half_memresp1_bits_data_M1 }
       : ( !subword_sign_M1 ) ? { 16'b0, half_memresp1_bits_data_M1 }
       :                        32'bx;

  // byte load raw
  wire [7:0]  byte_memresp1_bits_data_M1;
  assign byte_memresp1_bits_data_M1 =
         ( sub_addr1_M1 == 2'd0 ) ? m[phys_addr1_M1][(DATA_SZ>>2)-1:0]
       : ( sub_addr1_M1 == 2'd1 ) ? m[phys_addr1_M1][(DATA_SZ>>1)-1:DATA_SZ>>2]
       : ( sub_addr1_M1 == 2'd2 ) ? m[phys_addr1_M1][DATA_SZ-(DATA_SZ>>2)-1:DATA_SZ>>1]
       : ( sub_addr1_M1 == 2'd3 ) ? m[phys_addr1_M1][DATA_SZ-1:DATA_SZ-(DATA_SZ>>2)]
       :                            8'bx;

  // byte load with signs
  wire [31:0] sign_byte_memresp1_bits_data_M1;
  assign sign_byte_memresp1_bits_data_M1 =
         ( subword_sign_M1 )  ? { {24{byte_memresp1_bits_data_M1[7]}}, byte_memresp1_bits_data_M1 }
       : ( !subword_sign_M1 ) ? { 24'b0, byte_memresp1_bits_data_M1 }
       :                        32'bx;

  // Read
  assign memresp0_bits_data = m[phys_addr0_M1];
  assign memresp1_bits_data = ( !subword_en_M1 )   ? m[phys_addr1_M1]
                            : ( subword_type_M1 )  ? sign_half_memresp1_bits_data_M1
                            : ( !subword_type_M1 ) ? sign_byte_memresp1_bits_data_M1
                            :                        32'bx;

  // Set req rdy signals
  assign memreq0_rdy        = ( reset || !rand_delay_en || rand_delay == 0 );
  assign memreq1_rdy        = ( reset || !rand_delay_en || rand_delay == 0 );
  assign memreq2_rdy        = ( reset || !rand_delay_en || rand_delay == 0 );

endmodule

`endif

