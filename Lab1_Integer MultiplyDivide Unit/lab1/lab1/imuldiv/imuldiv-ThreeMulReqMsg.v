//====================================================================================
// imuldiv-ThreeMulReqMsg : Three Input Multiplier/Divider Request Message
//====================================================================================
// A three input multiplier/divider request message contains three input datas and the
// desired operation and is sent to a three input multipler/divider unit. The unit
// will respond with a multiplier/divider response message.
//
// Message Format:
//
//   98  96 95       64 63       32 31        0
//  +------+-----------+-----------+-----------+
//  | func | operand c | operand b | operand a |
//  +------+-----------+-----------+-----------+
//

`ifndef IMULDIV_THREE_MULDIVREQ_MSG_V
`define IMULDIV_THREE_MULDIVREQ_MSG_V

//------------------------------------------------------------------------
// Message defines
//------------------------------------------------------------------------

// Size of message

`define IMULDIV_THREE_MULDIVREQ_MSG_SZ         99

// Size and enums for each field

`define IMULDIV_THREE_MULDIVREQ_MSG_FUNC_SZ    3
`define IMULDIV_THREE_MULDIVREQ_MSG_FUNC_MUL   3'd0
`define IMULDIV_THREE_MULDIVREQ_MSG_FUNC_DIV   3'd1
`define IMULDIV_THREE_MULDIVREQ_MSG_FUNC_DIVU  3'd2
`define IMULDIV_THREE_MULDIVREQ_MSG_FUNC_REM   3'd3
`define IMULDIV_THREE_MULDIVREQ_MSG_FUNC_REMU  3'd4

`define IMULDIV_THREE_MULDIVREQ_MSG_A_SZ       32
`define IMULDIV_THREE_MULDIVREQ_MSG_B_SZ       32
`define IMULDIV_THREE_MULDIVREQ_MSG_C_SZ       32

// Location of each field

`define IMULDIV_THREE_MULDIVREQ_MSG_FUNC_FIELD 98:96
`define IMULDIV_THREE_MULDIVREQ_MSG_A_FIELD    95:64
`define IMULDIV_THREE_MULDIVREQ_MSG_B_FIELD    63:32
`define IMULDIV_THREE_MULDIVREQ_MSG_C_FIELD    31:0

//------------------------------------------------------------------------
// Convert message to bits
//------------------------------------------------------------------------

module imuldiv_ThreeMulReqMsgToBits
(
  // Input message

  input [`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_SZ-1:0] func,
  input [`IMULDIV_THREE_MULDIVREQ_MSG_A_SZ-1:0]    a,
  input [`IMULDIV_THREE_MULDIVREQ_MSG_B_SZ-1:0]    b,
  input [`IMULDIV_THREE_MULDIVREQ_MSG_C_SZ-1:0]    c,

  // Output bits

  output [`IMULDIV_THREE_MULDIVREQ_MSG_SZ-1:0] bits
);

  assign bits[`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_FIELD] = func;
  assign bits[`IMULDIV_THREE_MULDIVREQ_MSG_A_FIELD]    = a;
  assign bits[`IMULDIV_THREE_MULDIVREQ_MSG_B_FIELD]    = b;
  assign bits[`IMULDIV_THREE_MULDIVREQ_MSG_C_FIELD]    = c;

endmodule

//------------------------------------------------------------------------
// Convert message from bits
//------------------------------------------------------------------------

module imuldiv_ThreeMulReqMsgFromBits
(
  // Input bits

  input [`IMULDIV_THREE_MULDIVREQ_MSG_SZ-1:0] bits,

  // Output message

  output [`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_SZ-1:0] func,
  output [`IMULDIV_THREE_MULDIVREQ_MSG_A_SZ-1:0]    a,
  output [`IMULDIV_THREE_MULDIVREQ_MSG_B_SZ-1:0]    b,
  output [`IMULDIV_THREE_MULDIVREQ_MSG_C_SZ-1:0]    c
);

  assign func = bits[`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_FIELD];
  assign a    = bits[`IMULDIV_THREE_MULDIVREQ_MSG_A_FIELD];
  assign b    = bits[`IMULDIV_THREE_MULDIVREQ_MSG_B_FIELD];
  assign c    = bits[`IMULDIV_THREE_MULDIVREQ_MSG_C_FIELD];

endmodule

//------------------------------------------------------------------------
// Convert message to string
//------------------------------------------------------------------------

`ifndef SYNTHESIS
module imuldiv_ThreeMulReqMsgToStr
(
  input [`IMULDIV_THREE_MULDIVREQ_MSG_SZ-1:0] msg
);

  // Extract fields

  wire [`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_SZ-1:0] func = msg[`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_FIELD];
  wire [`IMULDIV_THREE_MULDIVREQ_MSG_A_SZ-1:0]    a    = msg[`IMULDIV_THREE_MULDIVREQ_MSG_A_FIELD];
  wire [`IMULDIV_THREE_MULDIVREQ_MSG_B_SZ-1:0]    b    = msg[`IMULDIV_THREE_MULDIVREQ_MSG_B_FIELD];
  wire [`IMULDIV_THREE_MULDIVREQ_MSG_C_SZ-1:0]    c    = msg[`IMULDIV_THREE_MULDIVREQ_MSG_C_FIELD];

  // Short names

  localparam mul   = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_MUL;
  localparam div   = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_DIV;
  localparam divu  = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_DIVU;
  localparam rem   = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_REM;
  localparam remu  = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_REMU;

  // Full string sized for 20 characters

  reg [30*8-1:0] full_str;
  always @(*) begin

    if ( msg === `IMULDIV_THREE_MULDIVREQ_MSG_SZ'bx )
      $sformat( full_str, "x            ");
    else begin
      case ( func )
        mul     : $sformat( full_str, "mul  %d, %d, %d", a, b, c );
        div     : $sformat( full_str, "div  %d, %d, %d", a, b, c );
        divu    : $sformat( full_str, "divu %d, %d, %d", a, b, c );
        rem     : $sformat( full_str, "rem  %d, %d, %d", a, b, c );
        remu    : $sformat( full_str, "remu %d, %d, %d", a, b, c );
        default : $sformat( full_str, "undefined func" );
      endcase
    end

  end

  // Tiny string sized for 2 characters

  reg [2*8-1:0] tiny_str;
  always @(*) begin

    if ( msg === `IMULDIV_THREE_MULDIVREQ_MSG_SZ'bx )
      $sformat( tiny_str, "x ");
    else begin
      case ( func )
        mul     : $sformat( tiny_str, "* "  );
        div     : $sformat( tiny_str, "/ "  );
        divu    : $sformat( tiny_str, "/u"  );
        rem     : $sformat( tiny_str, "%% " );
        remu    : $sformat( tiny_str, "%%u" );
        default : $sformat( tiny_str, "??"  );
      endcase
    end

  end

endmodule
`endif /* SYNTHESIS */

`endif /* IMULDIV_THREE_MULDIVREQ_MSG_V */


