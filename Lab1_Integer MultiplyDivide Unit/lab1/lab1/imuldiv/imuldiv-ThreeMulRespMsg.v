//====================================================================================
// imuldiv-ThreeMulRespMsg.v : Three Input Multiplier/Divider Response Message
//====================================================================================
// A three input multiplier/divider response message is a trivial message with just
// the result data.
//
// Message Format:
//
//   95                   0
//  +----------------------+
//  |       result         |
//  +----------------------+
//

`ifndef IMULDIV_THREE_MULDIVRESP_MSG_V
`define IMULDIV_THREE_MULDIVRESP_MSG_V

//------------------------------------------------------------------------
// Message defines
//------------------------------------------------------------------------

// Size of message

`define IMULDIV_THREE_MULDIVRESP_MSG_SZ         96

`endif /* IMULDIV_THREE_MULDIVRESP_MSG_V */

