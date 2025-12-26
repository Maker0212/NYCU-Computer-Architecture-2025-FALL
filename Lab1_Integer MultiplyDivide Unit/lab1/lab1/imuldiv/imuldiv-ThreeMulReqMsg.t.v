//========================================================================
// imuldiv-ThreeMulReqMsg Unit Tests
//========================================================================

`include "imuldiv-ThreeMulReqMsg.v"
`include "vc-Test.v"

module tester;
  `VC_TEST_SUITE_BEGIN( "imuldiv-ThreeMulReqMsg" )

  //----------------------------------------------------------------------
  // Local parametersT
  //----------------------------------------------------------------------

  localparam mul  = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_MUL;
  localparam div  = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_DIV;
  localparam divu = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_DIVU;
  localparam rem  = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_REM;
  localparam remu = `IMULDIV_THREE_MULDIVREQ_MSG_FUNC_REMU;

  //----------------------------------------------------------------------
  // LcdMsgToString helper module
  //----------------------------------------------------------------------

  reg [`IMULDIV_THREE_MULDIVREQ_MSG_SZ-1:0] msg_test;
  reg [`IMULDIV_THREE_MULDIVREQ_MSG_SZ-1:0] msg_ref;

  imuldiv_ThreeMulReqMsgToStr three_muldivreq_msg_to_str( msg_test );

  //----------------------------------------------------------------------
  // TestBasicFullStr
  //----------------------------------------------------------------------

  // Helper tasks

  task t1_do_test
  (
    input [`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_SZ-1:0] func,
    input [`IMULDIV_THREE_MULDIVREQ_MSG_A_SZ-1:0]    a,
    input [`IMULDIV_THREE_MULDIVREQ_MSG_B_SZ-1:0]    b,
    input [`IMULDIV_THREE_MULDIVREQ_MSG_C_SZ-1:0]    c
  );
  begin

    // Create a wire and set msg fields using `defines

    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_FIELD] = func;
    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_A_FIELD]    = a;
    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_B_FIELD]    = b;
    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_C_FIELD]    = c;

    // Create a wire and set msg fields using concatentation

    msg_ref = { func, a, b, c };

    // Check that both msgs are the same

    #1;
    `VC_TEST_EQ( three_muldivreq_msg_to_str.full_str, msg_test, msg_ref )
    #9;
  end
  endtask

  `VC_TEST_CASE_BEGIN( 1, "TestBasicFullStr" )
  begin

    // Create mul messages

    t1_do_test( mul,  32'd00, 32'd01, 32'd10 );
    t1_do_test( mul,  32'd42, 32'd01, 32'd11 );
    t1_do_test( mul,  32'd18, 32'd68, 32'd12 );

    // Create div messages

    t1_do_test( div,  32'd00, 32'd01, 32'd20 );
    t1_do_test( div,  32'd42, 32'd01, 32'd21 );
    t1_do_test( div,  32'd18, 32'd68, 32'd22 );

    // Create divu messages

    t1_do_test( divu, 32'd00, 32'd01, 32'd30 );
    t1_do_test( divu, 32'd42, 32'd01, 32'd31 );
    t1_do_test( divu, 32'd18, 32'd68, 32'd32 );

    // Create rem messages

    t1_do_test( rem,  32'd00, 32'd01, 32'd40 );
    t1_do_test( rem,  32'd42, 32'd01, 32'd41 );
    t1_do_test( rem,  32'd18, 32'd68, 32'd42 );

    // Create remu messages

    t1_do_test( remu, 32'd00, 32'd01, 32'd50 );
    t1_do_test( remu, 32'd42, 32'd01, 32'd51 );
    t1_do_test( remu, 32'd18, 32'd68, 32'd52 );

  end
  `VC_TEST_CASE_END

  //----------------------------------------------------------------------
  // TestBasicFullStr
  //----------------------------------------------------------------------

  // Helper tasks

  task t2_do_test
  (
    input [`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_SZ-1:0] func,
    input [`IMULDIV_THREE_MULDIVREQ_MSG_A_SZ-1:0]    a,
    input [`IMULDIV_THREE_MULDIVREQ_MSG_B_SZ-1:0]    b,
    input [`IMULDIV_THREE_MULDIVREQ_MSG_C_SZ-1:0]    c
  );
  begin

    // Create a wire and set msg fields using `defines

    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_FUNC_FIELD] = func;
    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_A_FIELD]    = a;
    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_B_FIELD]    = b;
    msg_test[`IMULDIV_THREE_MULDIVREQ_MSG_C_FIELD]    = c;

    // Create a wire and set msg fields using concatentation

    msg_ref = { func, a, b, c };

    // Check that both msgs are the same

    #1;
    `VC_TEST_EQ( three_muldivreq_msg_to_str.tiny_str, msg_test, msg_ref )
    #9;
  end
  endtask

  `VC_TEST_CASE_BEGIN( 2, "TestBasicTinyStr" )
  begin

    // Create mul messages

    t2_do_test( mul,  32'd00, 32'd01, 32'd13 );
    t2_do_test( mul,  32'd42, 32'd01, 32'd14 );
    t2_do_test( mul,  32'd18, 32'd68, 32'd15 );

    // Create div messages

    t2_do_test( div,  32'd00, 32'd01, 32'd23 );
    t2_do_test( div,  32'd42, 32'd01, 32'd24 );
    t2_do_test( div,  32'd18, 32'd68, 32'd25 );

    // Create divu messages

    t2_do_test( divu, 32'd00, 32'd01, 32'd33 );
    t2_do_test( divu, 32'd42, 32'd01, 32'd34 );
    t2_do_test( divu, 32'd18, 32'd68, 32'd35 );

    // Create rem messages

    t2_do_test( rem,  32'd00, 32'd01, 32'd43 );
    t2_do_test( rem,  32'd42, 32'd01, 32'd44 );
    t2_do_test( rem,  32'd18, 32'd68, 32'd45 );

    // Create remu messages

    t2_do_test( remu, 32'd00, 32'd01, 32'd53 );
    t2_do_test( remu, 32'd42, 32'd01, 32'd54 );
    t2_do_test( remu, 32'd18, 32'd68, 32'd55 );

  end
  `VC_TEST_CASE_END

  `VC_TEST_SUITE_END( 2 )
endmodule

