#=========================================================================
# riscvssc Subpackage
#=========================================================================

riscvssc_deps = \
  vc \
  imuldiv \

riscvssc_srcs = \
  riscvssc-CoreDpath.v \
  riscvssc-CoreDpathRegfile.v \
  riscvssc-CoreDpathAlu.v \
  riscvssc-CoreDpathPipeMulDiv.v \
  riscvssc-CoreCtrl.v \
  riscvssc-CoreScoreboard.v \
  riscvssc-Core.v \
  riscvssc-InstMsg.v \

riscvssc_test_srcs = \
  riscvssc-InstMsg.t.v \
  riscvssc-CoreDpathPipeMulDiv.t.v \

riscvssc_prog_srcs = \
  riscvssc-sim.v \
  riscvssc-randdelay-sim.v \

