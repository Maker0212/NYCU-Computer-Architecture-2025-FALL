#=========================================================================
# riscvooo Subpackage
#=========================================================================

riscvooo_deps = \
  vc \
  imuldiv \
  pcache \

riscvooo_srcs = \
  riscvooo-CoreDpath.v \
  riscvooo-CoreDpathRegfile.v \
  riscvooo-CoreDpathAlu.v \
  riscvooo-CoreScoreboard.v \
  riscvooo-CoreReorderBuffer.v \
  riscvooo-CoreCtrl.v \
  riscvooo-Core.v \
  riscvooo-InstMsg.v \

riscvooo_test_srcs = \
  riscvooo-InstMsg.t.v \

riscvooo_prog_srcs = \
  riscvooo-sim.v \
  riscvooo-randdelay-sim.v \

