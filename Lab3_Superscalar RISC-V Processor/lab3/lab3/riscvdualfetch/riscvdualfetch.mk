#=========================================================================
# riscvdualfetch Subpackage
#=========================================================================

riscvdualfetch_deps = \
  vc \
  imuldiv \

riscvdualfetch_srcs = \
  riscvdualfetch-CoreDpath.v \
  riscvdualfetch-CoreDpathRegfile.v \
  riscvdualfetch-CoreDpathAlu.v \
  riscvdualfetch-CoreDpathPipeMulDiv.v \
  riscvdualfetch-CoreCtrl.v \
  riscvdualfetch-CoreScoreboard.v \
  riscvdualfetch-Core.v \
  riscvdualfetch-InstMsg.v \

riscvdualfetch_test_srcs = \
  riscvdualfetch-InstMsg.t.v \
  riscvdualfetch-CoreDpathPipeMulDiv.t.v \

riscvdualfetch_prog_srcs = \
  riscvdualfetch-sim.v \
  riscvdualfetch-randdelay-sim.v \

