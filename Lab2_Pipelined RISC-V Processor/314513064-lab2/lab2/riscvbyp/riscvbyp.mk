#=========================================================================
# riscvbyp Subpackage
#=========================================================================

riscvbyp_deps = \
  vc \
  imuldiv \

riscvbyp_srcs = \
  riscvbyp-CoreDpath.v \
  riscvbyp-CoreDpathRegfile.v \
  riscvbyp-CoreDpathAlu.v \
  riscvbyp-CoreCtrl.v \
  riscvbyp-Core.v \
  riscvbyp-InstMsg.v \
  riscvbyp-CoreDpathPipeMulDiv.v \

riscvbyp_test_srcs = \
  riscvbyp-InstMsg.t.v \
  riscvbyp-CoreDpathPipeMulDiv.t.v \

riscvbyp_prog_srcs = \
  riscvbyp-sim.v \
  riscvbyp-randdelay-sim.v \

