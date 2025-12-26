#=========================================================================
# riscvstall Subpackage
#=========================================================================

riscvstall_deps = \
  vc \
  imuldiv \

riscvstall_srcs = \
  riscvstall-CoreDpath.v \
  riscvstall-CoreDpathRegfile.v \
  riscvstall-CoreDpathAlu.v \
  riscvstall-CoreCtrl.v \
  riscvstall-Core.v \
  riscvstall-InstMsg.v \
  riscvstall-CoreDpathPipeMulDiv.v \

riscvstall_test_srcs = \
  riscvstall-InstMsg.t.v \
  riscvstall-CoreDpathPipeMulDiv.t.v \

riscvstall_prog_srcs = \
  riscvstall-sim.v \
  riscvstall-randdelay-sim.v \

