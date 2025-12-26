#=========================================================================
# imuldiv Subpackage
#=========================================================================

imuldiv_deps = vc

imuldiv_srcs = \
  imuldiv-IntMulIterative.v \
  imuldiv-IntDivIterative.v \
  imuldiv-IntMulDivIterative.v \
  imuldiv-IntMulDivSingleCycle.v \
  imuldiv-IntMulBooth.v \
  imuldiv-IntMulThreeInput.v \
  imuldiv-ThreeMulReqMsg.v \

imuldiv_test_srcs = \
  imuldiv-IntMulIterative.t.v \
  imuldiv-IntDivIterative.t.v \
  imuldiv-IntMulDivIterative.t.v \
  imuldiv-IntMulDivSingleCycle.t.v \
  imuldiv-IntMulBooth.t.v \
  imuldiv-IntMulThreeInput.t.v \
  imuldiv-ThreeMulReqMsg.t.v \
  
imuldiv_prog_srcs = \
  imuldiv-singcyc-sim.v \
  imuldiv-iterative-sim.v \
  imuldiv-booth-sim.v \
