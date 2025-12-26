#=========================================================================
# riscvbc Subpackage
#=========================================================================

riscvbc_deps = \
  vc \
  imuldiv \
  riscvlong \

riscvbc_srcs = \
  riscvbc-CacheBase.v \
  riscvbc-CacheIcache.v \
  riscvbc-CacheAlt.v \
  riscvbc-CacheDcache.v \
  riscvbc-CacheAll.v \
  riscvbc-CacheBypass.v \
  riscvbc-CacheNone.v \

riscvbc_test_srcs = \

riscvbc_prog_srcs = \
  riscvbc-sim.v \
  riscvbc-randdelay-sim.v \

