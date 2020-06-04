HhRoot = ..
include $(HhRoot)/make/Makefile_defs

gdloopdir = $(HhRoot)/VideoViewer
extraobjs = $(foreach n,GradientDomainLoop,$(gdloopdir)/$(n).$(extobj))

include $(HhRoot)/make/Makefile_prog

# Note: caused assembler error message "operand size mismatch for `movhps'" in gcc4.7.
# Filtervideo.$(extobj): cxxall += $(cxx_list_assembly_code)
