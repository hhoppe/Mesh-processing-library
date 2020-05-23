HhRoot = ..
include $(HhRoot)/make/Makefile_defs

gdloopdir = $(HhRoot)/VideoViewer
extraobjs = $(foreach n,GradientDomainLoop,$(gdloopdir)/$(n).$(extobj))
cppinc += -I$(gdloopdir)#  for GradientDomainLoop.h

include $(HhRoot)/make/Makefile_prog

# Note: causes assembler error message "operand size mismatch for `movhps'" in cygwin and mingw32 (gcc4.7?)
# Filtervideo.$(extobj): cxxall += $(cxx_list_assembly_code)
