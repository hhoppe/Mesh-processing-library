HhRoot = ..
include $(HhRoot)/make/Makefile_prog

gdloopdir = $(HhRoot)/VideoViewer
extraobjs = $(foreach n,GradientDomainLoop,$(gdloopdir)/$(n).$(extobj))
cppinc += -I$(gdloopdir)#  for GradientDomainLoop.h

# Note: causes assembler error messhage "operand size mismatch for `movhps'" in cygwin and mingw32 (gcc4.7?)
# Filtervideo.$(extobj): cxxall += $(cxx_list_assembly_code)
