HhRoot = ..
include $(HhRoot)/Makefile_defs

LDLIBS := $(HhRoot)/lib$(HW)/lib$(HW).$(extlib) $(gfx_libs) $(LDLIBS)
cppinc += -I$(HhRoot)/lib$(HW)

g3dogldir = $(HhRoot)/G3dOGL
extraobjs = $(foreach n,G3d G3ddraw G3devent G3dio,$(g3dogldir)/$(n).$(extobj))
cppinc += -I$(g3dogldir)#  for HB.h

include $(HhRoot)/Makefile_prog
