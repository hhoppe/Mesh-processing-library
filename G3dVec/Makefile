HhRoot = ..
include $(HhRoot)/make/Makefile_defs

$(call prepend,LDLIBS,$(HhRoot)/lib$(HW)/lib$(HW).$(extlib) $(gfx_libs) )
cppinc += -I$(HhRoot)/lib$(HW)
g3dogldir = $(HhRoot)/G3dOGL
extraobjs = $(foreach n,G3d G3ddraw G3devent G3dio,$(g3dogldir)/$(n).$(extobj))

include $(HhRoot)/make/Makefile_prog
