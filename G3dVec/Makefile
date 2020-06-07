MeshRoot = ..
include $(MeshRoot)/make/Makefile_defs

$(call prepend,LDLIBS,$(MeshRoot)/lib$(HW)/lib$(HW).$(extlib) $(gfx_libs) )
cppinc += -I$(MeshRoot)/lib$(HW)
g3dogldir = $(MeshRoot)/G3dOGL
extraobjs = $(foreach n,G3d G3ddraw G3devent G3dio,$(g3dogldir)/$(n).$(extobj))

include $(MeshRoot)/make/Makefile_prog
