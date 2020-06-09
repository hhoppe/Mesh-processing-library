MeshRoot = ..
include $(MeshRoot)/make/Makefile_defs

$(call prepend,LDLIBS,$(MeshRoot)/lib$(HW)/lib$(HW).$(extlib) $(graphics_libs) )
includes += -I$(MeshRoot)/lib$(HW)

g3dogl_dir = $(MeshRoot)/G3dOGL
extra_objs = $(foreach n,G3d G3ddraw G3devent G3dio,$(g3dogl_dir)/$(n).$(extobj))

include $(MeshRoot)/make/Makefile_prog
