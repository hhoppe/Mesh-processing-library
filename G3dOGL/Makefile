MeshRoot = ..
include $(MeshRoot)/make/Makefile_defs

$(call prepend,LDLIBS,$(MeshRoot)/lib$(HW)/lib$(HW).$(extlib) $(graphics_libs) )
includes += -I$(MeshRoot)/lib$(HW)

include $(MeshRoot)/make/Makefile_prog
