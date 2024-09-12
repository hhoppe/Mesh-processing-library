MeshRoot = ..
include $(MeshRoot)/make/Makefile_defs

$(call prepend,LDLIBS,$(MeshRoot)/lib$(Hw)/lib$(Hw).$(extlib) $(graphics_libs) )
includes += -I$(MeshRoot)/lib$(Hw)

include $(MeshRoot)/make/Makefile_prog
