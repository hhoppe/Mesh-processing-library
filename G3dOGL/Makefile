MeshRoot = ..
include $(MeshRoot)/make/Makefile_defs

# $(info 0 LDLIBS=$(LDLIBS) value(LDLIBS)=$(value LDLIBS))
$(call prepend,LDLIBS,$(MeshRoot)/lib$(HW)/lib$(HW).$(extlib) $(gfx_libs) )
cppinc += -I$(MeshRoot)/lib$(HW)
# $(info 1 LDLIBS=$(LDLIBS) value(LDLIBS)=$(value LDLIBS))

include $(MeshRoot)/make/Makefile_prog
