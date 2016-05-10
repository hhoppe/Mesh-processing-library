HhRoot = ..
include $(HhRoot)/make/Makefile_defs

# $(info 0 LDLIBS=$(LDLIBS) value(LDLIBS)=$(value LDLIBS))
$(call prepend,LDLIBS,$(HhRoot)/lib$(HW)/lib$(HW).$(extlib) $(gfx_libs) )
cppinc += -I$(HhRoot)/lib$(HW)
# $(info 1 LDLIBS=$(LDLIBS) value(LDLIBS)=$(value LDLIBS))

include $(HhRoot)/make/Makefile_prog
