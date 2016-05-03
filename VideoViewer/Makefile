HhRoot = ..

# DEBUG_MAKE = 1

include $(HhRoot)/make/Makefile_prog

$(call prepend,LDLIBS,$(HhRoot)/lib$(HW)/lib$(HW).$(extlib) $(gfx_libs) )
cppinc += -I$(HhRoot)/lib$(HW)

# Creating a windows app (rather than console app) seems a bad idea as it disconects from shell
#  and does not launch subprocesses (RFile / WFile / my_system) without constructing new consoles anyways.
# Instead, I use "ShowWindow(GetConsoleWindow(), SW_HIDE);" within libHWin/HW.cpp
# LDFLAGS += -subsystem:windows -entry:mainCRTStartup

