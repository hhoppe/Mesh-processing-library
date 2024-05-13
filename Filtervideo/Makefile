MeshRoot = ..
include $(MeshRoot)/make/Makefile_defs

gdloop_dir = $(MeshRoot)/VideoViewer
extra_objs = $(foreach n,GradientDomainLoop,$(gdloop_dir)/$(n).$(extobj))

include $(MeshRoot)/make/Makefile_prog

# Filtervideo.$(extobj): cxxall += $(cxx_list_assembly_code)
