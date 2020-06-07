MeshRoot = ..
include $(MeshRoot)/make/Makefile_defs

gdloop_dir = $(MeshRoot)/VideoViewer
extra_objs = $(foreach n,GradientDomainLoop,$(gdloop_dir)/$(n).$(extobj))

include $(MeshRoot)/make/Makefile_prog

# Note: caused assembler error message "operand size mismatch for `movhps'" in gcc4.7.
# Filtervideo.$(extobj): cxxall += $(cxx_list_assembly_code)
