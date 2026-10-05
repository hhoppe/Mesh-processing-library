MeshRoot = ../..
include $(MeshRoot)/make/Makefile_defs

gdloop_dir = $(MeshRoot)/progs/VideoViewer
extra_objs = $(foreach n,GradientDomainLoop,$(gdloop_dir)/$(n).$(extobj))

include $(MeshRoot)/make/Makefile_prog
