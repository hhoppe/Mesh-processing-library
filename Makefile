# Makefile (for GNU make, a.k.a. gmake).
# Note: the default CONFIG (set in make/Makefile_defs) is either "win" (Windows debug) or "unix" (Unix release),
#  depending on the detected platform.
# Setting "CONFIG=all" runs the make process successively on all available configurations.
# Examples:
#  make -j12 demos
#  make -j12 Filtermesh  # Builds single program using default CONFIG (either "win" or "unix").
#  make -j4  # Limit parallelism to 4 cores; important on a Virtual Machine.
#  make CONFIG=mingw -j12 demos
#  make makeall   # Run all CONFIG   (same as "make CONFIG=all -j12").
#  make cleanall  # Clean all CONFIG (same as "make CONFIG=all -j12 deepclean").
#  make CONFIG=cygwin CC=clang -j12  # Use clang compiler under cygwin.
#  make CONFIG=clang CXX_STD=c++26 -j12  # Test code compatibility with C++26.
#  make CONFIG=win CXX_STD=c++latest -j12  # Test compatibility with proposed c++ features.
#  make CONFIG=all PEDANTIC=1  # Enable most compiler warnings.
#  make CXX_STD=c++26 PEDANTIC=1 -j12
#  make CC=gcc CXX_STD=c++26 PEDANTIC=1 -j12
#  make CONFIG=all debug  # Show the compiler paths and versions.
#  make SHELL='sh -x -v'  # Show all executed commands.

MeshRoot ?= .#  This current file is located in the root directory of the package.

include $(MeshRoot)/make/Makefile_defs

ifneq ($(CONFIG),all)  # For the rest of this file.

lib_dirs = \
  libHh \
  lib$(Hw)#  Either libHwWindows (Windows program) or libHwX (X11 program).

prog_dirs = \
  Recon Meshfit Subdivfit Polyfit MeshDistance \
  MeshSimplify reverselines Filterprog FilterPM StitchPM \
  MinCycles MeshReorder \
  SphereParam SphereSample \
  Filtermesh Filtera3d Filterframe Filterimage Filtervideo \
  G3dOGL G3dVec VideoViewer \

dirs = $(lib_dirs) $(prog_dirs)
dirs+test = $(dirs) test demos
dirs+test+all = $(sort $(dirs+test) libHwWindows libHwX)#  Sort to remove duplicates.

all: progs test

demos: progs                    # Run all demos (after building programs).

progs: $(dirs)                  # Build all programs.

libs: $(lib_dirs)               # Build all libraries.

test: $(lib_dirs)               # Run all unit tests (after building libraries).


$(dirs+test):                   # Build any subproject by running make in its subdirectory.
	$(MAKE) -C $@

$(prog_dirs): $(lib_dirs)       # Building a program first requires building the libraries.

ifeq ($(use_pch),1)             # If using precompiler headers, build libHh before other libraries.
  $(filter-out libHh,$(lib_dirs)): libHh
endif

G3dVec: G3dOGL                  # Compile the shared files in G3dOGL first.
# (To make G3dVec without HH_OGLX, run "make -C ./G3dVec".)

Filtervideo: VideoViewer        # Compile shared files in VideoViewer first.

clean_dirs = $(foreach n,$(dirs+test),clean_$(n))  # Pseudo-dependency to enable "make -j clean" parallelism.
clean: $(clean_dirs)
$(clean_dirs):
	$(MAKE) -C $(@:clean_%=%) -s clean

deepclean_dirs = $(foreach n,$(dirs+test),deepclean_$(n))  # Pseudo-dep. to enable "make -j deepclean" parallelism.
deepclean: $(deepclean_dirs)
$(deepclean_dirs):
	$(MAKE) -C $(@:deepclean_%=%) -s deepclean

depend_dirs = $(foreach n,$(dirs+test+all),depend_$(n))  # Pseudo-dep. to enable "make -j depend" parallelism.
depend: $(depend_dirs)
$(depend_dirs):                 # Enables "make -j" parallelism.
	rm -f $(@:depend_%=%)/$(make_dep)
	$(MAKE) -C $(@:depend_%=%) $(make_dep)

debug:
#	$(info LDLIBS=$(LDLIBS) value(LDLIBS)=$(value LDLIBS))
	@echo 'PATH=$(PATH)'
	@which $(CXX)
	$(CXX) --version

# Location of executable used for timing test.
rel_exe_dir = $(if $(CONFIG:win=),bin/$(CONFIG),bin)#  CONFIG=win instead uses the release exe created by msbuild.

timingtest: Filterimage Filtervideo
# hhoppeg   win: expect 0.40 sec, 7.5 sec
# hhoppeg mingw: expect 0.42 sec, 7.2 sec
# hhoppeg clang: expect 0.42 sec, 7.1 sec
# hhoppeg  unix: expect 0.39 sec, 7.0 sec
	@for i in {1..5}; do $(rel_exe_dir)/Filterimage -create 8192 8192 -scaletox 3000 -noo; done
ifeq ($(x64),1)
	@for i in {1..2}; do VIDEOLOOP_PRECISE=1 $(rel_exe_dir)/Filtervideo -create 215 1920 1080 -framerate 30 -end 7sec -start -6sec -trimend -1 -loadvlp ~/prevproj/2013/videoloops/data/ReallyFreakinAll/out/HDgiant_loop.downscaled.vlp -gdloop 5sec -noo 2>&1 | grep '(_gdloop:'; done
endif
#	GDLOOP_USE_VECTOR4=1 $(rel_exe_dir)/Filtervideo -create 215 1920 1080 -framerate 30 -end 7sec -start -5sec -trimend -1 -loadvlp ~/prevproj/2013/videoloops/data/ReallyFreakinAll/out/HDgiant_loop.vlp -gdloop 5sec -noo 2>&1 | grep '(_gdloop:'

.PHONY: all progs libs $(dirs+test) clean $(clean_dirs) \
  deepclean $(deepclean_dirs) depend $(depend_dirs) debug timingtest

endif  # ifneq ($(CONFIG),all)
