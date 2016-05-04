# Makefile (for GNU make, a.k.a. gmake).
# Note: default CONFIG (set in make/Makefile_defs) is either "win" (Windows debug) or "unix" (Unix release),
#  depending on the detected platform.
# Setting "CONFIG=all" runs make successively on all available configurations.
# Examples:
#  make CONFIG=win -j Filtermesh
#  make -j4  # limit parallelism to 4 cores; important on a Virtual Machine
#  make CONFIG=mingw -j demos
#  make makeall   # run all CONFIG   (same as "make CONFIG=all -j")
#  make cleanall  # clean all CONFIG (same as "make CONFIG=all -j deepclean")
#  make everything  # makeall plus progtest
#  make CONFIG=mingw -C ~/src -j8 -t libs  # touch all files

# Test various implementations of gmake:
#  c:/cygwin/bin/make CONFIG=clang -C ~/src -j
#  c:/progra~1/mingw-w64/x86_64-5.2.0-posix-seh-rt_v4-rev0/mingw64/bin/mingw32-make CONFIG=mingw -C ~/src -j
#  c:/MinGW/msys/1.0/bin/make CONFIG=win -C ~/src -j

HhRoot ?= .#  this current file is located in the root directory of the package

include make/Makefile_defs

ifneq ($(CONFIG),all)

libdirs = \
  libHh \
  lib$(HW)
progdirs = \
  Recon Meshfit Subdivfit Polyfit MeshDistance \
  MeshSimplify reverselines Filterprog FilterPM StitchPM \
  MinCycles \
  Filtermesh Filtera3d Filterframe Filterimage Filtervideo \
  G3dOGL G3dVec VideoViewer \

ifeq ($(win_base),1)  # CONFIG = win or w32
endif

ifeq ($(win32),1)
endif

dirs = $(libdirs) $(progdirs)
dirs+test = $(dirs) test demos
dirs+test+all = $(sort $(dirs+test) libHWin libHWX)#  sort to remove duplicates

all: progs test

everything:
	$(MAKE) makeall
	beep && beep

demos: progs                    # run all demos (after building programs)

progs: $(dirs)                  # build all programs

libs: $(libdirs)                # build all libraries

test: $(libdirs)                # run all unit tests (after building libraries)


$(dirs) test demos:    # build any subproject by running make in its subdirectory
	$(MAKE) -C $@

$(progdirs): $(libdirs)         # building a program first requires building libraries

ifeq ($(use_pch),1)             # if using precompiler headers, build libHh before other libraries
  $(filter-out libHh,$(libdirs)): libHh
endif

G3dVec: G3dOGL                  # compile shared files in G3dOGL first
# (to make G3dVec without HH_OGLX, run "make -C ./G3dVec")

Filtervideo: VideoViewer        # compile shared files in VideoViewer first

clean_dirs = $(foreach n,$(dirs+test),clean_$(n))  # pseudo-dependency to allow "make -j clean" parallelism
clean: $(clean_dirs)
$(clean_dirs):
	$(MAKE) -C $(@:clean_%=%) clean

deepclean_dirs = $(foreach n,$(dirs+test),deepclean_$(n))  # pseudo-dependency to allow "make -j deepclean" parallelism
deepclean: $(deepclean_dirs)
$(deepclean_dirs):
	$(MAKE) -C $(@:deepclean_%=%) deepclean

depend_dirs = $(foreach n,$(dirs+test+all),depend_$(n))  # pseudo-dependency to allow "make -j depend" parallelism
depend: $(depend_dirs)
$(depend_dirs):  # allows "make -j" parallelism
	rm -f $(@:depend_%=%)/$(make_dep)
	$(MAKE) -C $(@:depend_%=%) $(make_dep)

TAGS tags:       # build source code index file used by Emacs
#	(allC -h; allC -C) | grep -v Repository | c:/progra~1/emacs/bin/etags - --no-members
# broken; without --no-members, it finds G3d.h for "Mesh".
# still it gives Filterterrain.cpp for "for_int", and "TerrainIO.h" for "Hh"
#	(allC -h; allC -C) | grep -v Repository | c:/hh/bin/sys/etags -C -  # no longer there
#  etags == ctags -e
#  sh -c 'ctags --list-kinds=c++'
#  -e is for Emacs;  -language-force=C++ unnecessary because *.h is already C++;  omit members and namespaces
# This one works in Windows cygwin, but I tried to unify below:
#	(allC -h; allC -C) | grep -v Repository | sh -c 'ctags -L - -e --c++-kinds=-mn --totals=yes'
# This one works on Unix:
#	(allC -h; allC -C) | grep -v Repository | xargs etags --no-members
# This one works in Windows cygwin:
#	(allC -h; allC -C) | grep -v Repository | perl -pe 'binmode(STDOUT); s@^@c:@' | PATH="/cygdrive/c/progra~1/emacs/bin:$$PATH" xargs etags --no-members
# This one should work cross-platform:
	(allC -h -use_rel; allC -C -use_rel) | grep -v Repository | PATH="/cygdrive/c/progra~1/emacs/bin:$$PATH" xargs etags --no-members

debug:
# Note that we can also use $(info any message) to print a message during the parsing of the Makefile.
	@echo '$PATH=$(PATH)'
#	make -p -f /dev/null   # print out all the predefined variables and implicit rules 
#	echo $(COMPILE.c)
#	$(COMPILE.c) --version
#	$(COMPILE.c) '-?'
#	$(CXX)
#	cl '-?'
#	@echo '.FEATURES: $(.FEATURES) .INCLUDE_DIRS: $(.INCLUDE_DIRS)'
#	@echo 'rel=$(rel) x64=$(x64) extobj=$(extobj) extlib=$(extlib) libdirs=($(libdirs))'
#	@echo 'CURDIR=$(CURDIR) PWD=$(PWD) MAKEFLAGS=$(MAKEFLAGS)'
#	@echo '.VARIABLES: $(.VARIABLES)'
#	@echo 'Variables:'$$'\n''$(foreach var,$(.VARIABLES),  $(var) = $($(var))'$$'\n'')'
#	@echo $(CURDIR),$(patsubst c:%,%,$(PWD))

# location of executable used for timing test
rel_exe_dir = $(if $(CONFIG:win=),bin/$(CONFIG),bin)#  CONFIG=win instead uses the release exe built by msbuild

timingtest: Filterimage Filtervideo
# hhopped   win: expect 1.0 sec, 22 sec (was 25 sec)
# hhopped mingw: expect 1.0 sec, 20 sec
# hhoppew   win: expect 1.0 sec, 15 sec
# hhoppew mingw: expect 0.8 sec, 13 sec
#  mingw32, clang: out-of-memory may result in segmentation fault
# See also HTest -timescaleimage
	$(rel_exe_dir)/Filterimage -create 8192 8192 -scaletox 3000 -noo
#	GDLOOP_USE_VECTOR4=1 $(rel_exe_dir)/Filtervideo -create 215 1920 1080 -framerate 30 -end 7sec -start -5sec -trimend -1 -loadvlp ~/proj/videoloops/data/ReallyFreakinAll/out/HDgiant_loop.vlp -gdloop 5sec -noo) |& grep '(_gdloop:'
	VIDEOLOOP_PRECISE=1 $(rel_exe_dir)/Filtervideo -create 215 1920 1080 -framerate 30 -end 7sec -start -6sec -trimend -1 -loadvlp ~/proj/videoloops/data/ReallyFreakinAll/out/HDgiant_loop.downscaled.vlp -gdloop 5sec -noo |& grep '(_gdloop:'

.PHONY: all everything progs libs $(dirs+test) clean $(clean_dirs) \
  deepclean $(deepcleandirs) depend $(depend_dirs) TAGS tags debug timingtest

endif  # ifneq ($(CONFIG),all)
