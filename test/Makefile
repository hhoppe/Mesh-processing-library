# Use make (GNU gmake), e.g.:
#  make CONFIG=win -j8
#  make clean tStack.ou

HhRoot = ..

new_all: test

include $(HhRoot)/make/Makefile_progs

ifeq ($(gcc_base),1)
  # For the test programs in this directory, reduce floating-point imprecision issues.
  ccommon += $(gcc_safe_precise_math)
endif

# for assembly code, use ./test/opt/ instead
# tMap: cxxall += $(cxx_list_assembly_code)
# tVector4: cxxall += $(cxx_list_assembly_code)
# tNetworkOrder: cxxall += $(cxx_list_assembly_code)

# Precompiled headers:
# - gcc and clang already benefit from "cxxall += -include $(HhRoot)/libHh/precompiled_libHh.h"
#    under all $(HhRoot) directories
# - win is too difficult; it would not work with multiple executables in the same directory, each requiring its
#    own pdb file.


# For CONFIG=mingw32:
# (set path=('/Program Files (x86)/mingw-builds/x32-4.8.1-posix-dwarf-rev5/mingw32/bin' $path:q); gdb --batch -ex run -ex where --args tMultigrid)
