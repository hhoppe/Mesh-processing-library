HhRoot = ..
include $(HhRoot)/Makefile_lib

# some_raw_c_file.$(extobj): cwarnings := $(no_warnings)

Video.$(extobj): cppinc += $(if $(VisionToolsRoot),-I$(VisionToolsRoot)/inc,)
# Video.$(extobj): cxxall += -Wno-unknown-pragmas

# MathOp.$(extobj): cxxall += $(cxx_list_assembly_code)

ifeq ($(gcc_base),1)
  # Selectively increase floating-point precision to reduce differences in the testing programs in ../test
  MeshSearch.$(extobj) Random.$(extobj) Polygon.$(extobj) : ccommon += $(gcc_safe_precise_math)
endif

ifeq ($(win_base)-pch$(use_pch),1-pch1)  # CONFIG = win or w32
  pch_root = precompiled_libHh
  abs_pch_file = $(call add_drive,$(abspath $(pch_root))).h
  # We must wait until the precompiled header is generated before compiling other files.
  $(filter-out $(pch_root).$(extobj),$(obj)) : | $(pch_root).$(extobj)
  # Create precompiled_libHh.pch when compiling this file.
  $(pch_root).$(extobj) : cxxall += -Yc$(abs_pch_file)
  ifeq ($(rel),0)
    # Even in debug configuration, we can benefit from precompiled headers in this directory due to the shared pdb file.
    # Use the named precompiled header, and force-include it into every *.cpp file.
    cxxall += -Yu$(abs_pch_file) -FI$(abs_pch_file)
  endif
endif

ifeq ($(CXX)-pch$(use_pch),g++-pch1)
  pch_root = precompiled_libHh
  abs_pch_file = $(pch_root).h
  # We must wait until the precompiled header is generated before compiling other files.
  $(filter-out $(pch_root).$(extobj),$(obj)) : | $(abs_pch_file).gch
  # Make the precompiled header dependent on the CONFIG, and inherit all dependencies in Makefile.dep.
  $(abs_pch_file).gch : built_ob.$(extobj).$(CONFIG) $(pch_root).$(extobj)
  # Omit precompiled header when compiling precompiled header file
  $(abs_pch_file).gch : cxxall := $(filter-out -include $(HhRoot)/libHh/precompiled_libHh.h,$(cxxall))
endif

ifeq ($(CXX)-pch$(use_pch),clang++-pch1)
  pch_root = precompiled_libHh
  abs_pch_file = $(pch_root).h
  # We must wait until the precompiled header is generated before compiling other files.
  $(filter-out $(pch_root).$(extobj),$(obj)) : | $(abs_pch_file).pch
  # Make the precompiled header dependent on the CONFIG, and inherit all dependencies in Makefile.dep.
  $(abs_pch_file).pch : built_ob.$(extobj).$(CONFIG) $(pch_root).$(extobj)
  # Omit precompiled header when compiling precompiled header file
  # No, this misbehaves: it results in errors when compiling ./libImageIO/ImageIO.o
  # Instead, I removed the *.h.pch using a "rm" in the "%.pch : %.h" rule of Makefile_defs_clang
  # $(abs_pch_file).pch : cxxall := $(filter-out -include $(HhRoot)/libHh/precompiled_libHh.h,$(cxxall))
  # This is necessary because $(pch_root).$(extobj) gets built before $(abs_pch_file).pch
  $(pch_root).$(extobj) : cxxall := $(filter-out -include $(HhRoot)/libHh/precompiled_libHh.h,$(cxxall))
endif
