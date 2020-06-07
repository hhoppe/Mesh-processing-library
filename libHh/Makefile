MeshRoot = ..
include $(MeshRoot)/make/Makefile_lib

Video.$(extobj): includes += $(if $(VisionToolsRoot),-I$(VisionToolsRoot)/inc,)

# MathOp.$(extobj): cxxall += $(cxx_list_assembly_code)

ifeq ($(gcc_or_clang),1)
  # Selectively increase floating-point precision to reduce differences in the testing programs in ../test
  MeshSearch.$(extobj) Random.$(extobj) Polygon.$(extobj) : ccommon += $(gcc_safe_precise_math)
endif

ifeq ($(use_pch),1)
  pch_root = precompiled_libHh
  ifeq ($(win_base),1)  # CONFIG = win or w32
    abs_pch_file = $(call add_drive,$(abspath $(pch_root))).h
    # We must wait until the precompiled header is generated before compiling other files.
    $(filter-out $(pch_root).$(extobj),$(obj)) : | $(pch_root).$(extobj)
    # Create precompiled_libHh.pch when compiling this file.
    $(pch_root).$(extobj) : cxxall += -Yc$(abs_pch_file)
    # For release configuration, Makefile_base_win already force-includes the pch into all *.cpp files.
    ifeq ($(rel),0)
      # In debug configuration, for this directory only, we can use precompiled headers due to the shared pdb file.
      cxxall += -Yu$(abs_pch_file) -FI$(abs_pch_file)
    endif
  endif
  ifeq ($(CXX),g++)
    abs_pch_file = $(pch_root).h
    # We must wait until the precompiled header is generated before compiling other files.
    $(filter-out $(pch_root).$(extobj),$(obj)) : | $(abs_pch_file).gch
    # Make the precompiled header dependent on the CONFIG, and inherit all dependencies in Makefile.dep.
    $(abs_pch_file).gch : built_ob.$(extobj).$(CONFIG) $(pch_root).$(extobj)
    # Omit precompiled header when compiling precompiled header file
    $(abs_pch_file).gch : cxxall := $(filter-out -include $(MeshRoot)/libHh/precompiled_libHh.h,$(cxxall))
  endif
  ifeq ($(CXX),clang++)
    abs_pch_file = $(pch_root).h
    # We must wait until the precompiled header is generated before compiling other files.
    $(filter-out $(pch_root).$(extobj),$(obj)) : | $(abs_pch_file).pch
    # Make the precompiled header dependent on the CONFIG, and inherit all dependencies in Makefile.dep.
    $(abs_pch_file).pch : built_ob.$(extobj).$(CONFIG) $(pch_root).$(extobj)
    # Omit precompiled header when compiling precompiled header file
    # No, this misbehaves: it results in errors when compiling ./libImageIO/ImageIO.o
    # Instead, I removed the *.h.pch using a "rm" in the "%.pch : %.h" rule of make/Makefile_defs_clang
    # $(abs_pch_file).pch : cxxall := $(filter-out -include $(MeshRoot)/libHh/precompiled_libHh.h,$(cxxall))
    # This is necessary because $(pch_root).$(extobj) gets built before $(abs_pch_file).pch
    $(pch_root).$(extobj) : cxxall := $(filter-out -include $(MeshRoot)/libHh/precompiled_libHh.h,$(cxxall))
  endif
endif
