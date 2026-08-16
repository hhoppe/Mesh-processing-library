// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HH_INIT_H_
#define MESH_PROCESSING_LIBHH_HH_INIT_H_
#include "libHh/Hh.h"

#include <iostream>  // Ensure std::ios_base::Init precedes dummy_hh_init in every translation unit.

namespace hh::details {

struct HhInit {
  HhInit();
};

// Ensure hh_init() in Hh_init.cpp runs before any dynamic initialization declared later in each translation unit.
[[maybe_unused]] static const HhInit dummy_hh_init;

}  // namespace hh::details

#endif  // MESH_PROCESSING_LIBHH_HH_INIT_H_
