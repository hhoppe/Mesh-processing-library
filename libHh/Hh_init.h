// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HH_INIT_H_
#define MESH_PROCESSING_LIBHH_HH_INIT_H_
#include "libHh/Hh.h"

// Make sure that hh::details::hh_init() is called before any other function.

namespace hh {
namespace details {
int hh_init();
[[maybe_unused]] static int dummy_init_hh = hh_init();
}  // namespace details
}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_HH_INIT_H_
