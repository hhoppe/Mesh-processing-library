// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_LINEARFUNC_H_
#define MESH_PROCESSING_LIBHH_LINEARFUNC_H_

#include "libHh/Geometry.h"

namespace hh {

// Linear function in 3D, represented as a vector and scalar offset.
struct LinearFunc {
  Vector v;
  float offset{0.f};
  LinearFunc() = default;
  LinearFunc(Vector pv, const Point& pp) : v(std::move(pv)), offset(-dot(pp, v)) {}
  float eval(const Point& p) const { return dot(p, v) + offset; }
  void add(const LinearFunc& lf) { v += lf.v, offset += lf.offset; }
  friend std::ostream& operator<<(std::ostream& os, const LinearFunc& lf) {
    return os << "LinearFunc(v=" << lf.v << ", offset=" << lf.offset << ")";
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_LINEARFUNC_H_
