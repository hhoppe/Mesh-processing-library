// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HASHPOINT_H_
#define MESH_PROCESSING_LIBHH_HASHPOINT_H_

#include "libHh/Geometry.h"
#include "libHh/HashFloat.h"
#include "libHh/Pool.h"
#include "libHh/Set.h"

namespace hh {

// Override parameters using getenv_int("HASHFLOAT_NIGNOREBITS") and getenv_float("HASHFLOAT_SMALL").

// Robustly hash similar 3D points.
class HashPoint {
 public:
  explicit HashPoint(int nignorebits = 8, float small = 1e-4f) {
    for_int(c, 3) _hf[c] = make_unique<HashFloat>(nignorebits, small);
  }
  // p is copied, ret: index (first is 0)
  int enter(const Point& p) {
    Point pp;
    for_int(c, 3) pp[c] = _hf[c]->enter(p[c]);
    bool is_new;
    int index = _mpi.enter(pp, _index, is_new);
    if (is_new) _index++;
    return index;
  }
  // more robust pre-pass
  void pre_consider(const Point& p) { for_int(c, 3) _hf[c]->pre_consider(p[c]); }

 private:
  Vec3<unique_ptr<HashFloat>> _hf;
  static uint32_t float_bits_to_unsigned(const float& f) {
    union {
      uint32_t ui;
      float f;
    } u;
    u.f = f;
    return u.ui;
  }
  struct hash_Point {
    size_t operator()(const Point& p) const {
      size_t h = 0;
      for_int(c, 3) h = h * 17 + float_bits_to_unsigned(p[c]);
      return h;
    }
  };
  struct equal_Point {
    bool operator()(const Point& p1, const Point& p2) const { return p1 == p2; }
  };
  Map<Point, int, hash_Point, equal_Point> _mpi;
  int _index{0};  // current index, assigned to next new point
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_HASHPOINT_H_
