// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BOUNDARY_H_
#define MESH_PROCESSING_LIBHH_BOUNDARY_H_

#include "libHh/Array.h"

namespace hh {

// A rule that defines the samples of a 1D grid beyond its domain, for use in filtering (resizing or sampling).
// It replaces the taps of a kernel row that fall outside the grid by combinations of interior samples (and possibly
// of a constant value `cval`), following the Python package `resampler`.  Instances are immortal named singletons
// obtained through Boundary::get(); user-defined rules derive from this class.
class Boundary : noncopyable {
 public:
  // reflect:  (r, reflected) samples are reflected across the domain boundaries (101 reflection on a primal grid).
  // wrap:     (w, periodic) samples repeat periodically (with period size - 1 on a primal grid).
  // clamp:    (c, clamped) the nearest interior sample is repeated.
  // border:   (b) samples outside the domain have the constant value cval.
  // auto:     reflect when magnifying or point sampling, clamp when minifying (resolved before use).
  [[HH_NO_DANGLING]] [[nodiscard]] static const Boundary& get(std::string_view name);
  [[HH_NO_DANGLING]] [[nodiscard]] static const Boundary& get(Bndrule bndrule);
  [[nodiscard]] const string& name() const { return _name; }
  // True if some folded rows reference cval, i.e., their interior weights may not sum to 1.
  [[nodiscard]] bool uses_cval() const { return _uses_cval; }
  [[nodiscard]] bool is_auto() const { return _is_auto; }
  // Modify a unit-domain coordinate before the kernel is evaluated (identity except for "tile").
  [[nodiscard]] virtual double remap_coordinate(double x) const { return x; }
  // Fold the dense row of taps at indices [index0, index0 + weights.num()) into interior indices of a grid of `size`
  // samples, appending (index, weight) pairs to `indices` and `folded`; the same index may appear several times,
  // and the caller merges them.  `x` is the unit-domain coordinate of the sample being reconstructed, used only by
  // the rules that override the value outside some extent.
  virtual void fold(int index0, CArrayView<float> weights, int size, Gridtype gridtype, double x, Array<int>& indices,
                    Array<float>& folded) const = 0;
  friend std::ostream& operator<<(std::ostream& os, const Boundary& boundary) {
    return os << "Boundary{" << boundary.name() << "}";
  }

 protected:
  explicit Boundary(string name) : _name(std::move(name)) {}
  virtual ~Boundary() = default;
  bool _uses_cval{false};
  bool _is_auto{false};

 private:
  string _name;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BOUNDARY_H_
