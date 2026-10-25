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
  // reflect:            (r, reflected) samples reflect across the domain boundaries (101 reflection on a primal grid).
  // wrap:               (w, periodic) samples repeat periodically (with period size - 1 on a primal grid).
  // tile:               like reflect within the unit domain, which is then tiled (discontinuously).
  // clamp:              (c, clamped) the nearest interior sample is repeated.
  // border:             (b) samples outside the domain have the constant value cval.
  // natural:            the reconstruction uses only the interior samples (renormalized); cval outside the domain.
  // linear:             exterior samples are extrapolated linearly from the two nearest samples.
  // quadratic:          exterior samples are extrapolated quadratically from the three nearest samples.
  // linear_constant:    like linear, but cval outside the unit domain.
  // quadratic_constant: like quadratic, but cval outside the unit domain.
  // reflect_clamp:      samples reflect once across each boundary, then clamp (mirror-clamp-to-edge).
  // constant:           like reflect, but cval outside the unit domain.
  // auto:               reflect when magnifying or point sampling, clamp when minifying (resolved before use).
  [[HH_NO_DANGLING]] [[nodiscard]] static const Boundary& get(std::string_view name);
  [[HH_NO_DANGLING]] [[nodiscard]] static const Boundary& get(Bndrule bndrule);
  [[nodiscard]] const string& name() const { return _name; }
  // True if some folded rows reference cval, i.e., their interior weights may not sum to 1.
  [[nodiscard]] bool uses_cval() const { return _uses_cval; }
  [[nodiscard]] bool is_auto() const { return _is_auto; }
  // True if the rule overrides the value outside the unit domain (natural, constant, *_constant).
  [[nodiscard]] bool overrides_exterior() const { return _overrides_exterior; }
  // Modify a unit-domain coordinate before the kernel is evaluated (identity except for "tile").
  [[nodiscard]] virtual double remap_coordinate(double x) const { return x; }
  // Fold the dense row of taps at indices [index0, index0 + weights.num()) into interior indices of a grid of `size`
  // samples, appending (index, weight) pairs to `indices` and `folded` in the order of the taps (so that their sum
  // matches a direct evaluation); the same index may appear several times, and an index of -1 denotes the constant
  // value cval.  The rules that override the value outside the unit domain use `x`, the unit-domain coordinate of
  // the point being reconstructed, and `spacing`, the distance between consecutive reconstruction points (0 for an
  // isolated point), which antialiases the transition to cval.
  virtual void fold(int index0, CArrayView<float> weights, int size, Gridtype gridtype, double x, double spacing,
                    Array<int>& indices, Array<float>& folded) const = 0;
  friend std::ostream& operator<<(std::ostream& os, const Boundary& boundary) {
    return os << "Boundary{" << boundary.name() << "}";
  }

 protected:
  explicit Boundary(string name) : _name(std::move(name)) {}
  virtual ~Boundary() = default;
  bool _uses_cval{false};
  bool _is_auto{false};
  bool _overrides_exterior{false};

 private:
  string _name;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BOUNDARY_H_
