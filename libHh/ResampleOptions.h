// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_RESAMPLEOPTIONS_H_
#define MESH_PROCESSING_LIBHH_RESAMPLEOPTIONS_H_

#include <optional>

#include "libHh/Boundary.h"
#include "libHh/Filter.h"
#include "libHh/Gamma.h"
#include "libHh/Vec.h"

namespace hh {

// A reference to one of the named singletons of T (Filter, Boundary, Gamma), settable from a name, a reference, or a
// pointer; an unrecognized name fails loudly at construction.
template <typename T> struct NamedRef {
  const T* ptr;
  NamedRef(std::string_view name) : ptr(&T::get(name)) {}           // NOLINT(google-explicit-constructor)
  NamedRef(const char* name) : NamedRef(std::string_view(name)) {}  // NOLINT(google-explicit-constructor)
  NamedRef(const T& t) : ptr(&t) {}                                 // NOLINT(google-explicit-constructor)
  [[nodiscard]] const T& operator*() const { return *ptr; }
  [[nodiscard]] const T* operator->() const { return ptr; }
  [[nodiscard]] friend bool operator==(NamedRef l, NamedRef r) { return l.ptr == r.ptr; }
  friend std::ostream& operator<<(std::ostream& os, NamedRef ref) { return os << *ref.ptr; }
};
using FilterRef = NamedRef<Filter>;
using BoundaryRef = NamedRef<Boundary>;
using GammaRef = NamedRef<Gamma>;

// A Vec<T, D> of per-dimension settings, initializable from a single value (broadcast to all dimensions) or from a
// Vec of values convertible to T, e.g., `PerDim<FilterRef, 2> f = "cubic";` or `f = V("cubic", "triangle");`.
template <typename T, int D> struct PerDim : Vec<T, D> {
  template <typename U> requires std::convertible_to<U, T>
  PerDim(const U& u) : Vec<T, D>(ntimes<D>(T(u))) {}  // NOLINT(google-explicit-constructor)
  template <typename U> requires std::convertible_to<U, T>
  PerDim(const Vec<U, D>& v) : PerDim(v, std::make_index_sequence<D>()) {}  // NOLINT(google-explicit-constructor)

 private:
  template <typename U, size_t... Is>
  PerDim(const Vec<U, D>& v, std::index_sequence<Is...>) : Vec<T, D>(T(v[int(Is)])...) {}
};

// How to reconstruct a D-dimensional grid of samples at arbitrary points (sample_domain(), Sampler, resample()).
// The field names and defaults follow the Python package resampler.
template <int D> struct SamplingOptions {
  PerDim<Gridtype, D> gridtype = Gridtype::dual;  // Placement of the samples over the unit domain.
  PerDim<FilterRef, D> filter = "cubic";          // Reconstruction kernel.
  PerDim<BoundaryRef, D> boundary = "auto";       // "auto" is reflect (point sampling never minifies).
  GammaRef gamma = "identity";                    // Transfer function applied to the sample values.
};

// How to resize a D-dimensional grid (scale()); a flat superset of SamplingOptions<D>.
template <int D> struct ResizeOptions {
  PerDim<Gridtype, D> gridtype = Gridtype::dual;  // Placement of the source samples over the unit domain.
  PerDim<FilterRef, D> filter = "cubic";          // Kernel used when magnifying.
  PerDim<FilterRef, D> prefilter = "auto";        // Kernel used when minifying; "auto" is the same as filter.
  PerDim<BoundaryRef, D> boundary = "auto";       // "auto" is reflect when magnifying and clamp when minifying.
  GammaRef gamma = "identity";                    // Transfer function applied to the sample values.
  // NOLINTNEXTLINE(readability-redundant-member-init): for designated initializers.
  std::optional<PerDim<Gridtype, D>> dst_gridtype{};  // Placement of the destination samples; default: as gridtype.
  PerDim<double, D> scale = 1.;                       // Scaling of the source domain onto the destination domain.
  PerDim<double, D> translate = 0.;                   // Offset of the scaled source domain in the destination domain.
  // NOLINTNEXTLINE(readability-redundant-member-init): for designated initializers.
  std::optional<Vec<int, D>> dim_order{};  // Override of the automatically chosen order of dimensions.
  [[nodiscard]] SamplingOptions<D> sampling() const { return {gridtype, filter, boundary, gamma}; }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RESAMPLEOPTIONS_H_
