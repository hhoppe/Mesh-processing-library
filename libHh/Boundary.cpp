// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Boundary.h"

#include "libHh/Vec.h"

namespace hh {

namespace {

// Map the index i of a sample into the interior of a grid of `size` samples, or return false if it has value cval.
using IndexMap = bool (*)(int& i, int size, Gridtype gridtype);

bool map_reflect(int& i, int size, Gridtype gridtype) {
  return map_boundaryrule_1d(i, size, gridtype == Gridtype::dual ? Bndrule::reflect : Bndrule::reflect_primal);
}

bool map_wrap(int& i, int size, Gridtype gridtype) {
  const int period = gridtype == Gridtype::dual ? size : size - 1;  // The last primal sample repeats the first.
  i = ((i % period) + period) % period;
  return true;
}

bool map_clamp(int& i, int size, Gridtype /*gridtype*/) { return map_boundaryrule_1d(i, size, Bndrule::clamp); }

bool map_border(int& i, int size, Gridtype /*gridtype*/) { return map_boundaryrule_1d(i, size, Bndrule::border); }

// Reflect once across each boundary, then clamp (OpenGL's mirror-clamp-to-edge).
bool map_reflect_clamp(int& i, int size, Gridtype gridtype) {
  i = gridtype == Gridtype::dual ? min(i < 0 ? -1 - i : i, size - 1) : min(abs(i), size - 1);
  return true;
}

// For a reconstruction point at unit-domain coordinate x outside [0, 1], scale the weights of the folded taps
// [first, num) so that the remainder goes to cval, with an antialiased transition of width `spacing` (the distance
// between consecutive reconstruction points; 0 for an isolated point).
void override_exterior(double x, double spacing, ArrayView<float> folded, int first) {
  const double signed_distance = abs(x - .5) - .5;  // Boundaries at 0 and 1.
  const double opacity = spacing > 0. ? clamp(.5 - signed_distance / spacing, 0., 1.) : signed_distance > 0. ? 0. : 1.;
  if (opacity == 1.) return;
  for_intL(j, first, folded.num()) folded[j] = float(folded[j] * opacity);
}

// Give the remainder 1 - sum of the folded taps [first, num) to cval.
void push_remainder(Array<int>& indices, Array<float>& folded, int first) {
  double sum = 0.;
  for_intL(j, first, folded.num()) sum += folded[j];
  const double remainder = 1. - sum;
  if (abs(remainder) < 1e-6) return;
  indices.push(-1);
  folded.push(float(remainder));
}

// A rule for which each exterior tap maps to a single interior sample (or to cval), optionally tiling the
// coordinates and overriding the value outside the unit domain by cval.
struct BoundaryIndexMap final : Boundary {
  explicit BoundaryIndexMap(string name, IndexMap map, bool uses_cval, bool tile = false, bool override = false)
      : Boundary(std::move(name)), _map(map), _tile(tile), _override(override) {
    _uses_cval = uses_cval, _overrides_exterior = override;
  }
  [[nodiscard]] double remap_coordinate(double x) const override { return _tile ? x - std::floor(x) : x; }
  void fold(int index0, CArrayView<float> weights, int size, Gridtype gridtype, double x, double spacing,
            Array<int>& indices, Array<float>& folded) const override {
    ASSERTX(size >= (gridtype == Gridtype::dual ? 1 : 2));
    const int first = indices.num();
    for_int(k, weights.num()) {
      int i = index0 + k;
      if (!_map(i, size, gridtype)) i = -1;  // The tap references cval.
      indices.push(i);
      folded.push(weights[k]);
    }
    if (_override) {
      override_exterior(x, spacing, folded, first);
      push_remainder(indices, folded, first);
    }
  }
  IndexMap _map;
  bool _tile, _override;
};

// Exterior taps are dropped and the interior weights renormalized; points outside the domain get cval.
struct BoundaryNatural final : Boundary {
  BoundaryNatural() : Boundary("natural") { _uses_cval = _overrides_exterior = true; }
  void fold(int index0, CArrayView<float> weights, int size, Gridtype /*gridtype*/, double x, double spacing,
            Array<int>& indices, Array<float>& folded) const override {
    const int first = indices.num();
    double sum = 0.;
    bool dropped = false;
    for_int(k, weights.num()) {
      const int i = index0 + k;
      if (i < 0 || i >= size) {
        dropped = true;
        continue;
      }
      indices.push(i);
      folded.push(weights[k]);
      sum += weights[k];
    }
    // (The weights are renormalized only if some tap was dropped, so that interior rows stay bitwise unchanged.)
    if (dropped && sum != 0.) for_intL(j, first, folded.num()) folded[j] = float(folded[j] / sum);
    override_exterior(x, spacing, folded, first);
    push_remainder(indices, folded, first);
  }
};

// Exterior samples are extrapolated linearly from the two nearest samples (quadratically from the three nearest),
// optionally overriding the value outside the unit domain by cval.
struct BoundaryExtrapolate final : Boundary {
  explicit BoundaryExtrapolate(string name, int degree, bool override)
      : Boundary(std::move(name)), _degree(degree), _override(override) {
    _uses_cval = _overrides_exterior = override;
  }
  void fold(int index0, CArrayView<float> weights, int size, Gridtype gridtype, double x, double spacing,
            Array<int>& indices, Array<float>& folded) const override {
    const int first = indices.num();
    if (size <= _degree) {  // Too few samples: reflect instead.
      for_int(k, weights.num()) {
        int i = index0 + k;
        map_reflect(i, size, gridtype);
        indices.push(i);
        folded.push(weights[k]);
      }
    } else {
      // Contributions to the samples 0, 1, 2 (low) and size - 1, size - 2, size - 3 (high).
      Vec3<double> low(0., 0., 0.), high(0., 0., 0.);
      for_int(k, weights.num()) {
        const int i = index0 + k;
        const double w = weights[k];
        if (i >= 0 && i < size) {
          indices.push(i);
          folded.push(weights[k]);
          continue;
        }
        const double t = i < 0 ? double(i) : double(size - 1 - i);  // The extrapolation parameter, with t < 0.
        Vec3<double>& v = i < 0 ? low : high;
        if (_degree == 1) {
          v[0] += (1. - t) * w, v[1] += t * w;
        } else {  // The quadratic through three samples ("cubic precision" in [Keys 1981]).
          v[0] += ((.5 * t - 1.5) * t + 1.) * w, v[1] += ((-t + 2.) * t) * w, v[2] += ((.5 * t - .5) * t) * w;
        }
      }
      for_int(c, _degree + 1) {
        if (low[c] != 0.) indices.push(c), folded.push(float(low[c]));
        if (high[c] != 0.) indices.push(size - 1 - c), folded.push(float(high[c]));
      }
    }
    if (_override) {
      override_exterior(x, spacing, folded, first);
      push_remainder(indices, folded, first);
    }
  }
  int _degree;
  bool _override;
};

struct BoundaryAuto final : Boundary {
  BoundaryAuto() : Boundary("auto") { _is_auto = true; }
  void fold(int, CArrayView<float>, int, Gridtype, double, double, Array<int>&, Array<float>&) const override {
    assertnever("Boundary 'auto' must be resolved to a concrete rule before use");
  }
};

// Former names (and single letters) that remain accepted by Boundary::get().
constexpr std::pair<std::string_view, std::string_view> k_boundary_aliases[] = {
    {"reflected", "reflect"}, {"r", "reflect"}, {"periodic", "wrap"}, {"w", "wrap"}, {"p", "wrap"},
    {"clamped", "clamp"},     {"c", "clamp"},   {"b", "border"},      {"a", "auto"},
};

}  // namespace

const Boundary& Boundary::get(std::string_view name) {
  assertx(name != "");
  // As in Filter::get(), the singletons are lazily created and intentionally never destroyed.
  static const auto& boundaries = *new Array<const Boundary*>{
      new BoundaryIndexMap("reflect", map_reflect, false),
      new BoundaryIndexMap("wrap", map_wrap, false),
      new BoundaryIndexMap("tile", map_reflect, false, true),
      new BoundaryIndexMap("clamp", map_clamp, false),
      new BoundaryIndexMap("border", map_border, true),
      new BoundaryNatural,
      new BoundaryExtrapolate("linear_constant", 1, true),
      new BoundaryExtrapolate("quadratic_constant", 2, true),
      new BoundaryIndexMap("reflect_clamp", map_reflect_clamp, false),
      new BoundaryIndexMap("constant", map_reflect, true, false, true),
      new BoundaryExtrapolate("linear", 1, false),
      new BoundaryExtrapolate("quadratic", 2, false),
      new BoundaryAuto,
  };
  for (const auto& [alias, canonical] : k_boundary_aliases)
    if (name == alias) name = canonical;
  for (const Boundary* boundary : boundaries)
    if (boundary->name() == name) return *boundary;
  assertnever("Boundary '" + string(name) + "' not recognized");
}

const Boundary& Boundary::get(Bndrule bndrule) {
  switch (bndrule) {
    case Bndrule::reflect: return get("reflect");
    case Bndrule::wrap: return get("wrap");
    case Bndrule::clamp: return get("clamp");
    case Bndrule::border: return get("border");
    case Bndrule::reflect_primal: assertnever("Bndrule::reflect_primal is 'reflect' on a primal grid");
    default: assertnever("");
  }
}

}  // namespace hh
