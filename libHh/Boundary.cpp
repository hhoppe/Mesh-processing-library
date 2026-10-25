// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Boundary.h"

namespace hh {

namespace {

// Map the index i of a sample into the interior of a grid of `size` samples, or return false if it has value cval.
using IndexMap = bool (*)(int& i, int size, Gridtype gridtype);

// A rule for which each exterior tap maps to a single interior sample (or to cval).
struct BoundaryIndexMap final : Boundary {
  explicit BoundaryIndexMap(string name, IndexMap map, bool uses_cval) : Boundary(std::move(name)), _map(map) {
    _uses_cval = uses_cval;
  }
  void fold(int index0, CArrayView<float> weights, int size, Gridtype gridtype, double /*x*/, Array<int>& indices,
            Array<float>& folded) const override {
    ASSERTX(size >= (gridtype == Gridtype::dual ? 1 : 2));
    for_int(k, weights.num()) {
      int i = index0 + k;
      if (!_map(i, size, gridtype)) continue;
      indices.push(i);
      folded.push(weights[k]);
    }
  }
  IndexMap _map;
};

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

struct BoundaryAuto final : Boundary {
  BoundaryAuto() : Boundary("auto") { _is_auto = true; }
  void fold(int, CArrayView<float>, int, Gridtype, double, Array<int>&, Array<float>&) const override {
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
      new BoundaryIndexMap("clamp", map_clamp, false),
      new BoundaryIndexMap("border", map_border, true),
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
