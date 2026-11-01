// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_COMBINATION_H_
#define MESH_PROCESSING_LIBHH_COMBINATION_H_

#include "libHh/Map.h"
#include "libHh/RangeOp.h"

#if 0
{
  Combination<Vertex> combination;
  for_combination(combination, [&](Vertex v, float val) { func(v, val); });
}
#endif

namespace hh {

// A weighted combination of a few elements (e.g., the vertices that define a subdivided vertex) with float weights,
// stored without heap allocation for up to 7 elements.
template <typename T> class Combination : public InlinedMap<T, float, 7> {
  using base = InlinedMap<T, float, 7>;

 public:
  [[nodiscard]] float sum() const { return hh::sum<float>(values()); }
  void shrink_to_fit() const requires Copyable<T> {  // Remove elements with zero weights.
    Combination& var_self = const_cast<Combination&>(*this);
    Array<T> ar;
    for (auto& [e, v] : *this)
      if (!v) ar.push(e);
    for (const T& e : ar) var_self.remove(e);
  }
  using base::values;
};

template <typename T, std::invocable<const T&, float> Func>
void for_combination(const Combination<T>& combination, Func func) {
  for (auto& [key, value] : combination) func(key, value);
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_COMBINATION_H_
