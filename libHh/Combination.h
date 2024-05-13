// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
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

// Represents a weighted combination of elements where the weights are float.
template <typename T> class Combination : public Map<T, float> {
  using base = Map<T, float>;

 public:
  float sum() const { return hh::sum<float>(values()); }
  void shrink_to_fit() const {  // remove elements with zero weights
    Combination& var_self = const_cast<Combination&>(*this);
    Array<T> ar;
    for (auto& [e, v] : *this)
      if (!v) ar.push(e);
    for (const T& e : ar) var_self.remove(e);
  }
  using base::values;
};

template <typename T, typename Func = void(const T& e, float val)>
void for_combination(const Combination<T>& combination, Func func) {
  for (auto& [key, value] : combination) func(key, value);
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_COMBINATION_H_
