// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Map.h"

#if 0
{
    Combination<Vertex> comb;
    for_combination(comb, [&](Vertex v, float val) { func(v, val); });
}
#endif

namespace hh {

// Represents a weighted combination of elements where the weights are float.
template<typename T> class Combination : public Map<T,float> {
    using base = Map<T,float>;
 public:
    float sum() const                           { float t = 0.f; for (float v : values()) { t += v; } return t; }
    void shrink_to_fit() const { // remove elements with zero weights
        Combination& var_self = const_cast<Combination&>(*this);
        Array<T> ar; for_map_key_value(*this, [&](const T& e, float v) { if (!v) ar.push(e); });
        for (const T& e : ar) { var_self.remove(e); }
    }
    using base::values;
};

template<typename T, typename Func = void(const T& e, float val)>
void for_combination(const Combination<T>& comb, Func func) {
    for (auto& kv : comb) { func(kv.first, kv.second); }
}

} // namespace hh
