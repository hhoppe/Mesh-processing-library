// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Combination.h"

#include "libHh/Array.h"
#include "libHh/HashTuple.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

// Returns the (key, weight) pairs of a combination sorted by key, because its iteration order is unspecified.
template <typename T> Array<std::pair<T, float>> sorted_entries(const Combination<T>& comb) {
  Array<std::pair<T, float>> entries;
  for_combination(comb, [&](const T& e, float val) { entries.push({e, val}); });
  assertx(entries.num() == comb.num());
  return sort(std::move(entries));
}

}  // namespace

int main() {
  {
    Combination<int> comb;
    comb[2] = .5f;
    comb[4] = .25f;
    comb[7] = .5f;
    comb[8] = .0f;
    // Like std::unordered_map, operator[] enters an absent key, here with weight zero.
    SHOW(comb[1]);
    SHOW(comb[2]);
    SHOW(comb[3]);
    SHOW(comb[4]);
    SHOW(comb.num());
    SHOW(comb.sum());
    SHOW(sorted_entries(comb));
    comb.shrink_to_fit();  // Removes the zero-weight entries for keys 1, 3, and 8.
    SHOW(comb.num());
    assertx(!comb.contains(1) && !comb.contains(3) && !comb.contains(8));
    assertx(comb.contains(2) && comb.contains(4) && comb.contains(7));
    SHOW(comb.sum());
    SHOW(sorted_entries(comb));
    comb.shrink_to_fit();  // It is idempotent.
    assertx(comb.num() == 3);
  }
  {
    Combination<int> comb;
    SHOW(comb.sum());  // An empty combination has zero sum.
    comb.shrink_to_fit();
    assertx(comb.empty());
    comb[5] = 0.f;
    comb[6] = 0.f;
    comb.shrink_to_fit();  // All weights are zero, so all entries are removed.
    assertx(comb.empty());
  }
  {
    // Weights may be negative (as in an affine combination), and they accumulate through operator[].
    Combination<Vec2<int>> comb;
    comb[V(0, 0)] += .75f;
    comb[V(1, 0)] += .5f;
    comb[V(0, 0)] -= .25f;
    comb[V(0, 1)] = -.5f;
    comb[V(1, 0)] -= .5f;  // The weight becomes exactly zero.
    SHOW(comb.sum());
    comb.shrink_to_fit();
    SHOW(sorted_entries(comb));
    // A weighted combination of positions, accumulated through for_combination().
    Vec2<float> p{};
    for_combination(comb, [&](const Vec2<int>& u, float w) { p += u.cast<float>() * w; });
    SHOW(p);
  }
  {
    // A combination whose keys are move-only (shrink_to_fit() is unavailable because it copies keys).
    Combination<unique_ptr<int>> comb;
    comb.enter(make_unique<int>(3), .5f);
    float sum_keys = 0.f;
    for_combination(comb, [&](const unique_ptr<int>& e, float val) { sum_keys += float(*e) * val; });
    SHOW(sum_keys, comb.sum());
  }
}

template class hh::Combination<unsigned>;
template class hh::Combination<std::tuple<void*, bool>>;
template class hh::Combination<unique_ptr<int>>;
