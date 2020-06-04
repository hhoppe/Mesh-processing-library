// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Combination.h"

#include "libHh/HashTuple.h"
using namespace hh;

int main() {
  {
    Combination<int> comb;
    comb[2] = .5f;
    comb[4] = .25f;
    comb[7] = .5f;
    comb[8] = .0f;
    SHOW(comb[1]);
    SHOW(comb[2]);
    SHOW(comb[3]);
    SHOW(comb[4]);
    SHOW(comb.sum());
    comb.shrink_to_fit();
  }
}

namespace hh {
template class Combination<unsigned>;
template class Combination<std::tuple<void*, bool>>;

using U = unique_ptr<int>;
template <> void Combination<U>::shrink_to_fit() const {}  // definition illegal for U
template class Combination<U>;
}  // namespace hh
