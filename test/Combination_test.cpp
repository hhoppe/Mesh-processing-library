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

template class hh::Combination<unsigned>;
template class hh::Combination<std::tuple<void*, bool>>;
template class hh::Combination<unique_ptr<int>>;
