// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/ArrayOp.h"
using namespace hh;

int main() {
  {
    const auto check_array = [](const auto& array) {
      SHOW(sort_unique(array));
      SHOW(median_two(array));
      SHOW(median(array));
      for_int(i, array.num()) SHOW(i, rank_element(array, i));
    };
    check_array(Array{4, 3, 2, 2, 5, 4});
    check_array(Array{1, 2, 3, 4});
    check_array(Array{4, 5, 2, 1, 3});
  }
}
