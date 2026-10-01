// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Advanced.h"

#include "libHh/Array.h"
using namespace hh;

namespace {

constexpr int sum_of_squares() {
  int sum = 0;
  unroll<5>([&](int i) { sum += i * i; });
  return sum;
}

}  // namespace

int main() {
  {
    Array<int> ar;
    unroll<4>([&](int i) { ar.push(i); });
    SHOW(ar);
    unroll<0>([&](int) { assertnever(""); });
    static_assert(sum_of_squares() == 0 + 1 + 4 + 9 + 16);  // Usable at compile time.
  }
  {
    // Both the unrolled path (n <= nmax) and the loop path (n > nmax) visit the indices in order.
    Array<int> ar1, ar2;
    unroll_max<6, 8>([&](int i) { ar1.push(i); });
    unroll_max<6, 3>([&](int i) { ar2.push(i); });
    assertx(ar1 == ar2 && ar1.num() == 6 && ar1[5] == 5);
  }
  {
    // The hash values are implementation-specific, so only their properties are checked.
    assertx(my_hash(42) == std::hash<int>()(42));
    const size_t h12 = hash_combine(hash_combine(size_t{0}, 1), 2);
    const size_t h21 = hash_combine(hash_combine(size_t{0}, 2), 1);
    assertx(h12 == hash_combine(hash_combine(size_t{0}, 1), 2));  // Deterministic.
    assertx(h12 != h21);                                          // Order-dependent.
    assertx(hash_combine(size_t{0}, 1) != hash_combine(size_t{1}, 1));
    // Different short sequences give distinct hashes.
    Array<size_t> hashes;
    for_int(i, 30) for_int(j, 30) hashes.push(hash_combine(hash_combine(size_t{7}, i), j));
    std::sort(hashes.begin(), hashes.end());
    assertx(std::adjacent_find(hashes.begin(), hashes.end()) == hashes.end());
  }
}
