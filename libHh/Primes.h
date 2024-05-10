// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PRIMES_H_
#define MESH_PROCESSING_LIBHH_PRIMES_H_

#include "libHh/Array.h"
#include "libHh/BinarySearch.h"
#include "libHh/Random.h"

namespace hh {

inline int smallest_factor_gt1(int i) {
  assertx(i > 1);
  int isqr = int(sqrt(float(i)));
  for_intL(j, 2, isqr + 1) {
    if (i % j == 0) return j;
  }
  return i;
}

inline bool is_prime(int i) {
  assertx(i > 0);
  if (i == 1) return false;  // according to Mathematica
  return smallest_factor_gt1(i) == i;
}

inline int next_prime(int i, int count = 1) {
  assertx(i >= 0 && count >= 0);
  for_int(j, count) {
    i++;
    while (!is_prime(i)) i++;
  }
  return i;
}

inline int prev_prime(int i, int count = 1) {
  assertx(i >= 0 && count >= 0);
  for_int(j, count) {
    --i;
    assertx(i > 1);
    while (!is_prime(i)) --i;
  }
  return i;
}

inline int random_prime_under(int n, Random& random) {
  assertx(n < 100'000);
  static Array<int> primes;  // not threadsafe
  if (!primes.num()) primes.push(2);
  while (primes.last() < n) primes.push(next_prime(primes.last()));
  int i = discrete_binary_search(primes, 0, primes.num() - 1, n - 1);
  return primes[random.get_unsigned(i + 1)];
}

inline bool are_coprime(int i1, int i2) {
  assertx(i1 > 0 && i2 > 0);
  // very inefficient
  int m = min(i1, i2);
  for_intL(i, 2, m + 1) {
    if (i1 % i == 0 && i2 % i == 0) return false;
  }
  return true;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PRIMES_H_
