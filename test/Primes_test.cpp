// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Primes.h"

#include <numeric>  // gcd()

#include "libHh/Random.h"
using namespace hh;

int main() {
  {
    // Compare is_prime() with a sieve of Eratosthenes.
    const int n = 100'000;
    Array<bool> sieve(n, true);
    sieve[0] = sieve[1] = false;
    for (int i = 2; i * i < n; i++)
      if (sieve[i])
        for (int j = i * i; j < n; j += i) sieve[j] = false;
    int num_primes = 0;
    for_intL(i, 1, n) {
      assertx(is_prime(i) == sieve[i]);
      num_primes += sieve[i];
    }
    SHOW(num_primes);
    string factors;
    for_intL(i, 2, 30) factors += sform(" %d:%d", i, smallest_factor_gt1(i));
    SHOW(factors);
  }
  {
    // The square root bound in smallest_factor_gt1() remains exact for large values.
    assertx(is_prime(2'147'483'647));     // The Mersenne prime 2^31 - 1.
    assertx(!is_prime(46'337 * 46'337));  // The square of the largest prime whose square fits in an int.
    assertx(smallest_factor_gt1(46'337 * 46'337) == 46'337);
    assertx(smallest_factor_gt1(46'327 * 46'337) == 46'327);  // The product of the two largest such primes.
  }
  {
    SHOW(next_prime(0), next_prime(1), next_prime(2), next_prime(13), next_prime(14), next_prime(1000));
    SHOW(next_prime(10, 5));
    SHOW(prev_prime(3), prev_prime(14), prev_prime(1000), prev_prime(100, 5));
    assertx(next_prime(7, 0) == 7 && prev_prime(7, 0) == 7);
    for_intL(i, 3, 2000) assertx(prev_prime(next_prime(i)) < i || is_prime(i));
  }
  {
    for_intL(i1, 1, 60) for_intL(i2, 1, 60) assertx(are_coprime(i1, i2) == (std::gcd(i1, i2) == 1));
    SHOW(are_coprime(8, 15), are_coprime(12, 18), are_coprime(1, 1), are_coprime(17, 34));
  }
  {
    Random random(1);
    for (const int n : {3, 10, 100, 1000}) {
      for_int(i, 200) {
        const int p = random_prime_under(n, random);
        assertx(p < n && is_prime(p));
      }
    }
  }
}
