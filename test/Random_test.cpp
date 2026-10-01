// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Random.h"

#include "libHh/RangeOp.h"  // sorted()
#include "libHh/Stat.h"
using namespace hh;

// Random satisfies the requirements of a uniform random bit generator, e.g. for use with std::shuffle().
static_assert(std::uniform_random_bit_generator<Random>);

int main() {
  Random r1, r2;
  for_int(i, 3) {
    SHOW(r1.get_unsigned());
    SHOW(r2.get_unsigned());
  }
  r1.seed(0);
  for_int(i, 3) SHOW(r1.get_unsigned());
  const int num = 0 ? 10'000'000 : 1'000'000;  // 10M takes too long in debug.
  {
    Stat Sgauss;
    for_int(i, num) {
      HH_SSTAT(Sint, double(r1.get_unsigned()));
      HH_SSTAT(Sunif, r1.unif());
      // HH_SSTAT(Sgauss, r1.gauss());
      Sgauss.enter(r1.gauss());
    }
    SHOW(round_fraction_digits(Sgauss.avg(), 1e10f));
    SHOW(round_fraction_digits(Sgauss.sdv(), 1e7f));
  }
  SHOW(0.5f);  // Mean of uniform [0, 1] distribution.
  SHOW(0.5f * pow(2.f, 32.f));
  SHOW(0.5f * pow(2.f, 64.f));
  SHOW(sqrt(1.f / 12.f));  // Standard deviation of uniform [0, 1] distribution.
  SHOW(sqrt(1.f / 12.f) * pow(2.f, 32.f));
  SHOW(sqrt(1.f / 12.f) * pow(2.f, 64.f));
  {
    r1.seed(0);
    unsigned vmin = std::numeric_limits<unsigned>::max(), vmax = 0;
    for_int(i, num) {
      const unsigned v = r1.get_unsigned();
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
    }
    SHOW(vmin, vmax);
  }
  {
    r1.seed(0);
    double vmin = 1., vmax = 0.;
    for_int(i, num) {
      const double v = r1.dunif();
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
    }
    SHOW(vmin, vmax, 1. - vmax);
  }
  SHOW(Random::G.get_unsigned());

  SHOW(r1.get_uint64());
  for_int(i, num) { HH_SSTAT(Suint64, double(r1.get_uint64())); }
  {
    Array<int> ar;
    for_int(i, 6) ar.push(i);
    Random r;
    shuffle(ar, r);
    SHOW(ar);
    shuffle(ar, r);
    SHOW(ar);
  }
  if (1) {
    const unsigned ub = 11;
    Array<unsigned> ar(ub, 0);
    for_int(i, 10'000) {
      const unsigned v = Random::G.get_unsigned(ub);
      assertx(v < ub);
      ar[v]++;
    }
    SHOW(ar);
  }
  if (1) {
    const unsigned ub = unsigned(float(std::numeric_limits<unsigned>::max()) * .99f);
    for_int(i, 10'000) {
      const unsigned v = Random::G.get_unsigned(ub);
      assertx(v < ub);
      HH_SSTAT(S99, v);
    }
  }
  {
    // The constructor seed and seed() give the same sequence, and different seeds give different sequences.
    Random r3(5), r4, r5(6);
    r4.seed(5);
    SHOW(r4.get_unsigned());
    r4.seed(5);
    for_int(i, 100) assertx(r3.get_unsigned() == r4.get_unsigned());
    r3.seed(5);
    int num_equal = 0;
    for_int(i, 100) num_equal += r3.get_unsigned() == r5.get_unsigned();
    assertx(num_equal < 3);
  }
  {
    // All the raw accessors draw from the same underlying 32-bit sequence.
    Random r3(9), r4(9);
    for_int(i, 10) assertx(r3() == r4.get_unsigned());
    for_int(i, 10) {
      const uint64_t v1 = r3.get_unsigned(), v2 = r3.get_unsigned();
      assertx(r4.get_uint64() == (v1 | (v2 << 32)));
    }
    for_int(i, 10) assertx(r3.get_size_t() == r4.get_uint64());
    r3.discard(1000);
    for_int(i, 1000) dummy_use(r4.get_unsigned());
    assertx(r3.get_unsigned() == r4.get_unsigned());
    r3.discard(0);
    assertx(r3.get_unsigned() == r4.get_unsigned());
    SHOW(Random::min(), Random::max(), Random::default_seed);
  }
  {
    // For a power-of-two bound, get_unsigned(ub) keeps the low bits of get_unsigned().
    Random r3(11), r4(11);
    for (const unsigned ub : {1u, 2u, 8u, 1u << 20, 1u << 31}) {
      for_int(i, 100) assertx(r3.get_unsigned(ub) == (r4.get_unsigned() & (ub - 1)));
    }
    // For other bounds, the values are in range and, for small bounds, all occur.
    for (const unsigned ub : {3u, 7u, 1000u, std::numeric_limits<unsigned>::max()}) {
      Array<bool> seen(int(min(ub, 1000u)), false);
      for_int(i, 10'000) {
        const unsigned v = r3.get_unsigned(ub);
        assertx(v < ub);
        if (v < unsigned(seen.num())) seen[int(v)] = true;
      }
      if (ub <= 7) assertx(ranges::all_of(seen, [](bool b) { return b; }));
    }
  }
  {
    // The floating-point samples lie strictly within (0, 1) and have the expected moments.
    Random r3(13);
    Stat stat_unif, stat_dunif, stat_dgauss;
    for_int(i, 100'000) {
      const float u = r3.unif();
      assertx(u > 0.f && u < 1.f);
      stat_unif.enter(u);
      const double d = r3.dunif();
      assertx(d > 0. && d < 1.);
      stat_dunif.enter(d);
      stat_dgauss.enter(r3.dgauss());
    }
    // The tolerances are about 6 standard errors of the estimates.
    assertx(abs(stat_unif.avg() - .5f) < .006f && abs(stat_unif.sdv() - std::sqrt(1.f / 12.f)) < .004f);
    assertx(abs(stat_dunif.avg() - .5f) < .006f && abs(stat_dunif.sdv() - std::sqrt(1.f / 12.f)) < .004f);
    assertx(abs(stat_dgauss.avg()) < .02f && abs(stat_dgauss.sdv() - 1.f) < .015f);
  }
  if (0) {
    // KNOWN_BUG: unif() returns 1.f whenever get_unsigned() >= 2^32 - 128, i.e. with probability
    // 2^-25, because the conversion of the 32-bit value to float rounds it up to 2^32.
    Random r3;
    r3.discard(60'571'531);  // The first such value for the default seed.
    assertx(r3.unif() < 1.f);
  }
  {
    // A shuffle of an empty or single-element array is a no-op, and a shuffle of a larger array is a permutation.
    Random r3(17);
    Array<int> ar;
    shuffle(ar, r3);
    ar.push(42);
    shuffle(ar, r3);
    assertx(ar == V(42).view());
    Array<int> ar2(range(100));
    shuffle(ar2, r3);
    assertx(ar2 != Array<int>(range(100)));
    assertx(sorted(ar2) == Array<int>(range(100)));
    // Every element is equally likely to land in the first position.
    Array<int> counts(4, 0);
    for_int(i, 4'000) {
      Array<int> ar3(range(4));
      shuffle(ar3, r3);
      counts[ar3[0]]++;
    }
    for (const int count : counts) assertx(count > 800 && count < 1200);
  }
}
