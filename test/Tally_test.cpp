// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Tally.h"

#include <list>
#include <vector>

#include "libHh/Random.h"
using namespace hh;

namespace {

// Returns a z-score measuring how far the tally of values in [0, m - 1] departs from a uniform distribution.
//
// Pearson's statistic is X2 = sum_j (O_j - E)^2 / E, where O_j is the count of value j and E = n / m is the count
// expected from n uniform draws.  The counts are multinomial, so X2 has exactly mean m - 1 and variance
// 2 (m - 1) (1 - 1 / n), and for E >= 5 it approximately follows the chi-square distribution with nu = m - 1
// degrees of freedom.  That distribution is skewed when nu is small, so rather than comparing X2 with
// nu +- c * sqrt(2 nu), apply the Wilson-Hilferty transform: (X2 / nu)^(1/3) is approximately normal with
// mean 1 - 2 / (9 nu) and variance 2 / (9 nu).  Standardizing it gives z, approximately N(0, 1) for any m and n.
//
// A large positive z means the counts are too uneven; a large negative z means they are too even to be random
// (for example, j = seed % m gives X2 near 0).
double chi_square_uniformity_z(const Tally& tally, int m) {
  assertx(tally.num() <= m);
  const double n = double(tally.total());
  const double expected = n / m;
  assertx(expected >= 5.);
  double x2 = 0.;
  for_int(j, m) x2 += square(tally[j] - expected) / expected;
  const double nu = m - 1.;
  const double variance = 2. / (9. * nu);
  return (std::cbrt(x2 / nu) - (1. - variance)) / sqrt(variance);
}

}  // namespace

int main() {
  {
    const Array<int> values{2, 4, 9, 2, 4};
    const Tally tally(values);
    tally.show();
    SHOW(tally.num(), tally.total());
    SHOW(tally[2], tally[3], tally[9], tally[10]);
    const Stat stat = tally.stat();
    SHOW(stat.num(), stat.min(), stat.max(), stat.avg());
    // A tally of the counts gives the number of values that occur 0, 1, or 2 times.
    Tally(Array<int>{0, 0, 2, 0, 2, 0, 0, 0, 0, 1}).show();
  }
  {
    const Tally tally(views::iota(0, 4));
    SHOW(tally.num(), tally.total());
  }
  {
    const Tally tally(Array<int>{});
    SHOW(tally.num(), tally.total(), tally[0]);
    assertx(tally.stat().num() == 0);
  }
  {
    const Tally tally(Array<int>{0});
    SHOW(tally.num(), tally.total(), tally[0], tally[1]);
  }
  {
    // Ranges of other integral types, a range that is not random-access, and a view that is not const-iterable.
    const Tally tally1(std::vector<uchar>{3, 3, 255});
    assertx(tally1.num() == 256 && tally1[3] == 2 && tally1[255] == 1 && tally1.total() == 3);
    const std::list<short> list{1, 0, 1};
    const Tally tally2(list);
    assertx(tally2.num() == 2 && tally2[0] == 1 && tally2[1] == 2);
    const Tally tally3(range(10) | views::filter([](int i) { return i % 3 == 0; }));
    SHOW(tally3.num(), tally3.total(), tally3[6], tally3[7]);
  }
  {
    // Compare against a reference model, and compare stat() against the Stat of the values themselves.
    Random random(7);
    const int m = 50;
    Array<int> values(1000), counts(m, 0);
    for (int& value : values) {
      value = int(random.get_unsigned(m));
      counts[value]++;
    }
    const Tally tally(values);
    assertx(tally.num() == max(values) + 1 && tally.total() == values.num());
    for_int(value, m + 5) assertx(tally[value] == (value < m ? counts[value] : 0));
    const Stat stat1 = tally.stat(), stat2(values);
    assertx(stat1.num() == stat2.num() && stat1.min() == stat2.min() && stat1.max() == stat2.max());
    assertx(abs(stat1.avg() - stat2.avg()) < 1e-4f && abs(stat1.sdv() - stat2.sdv()) < 1e-4f);
  }
  {
    // The first value drawn after each of n successive seeds is uniformly distributed over m values.
    const int m = 700, n = 100'000;
    Random random;
    Array<int> values(n);
    for_int(seed, n) {
      random.seed(seed);
      values[seed] = random.get_unsigned(m);
    }
    const Tally tally(values);
    assertx(tally.total() == n);
    // For a truly uniform distribution, |z| >= 6 has probability about 2e-9.
    assertx(abs(chi_square_uniformity_z(tally, m)) < 6.);
  }
}
