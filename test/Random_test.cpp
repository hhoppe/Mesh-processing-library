// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Random.h"

#include "libHh/Stat.h"
using namespace hh;

int main() {
  Random r1, r2;
  for_int(i, 3) {
    SHOW(r1.get_unsigned());
    SHOW(r2.get_unsigned());
  }
  r1.seed(0);
  for_int(i, 3) SHOW(r1.get_unsigned());
  const int num = 0 ? 10 * 1000 * 1000 : 1000 * 1000;  // 10M takes too long in debug
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
  SHOW(0.5f);  // mean of uniform [0, 1] distribution
  SHOW(0.5f * pow(2.f, 32.f));
  SHOW(0.5f * pow(2.f, 64.f));
  SHOW(sqrt(1.f / 12.f));  // standard deviation of uniform [0, 1] distribution
  SHOW(sqrt(1.f / 12.f) * pow(2.f, 32.f));
  SHOW(sqrt(1.f / 12.f) * pow(2.f, 64.f));
  {
    r1.seed(0);
    unsigned vmin = INT_MAX, vmax = 0;
    for_int(i, num) {
      unsigned v = r1.get_unsigned();
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
    }
    SHOW(vmin, vmax);
  }
  {
    r1.seed(0);
    double vmin = 1., vmax = 0.;
    for_int(i, num) {
      double v = r1.dunif();
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
    }
    SHOW(vmin, vmax, 1. - vmax);
  }
  SHOW(Random::G.get_unsigned());
  //
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
    for_int(i, 10000) {
      unsigned v = Random::G.get_unsigned(ub);
      assertx(v < ub);
      ar[v]++;
    }
    SHOW(ar);
  }
  if (1) {
    const unsigned ub = unsigned(std::numeric_limits<unsigned>::max() * .99f);
    for_int(i, 10000) {
      unsigned v = Random::G.get_unsigned(ub);
      assertx(v < ub);
      HH_SSTAT(S99, v);
    }
  }
}
