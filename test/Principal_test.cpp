// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Principal.h"

#include "libHh/FileIO.h"
#include "libHh/RangeOp.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

void test(MatrixView<float> mi) {
  // SHOW(mi);
  subtract_mean(mi);
  // SHOW(mi);
  Matrix<float> mo(mi.xsize(), mi.xsize());
  Array<float> eimag(mi.xsize());
  principal_components(mi, mo, eimag);
  SHOW(round_elements(eimag));
  SHOW(round_elements(mo));
  {
    incr_principal_components(mi, mo, eimag, 10);
    SHOW(round_elements(eimag));
    SHOW(round_elements(mo));
  }
  {
    assertx(em_principal_components(mi, mo, eimag, 10));
    SHOW(round_elements(eimag));
    SHOW(round_elements(mo));
  }
}

void read_data(Matrix<float>& mi) {
  RFile fi("lvl00.greencells_161b_tor.nontrsf.txt");
  string sline;
  for_int(i, 4) assertx(my_getline(fi(), sline));
  const int m = 4096, n = 75;
  mi.init(m, n);
  for_int(j, n) for_int(i, m) assertx(fi() >> mi[i][j]);
  {
    float dummy;
    fi() >> dummy;
    assertx(!fi());
  }
}

void test_inc() {
  Matrix<float> mi;
  read_data(mi);
  int n = mi.xsize();
  const int ne = 8;
  Matrix<float> mo1(n, n);
  Array<float> eimag1(n);
  {
    HH_TIMER("_pc1");
    principal_components(mi, mo1, eimag1);
  }
  Matrix<float> mo2(ne, n);
  Array<float> eimag2(ne);
  {
    HH_TIMER("_pc2");
    incr_principal_components(mi, mo2, eimag2, 2);
  }
  for_int(i, ne) {
    float d = float(dot(mo1[i], mo2[i]));
    showf("%d %14g %14g dot=%14g\n", i, eimag1[i], eimag2[i], d);
  }
  Matrix<float> mdot(ne, ne);
  for_int(i, ne) for_int(j, ne) mdot[i][j] = float(abs(dot(mo1[i], mo2[j])));
  SHOW(mdot);
}

void test_em() {
  Matrix<float> mi;
  read_data(mi);
  subtract_mean(mi);
  int n = mi.xsize();
  // for ne=8,  10-20  iterations are needed (variance=0.983087)
  // for ne=10, only 4 iterations are needed (variance=0.989540)
  const int ne = 8;
  Matrix<float> mo1(n, n);
  Array<float> eimag1(n);
  {
    HH_TIMER("_pc1");
    principal_components(mi, mo1, eimag1);
  }
  float sum = float(mag2(eimag1));
  float sumact = float(mag2(eimag1.head(ne)));
  Matrix<float> mo2(ne, n);
  Array<float> eimag2(ne);
  {
    HH_TIMER("_pc2");
    assertx(em_principal_components(mi, mo2, eimag2, 10));
  }
  float sumest = float(mag2(eimag2));
  showf("sum=%g  sumest=%g (%g)  sumact=%g (%g) (lost=%g)\n",  //
        sum, sumest, sumest / sum, sumact, sumact / sum, sumact / sum - sumest / sum);
  {
    double var = 0.f;
    for_int(y, mi.ysize()) var += mag2(mi[y]);
    SHOW(var / mi.ysize());
  }
  for_int(i, ne) {
    float d = float(dot(mo1[i], mo2[i]));
    showf("%d %14g %14g dot=%14g\n", i, eimag1[i], eimag2[i], d);
  }
  Matrix<float> mdot(ne, ne);
  for_int(i, ne) for_int(j, ne) mdot[i][j] = float(abs(dot(mo1[i], mo2[j])));
  SHOW(mdot);
}

}  // namespace

int main() {
  if (0) {
    Matrix<float> mi = {
        {10.f, 5.f},
        {12.f, 6.f},
        {14.f, 5.f},
        {15.f, 6.f},
    };
    test(mi);
  }
  if (1) {
    Matrix<float> mi = {
        {20.f, 10.f},
        {22.f, 11.f},
        {18.f, 9.f},
        {19.f, 12.f},
    };
    test(mi);
  }
  if (0) {
    Matrix<float> mi = {
        {2.f, 0.f},
        {4.f, 0.f},
        {0.f, 0.f},
        {2.f, 2.f},
    };
    test(mi);
  }
  if (0) test_inc();
  if (0) test_em();
}
