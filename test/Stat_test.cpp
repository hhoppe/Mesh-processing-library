// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Stat.h"

#include "libHh/Array.h"
#include "libHh/Vec.h"
using namespace hh;

// Optionally, run with:    rm -f Stat.Stat_test; (setenv STAT_FILES; Stat_test); cat Stat.Stat_test

int main() {
  {
    Stat s1("", true);
    for (int i : {2, 4, -1, 10, 8}) s1.enter(i);
    SHOW(s1.short_string());
    s1.enter(12);
    s1.enter(11);
    SHOW(s1.short_string());
    s1.add(s1);
    SHOW(s1);
    Stat s2("Stat_test", true);
    s2.enter(1);
    s2.enter(5);
    s2.enter(6);
    SHOW(s2);
    SHOW("end");
  }
  SHOW("before Stot");
  HH_SSTAT(Stot, 0);
  {
    Stat Svar("Svar", true);
    for_int(i, 100) Svar.enter(i);
  }
  {
    HH_STAT(Ssquare);
    for_int(i, 100) Ssquare.enter(i);
  }
  {
    float values[] = {2.f, 4.f, 4.f, 5.f, 4.f};  // test C-array
    SHOW(ArView(values));
    for (float v : values) SHOW(v);
    HH_RSTAT(Svalues, ArView(values));
  }
  {
    const float values[] = {2.f, 4.f, 4.f, 5.f, 4.f};  // test C-array
    for (float v : values) SHOW(v);
    HH_RSTAT(Svalues, values);
  }
  { HH_RSTAT(Svalues2, V(2.f, 4.f, 4.f, 5.f, 4.f)); }
  {
    SHOW(Stat(V(1., 4., 5., 6.)).short_string());
    SHOW(Stat(V(1., 4., 5., 6.)).sdv());
  }
}
