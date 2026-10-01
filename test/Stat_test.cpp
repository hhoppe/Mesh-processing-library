// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Stat.h"

#include "libHh/Array.h"
#include "libHh/Parallel.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // round_elements()
#include "libHh/Vec.h"
using namespace hh;

namespace {

bool is_near(double a, double b, double tolerance = 1e-6) { return abs(a - b) <= tolerance * max(1., abs(b)); }

}  // namespace

// Optionally, run with:  rm -f Stat.Stat_test; STAT_FILES=1 Stat_test; cat Stat.Stat_test.

int main() {
  {
    Stat s1("", true);
    for (const int i : {2, 4, -1, 10, 8}) s1.enter(i);
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
    float values[] = {2.f, 4.f, 4.f, 5.f, 4.f};  // Test C-array.
    SHOW(CArrayView(values));
    for (float v : values) SHOW(v);
    HH_RSTAT(Svalues, CArrayView(values));
  }
  {
    const float values[] = {2.f, 4.f, 4.f, 5.f, 4.f};  // Test C-array.
    for (float v : values) SHOW(v);
    HH_RSTAT(Svalues, values);
  }
  {
    HH_RSTAT(Svalues2, V(2.f, 4.f, 4.f, 5.f, 4.f));
  }
  {
    SHOW(Stat(V(1., 4., 5., 6.)).short_string());
    SHOW(Stat(V(1., 4., 5., 6.)).sdv());
  }
  {
    // Compare all the accessors against a reference model.
    Random random(1);
    Array<double> values(200);
    for (double& value : values) value = random.dunif() * 20. - 7.;
    double sum = 0., sum2 = 0., vmin = BIGFLOAT, vmax = -BIGFLOAT;
    for (const double value : values) {
      sum += value, sum2 += square(value);
      vmin = min(vmin, double(float(value))), vmax = max(vmax, double(float(value)));
    }
    const int n = values.num();
    const double avg = sum / n, ssd = sum2 - square(sum) / n, var = ssd / (n - 1.);
    const Stat stat(values);
    assertx(stat.num() == n && stat.inum() == n && stat.name() == "");
    assertx(stat.min() == float(vmin) && stat.max() == float(vmax));
    assertx(stat.max_abs() == max(abs(stat.min()), abs(stat.max())));
    assertx(is_near(stat.sum(), sum) && is_near(stat.avg(), avg) && is_near(stat.ssd(), ssd) &&
            is_near(stat.var(), var));
    assertx(is_near(stat.sdv(), std::sqrt(var)) && is_near(stat.rms(), std::sqrt(sum2 / n)));
    assertx(is_near(range_stat(values).avg(), avg));
  }
  {
    // A single element has zero ssd.  (Its var() would warn, as var() requires at least two elements.)
    Stat stat;
    stat.enter(-3.f);
    SHOW(stat.num(), stat.min(), stat.max(), stat.avg(), stat.ssd(), stat.rms(), stat.max_abs());
    SHOW(stat.short_string());
  }
  {
    // Integral arguments are entered as double, so even those beyond the float precision are exact.
    Stat stat;
    stat.enter(16'777'217);
    stat.enter(1u);
    SHOW(stat.short_string());
    assertx(stat.short_string().contains("av=8388609 "));
  }
  {
    // enter_multiple() is equivalent to repeated enter(), and remove() undoes enter() except for min() and max().
    Stat stat1, stat2;
    stat1.enter_multiple(2.5f, 3);
    stat1.enter(1.f);
    for_int(i, 3) stat2.enter(2.5f);
    stat2.enter(1.f);
    assertx(stat1.short_string() == stat2.short_string());
    stat1.enter(100.f);
    stat1.remove(100.f);
    assertx(stat1.num() == 4 && stat1.sum() == stat2.sum() && stat1.var() == stat2.var() && stat1.max() == 100.f);
    stat1.enter_multiple(2.5f, -3);  // A negative factor removes the value.
    SHOW(stat1.num(), stat1.sum());
    stat1.zero();
    SHOW(stat1.short_string());
    assertx(stat1.num() == 0 && stat1.min() == BIGFLOAT && stat1.max() == -BIGFLOAT);
  }
  {
    // add() combines the statistics of two disjoint sets of values.
    const Stat stat2(V(10.f, 20.f)), stat12(V(1.f, 2.f, 3.f, 10.f, 20.f));
    Stat stat(V(1.f, 2.f, 3.f));
    stat.add(stat2);
    assertx(stat.short_string() == stat12.short_string());
    stat.add(Stat{});  // Adding an empty Stat has no effect.
    assertx(stat.short_string() == stat12.short_string());
  }
  {
    // set_rms() reports the root-mean-square in place of the standard deviation.
    Stat stat("Srms");
    stat.set_rms();
    for (const float value : {3.f, -4.f}) stat.enter(value);
    SHOW(stat);
    SHOW(stat.rms(), stat.sdv());
  }
  {
    // Names longer than 27 characters are truncated in the output.
    Stat stat("A_very_long_statistic_name_that_is_truncated");
    stat.enter(1.f);
    stat.set_name("Renamed");
    SHOW(stat);
    stat.set_name("A_very_long_statistic_name_that_is_truncated");
    SHOW(stat);
    assertx(make_string(stat) == stat.name_string());
    stat.set_print(true);  // Printed upon destruction.
  }
  {
    // Move construction, move assignment, and swap transfer the accumulated values and the print flag.
    Stat stat1("Smoved", true);
    stat1.enter(5.f);
    Stat stat2(std::move(stat1));
    SHOW(stat2.name(), stat2.num());
    Stat stat3;
    stat3 = std::move(stat2);
    SHOW(stat3.name(), stat3.num());
    Stat stat4(V(7.f, 8.f));
    swap(stat3, stat4);
    SHOW(stat3.num(), stat4.num(), stat4.name());
  }
  {
    // standardize() and standardize_rms() modify the range in place.
    Array<float> ar{1.f, 2.f, 3.f, 6.f};
    const Array<float> ar_standardized = standardize(clone(ar));
    const Stat stat(ar_standardized);
    assertx(abs(stat.avg()) < 1e-6f && abs(stat.sdv() - 1.f) < 1e-6f);
    assertx(ar[3] == 6.f);  // The original is unmodified.
    standardize_rms(ar);
    assertx(abs(Stat(ar).rms() - 1.f) < 1e-6f);
    SHOW(round_elements(clone(ar_standardized)));
    SHOW(round_elements(clone(ar)));
  }
  {
    // HH_SSTAT accumulates into per-thread Stat objects within a parallel loop; they are folded at program end.
    parallel_for(range(1000), [&](const int i) { HH_SSTAT(Sparallel, i); });
    for_int(i, 4) HH_SSTAT_RMS(Sstatic_rms, float(i));
  }
}
