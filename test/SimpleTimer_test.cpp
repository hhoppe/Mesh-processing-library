// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/SimpleTimer.h"

#include "libHh/Hh.h"
using namespace hh;

// The upper bounds on elapsed times are generous so that the test is robust under heavy machine load.

int main() {
  {
    double time_elapsed = 0.;
    for (int i = 0; i < 10; i++) {
      const SimpleTimer timer;
      my_sleep(0.03);
      time_elapsed += timer.elapsed();
    }
    if (time_elapsed < 0.24 || time_elapsed > 20.) assertnever(sform("time_elapsed=%g is out of range", time_elapsed));
  }
  {
    // Successive calls to elapsed() are nonnegative and nondecreasing.
    const SimpleTimer timer;
    double previous = 0.;
    for_int(i, 1000) {
      const double elapsed = timer.elapsed();
      assertx(elapsed >= previous);
      previous = elapsed;
    }
  }
  {
    // The elapsed time is at least (nearly) the precise sleep duration, and at most the interval measured by
    // get_precise_time() around the timer's lifetime (both use the same monotonic clock).
    const double time0 = get_precise_time();
    const SimpleTimer timer;
    my_precise_sleep(0.02);
    const double elapsed = timer.elapsed();
    const double time1 = get_precise_time();
    if (elapsed < 0.015 || elapsed > time1 - time0 + 1e-6 || elapsed > 10.)
      assertnever(sform("elapsed=%g interval=%g", elapsed, time1 - time0));
  }
}
