// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/SimpleTimer.h"

#include "libHh/Hh.h"
using namespace hh;

int main() {
  double time_elapsed = 0.;
  for (int i = 0; i < 10; i++) {
    SimpleTimer timer;
    my_sleep(0.05);
    time_elapsed += timer.elapsed();
  }
  if (time_elapsed < 0.4 || time_elapsed > 5.0) assertnever(sform("time_elapsed=%g is out of range", time_elapsed));
}
