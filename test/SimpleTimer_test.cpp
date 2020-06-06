// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/SimpleTimer.h"
using namespace hh;

namespace {

volatile int external_value = 3;  // volatile to prevent compiler optimizations

void some_work() {
  const int n = 10'000'000;
  int value = 13;
  for (int i = 0; i < n; i++) {
    value = value / 2 + external_value;
    if (value > 100) exit(1);
  }
}

}

int main() {
  double time_elapsed = 0.;
  for (int i = 0; i < 10; i++) {
    SimpleTimer timer;
    some_work();
    time_elapsed += timer.elapsed();
  }
  if (time_elapsed < 0.001 || time_elapsed > 2.0) {
    fprintf(stderr, "time_elapsed=%g is out of range\n", time_elapsed);
    exit(1);
  }
}
