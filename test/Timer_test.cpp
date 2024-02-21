// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Timer.h"
using namespace hh;

int main() {
  HH_TIMER("total");
  Timer timer_firsthalf("firsthalf");
  {
    HH_TIMER("t1");
    my_sleep(.1);
  }
  {
    Timer timer("t2");
    HH_TIMER("t3");
    my_sleep(.1);
    timer.terminate();
    my_sleep(.05);
  }
  { Timer timer; }  // Should not print.
  { Timer timer("should_not_print", Timer::EMode::noprint); }
  int count = getenv_int("TTIMER_COUNT");
  if (!count) count = 1000;
  Timer timer("abbrev");
  for_int(i, count) { HH_ATIMER("oneabbrev"); }
  timer.terminate();
  timer_firsthalf.terminate();
  HH_TIMER("secondhalf");
  {
    Timer t5;
    dummy_use(t5);
  }
  for_int(i, 10) { HH_TIMER("t7"); }
  { HH_DTIMER("t8"); }
}
