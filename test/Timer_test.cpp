// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Timer.h"
using namespace hh;

int main() {
  HH_TIMER("total");
  Timer timer_firsthalf("firsthalf");
  {
    HH_TIMER("t1");
    my_precise_sleep(0.1);
  }
  {
    Timer timer("t2");
    HH_TIMER("t3");
    my_sleep(0.1);
    timer.terminate();
    my_sleep(0.05);
  }
  {
    Timer timer;  // Should not print.
  }
  {
    Timer timer("should_not_print", Timer::EMode::noprint);
  }
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
  {
    HH_DTIMER("t8");
  }
  {
    HH_CTIMER("t9_conditional", true);
    HH_CTIMER("should_not_print_conditional", false);
    HH_STIMER("t10_summary_only");
    HH_PTIMER("should_not_print_possibly");
    const Timer timer_always("t11_always", Timer::EMode::always);
  }
  {
    // An unnamed timer is not started and never prints, but it may be started and stopped explicitly.
    // The bounds are generous so that the test is robust under heavy machine load.
    Timer timer_explicit;
    timer_explicit.start();
    my_precise_sleep(0.02);
    timer_explicit.stop();
    const double real1 = timer_explicit.real();
    if (real1 < 0.015 || real1 > 30.) assertnever(SSHOW(real1));
    assertx(timer_explicit.cpu() >= 0. && timer_explicit.cpu() <= real1);  // By definition, cpu() is at most real().
    assertx(timer_explicit.parallelism() >= 0.);
    // Restarting the timer accumulates further time.
    timer_explicit.start();
    my_precise_sleep(0.02);
    timer_explicit.stop();
    const double real2 = timer_explicit.real();
    if (real2 < real1 + 0.015 || real2 > real1 + 30.) assertnever(SSHOW(real1, real2));
  }
}
