// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Timer.h"
using namespace hh;

int main() {
    HH_TIMER(total);
    HH_TIMER(firsthalf);
    { HH_TIMER(t1); my_sleep(.1); }
    { HH_TIMER(t2); HH_TIMER(t3); my_sleep(.1); HH_TIMER_END(t2); my_sleep(.05); }
    { Timer timer("should_not_print"); }
    int count = getenv_int("TTIMER_COUNT");
    if (!count) count = 1000;
    HH_TIMER(abbrev);
    for_int(i, count) {
        HH_ATIMER(oneabbrev);
    }
    HH_TIMER_END(abbrev);
    HH_TIMER_END(firsthalf);
    HH_TIMER(secondhalf);
    { Timer t5; dummy_use(t5); }
    for_int(i, 10) {
        HH_TIMER(t7);
    }
    { HH_DTIMER(t8); }
}
