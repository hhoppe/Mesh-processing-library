// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SIMPLETIMER_H_
#define MESH_PROCESSING_LIBHH_SIMPLETIMER_H_

#include <cstdio>  // for fprintf() standalone pretty formatting
#include <string>

// Simple precise timer; standalone for portability.

#if 0
{
  procedure() {
    HH_SIMPLE_TIMER(_proc);  // timing for entire procedure
    if (something) {
      HH_SIMPLE_TIMER(__step1);  // sub-timings for substeps
      step1();
    }
    if (1) {
      HH_SIMPLE_TIMER(__step2);
      step2();
    }
    double tot_step2 = 0.;
    for (;;) {
      step1();
      {
        SimpleTimer timer;
        step2();
        tot_step2 += timer.elapsed();
      }
    }
    std::cout << tot_step2 << "\n";
  }
}
#endif

namespace hh {

#define HH_SIMPLE_TIMER(id) hh::SimpleTimer simple_timer_##id(#id)

class SimpleTimer {
 public:
  explicit SimpleTimer(std::string name = "") : _name(std::move(name)) { _real_counter = get_precise_counter(); }
  ~SimpleTimer();
  double elapsed() const { return (get_precise_counter() - _real_counter) * get_seconds_per_counter(); }

 private:
  std::string _name;
  int64_t _real_counter;
  static int64_t get_precise_counter();
  static double get_seconds_per_counter();
};

}  // namespace hh

//----------------------------------------------------------------------------

#if 0 && _MSC_VER >= 1900
#define HH_USE_HIGH_RESOLUTION_CLOCK  // C++11 (but slightly slower)
#endif

#if defined(HH_USE_HIGH_RESOLUTION_CLOCK)
#include <chrono>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOGDI         // avoid name collision on symbol Polygon
#include <Windows.h>  // winbase.h: LARGE_INTEGER, QueryPerformanceCounter, QueryPerformanceFrequency
#else
#include <time.h>  // clock_gettime()
#endif

namespace hh {

inline int64_t SimpleTimer::get_precise_counter() {
#if defined(HH_USE_HIGH_RESOLUTION_CLOCK)
  return possible_cast<int64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
#elif defined(_WIN32)
  LARGE_INTEGER l;
  QueryPerformanceCounter(&l);
  return l.QuadPart;
#else
  struct timespec ti;
  assertx(!clock_gettime(CLOCK_MONOTONIC, &ti));
  return int64_t{ti.tv_sec} * (1000 * 1000 * 1000) + ti.tv_nsec;
#endif
}

inline double SimpleTimer::get_seconds_per_counter() {
#if defined(HH_USE_HIGH_RESOLUTION_CLOCK)
  using Clock = std::chrono::high_resolution_clock;
  constexpr double v = 1. / std::chrono::duration_cast<Clock::duration>(std::chrono::duration<Clock::rep>{1}).count();
  return v;
#elif defined(_WIN32)
  static double v = 0.;
  if (!v) {
    LARGE_INTEGER l;
    QueryPerformanceFrequency(&l);
    v = 1. / static_cast<double>(l.QuadPart);
  }
  return v;
#else
  return 1e-9;
#endif
}

inline SimpleTimer::~SimpleTimer() {
  if (!_name.empty()) {
    fprintf(stderr, " (%-20.20s %8.2f)\n", _name.c_str(), elapsed());
    fflush(stderr);
  }
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SIMPLETIMER_H_
