// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_TIMER_H_
#define MESH_PROCESSING_LIBHH_TIMER_H_

#include "libHh/Hh.h"

#if 0
{
  procedure() {
    HH_TIMER("_proc");  // timing for entire procedure
    if (something) {
      HH_TIMER("__step1");  // sub-timings for substeps
      step1();
    }
    if (1) {
      HH_TIMER("__step2");
      step2();
    }
  }
  {
    Timer timer("atimer2");
    statements;
    timer.terminate();  // shows timing up to here.
    more_statements;
  }
  // getenv_int("SHOW_TIMES") == -1 : all -> noprint
  // getenv_int("SHOW_TIMES") == 1  : all but noprint -> normal
  Timer::set_show_times(-1);  // disable printing of all timers
}
#endif

namespace hh {

// Object that tracks elapsed time, per-thread computation, and per-process computation over its lifetime.
// It reports effective multithreading factor (for parallelism defined inside its scope, not outside).
// Timing data associated with multiple Timers with the same name are accumulated and reported at program end.
//  (This accumulation is not thread-safe; I assume that the timers are created outside multi-threading sections.)
class Timer : noncopyable {
 public:
  enum class EMode { normal, diagnostic, abbrev, summary, possibly, noprint };
  // normal:          showdf() every time
  // diagnostic:      showf() every time
  // abbrev:          showf() first time
  // summary:         only print in summary
  // possibly:        never print, do not keep stats (except if SHOW_TIMES)
  // noprint:         never print, do not keep stats
  //
  // If a name is given in the constructor, the timer is automatically started.
  // Otherwise, mode is overriden to EMode::noprint and the timer is not started.
  explicit Timer(string pname = "", EMode mode = EMode::normal);
  ~Timer() { terminate(); }
  void terminate();  // finish the timer earlier than its end of scope
  void stop();
  void start();
  double real() const;  // timer must be stopped
  double cpu() const;   // timer must be stopped
  //
  static int show_times() { return _s_show; }
  static void set_show_times(int val) { _s_show = val; }

 private:
  string _name;
  EMode _mode;
  bool _started;
  bool _ever_started;
  double _thread_cpu_time;   // thread  user + system time
  double _process_cpu_time;  // process user + system time
  int64_t _real_counter;
  static int _s_show;
  friend class Timers;
  void zero();
};

#define HH_TIMER_AUX(name, mode) hh::Timer HH_UNIQUE_ID(timer)(name, mode)
#define HH_TIMER(name) HH_TIMER_AUX(name, hh::Timer::EMode::normal)
#define HH_CTIMER(name, cond) HH_TIMER_AUX(name, (cond) ? hh::Timer::EMode::normal : hh::Timer::EMode::noprint)
#define HH_DTIMER(name) HH_TIMER_AUX(name, hh::Timer::EMode::diagnostic)
#define HH_ATIMER(name) HH_TIMER_AUX(name, hh::Timer::EMode::abbrev)
#define HH_STIMER(name) HH_TIMER_AUX(name, hh::Timer::EMode::summary)
#define HH_PTIMER(name) HH_TIMER_AUX(name, hh::Timer::EMode::possibly)

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_TIMER_H_
