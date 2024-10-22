// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_TIMER_H_
#define MESH_PROCESSING_LIBHH_TIMER_H_

#include "libHh/Hh.h"

#if 0
{
  procedure() {
    HH_TIMER("_proc");  // Timing for entire procedure.
    if (something) {
      HH_TIMER("__step1");  // Sub-timings for substeps.
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
    timer.terminate();  // Shows timing up to here.
    more_statements;
  }
  // getenv_int("SHOW_TIMES") == -1 : all -> noprint.
  // getenv_int("SHOW_TIMES") == 1  : all but noprint -> normal.
  Timer::set_show_times(-1);  // Disable printing of all timers.
  // getenv_bool("HH_HIDE_SUMMARIES") : if true, omit summary of timers.
}
#endif

namespace hh {

// Object that tracks elapsed time and process computation (user+system) time over its lifetime.
// It reports effective multithreading factor as a percentage, if detected.
// Timing data associated with multiple Timers with the same name are accumulated and reported at program end.
// (This accumulation is not threadsafe; we assume that any timers are created outside multithreading sections.)
// We find that the computation time of the main thread is not all that useful.  Although in OpenMP, the main
// thread participates in task work, this is not the case with hh::ThreadPoolIndexedTask, so the ratio
// process_cpu_time / main_thread_cpu_time is not meaningful.
class Timer : noncopyable {
 public:
  enum class EMode { normal, diagnostic, abbrev, summary, possibly, noprint, always };
  // normal:          showdf() every time
  // diagnostic:      showf() every time
  // abbrev:          showf() first time
  // summary:         only print in summary
  // possibly:        never print, do not keep stats (except if SHOW_TIMES)
  // noprint:         never print, do not keep stats
  // always:          showdf() every time even if SHOW_TIMES == -1
  //
  // If a name is given in the constructor, the timer is automatically started.
  // Otherwise, mode is overridden to EMode::noprint and the timer is not started.
  explicit Timer(string name_ = "", EMode mode = EMode::normal);
  ~Timer() { terminate(); }
  void terminate();  // finish the timer earlier than its end of scope
  void stop();
  void start();
  // For the following, the timer must be stopped.
  double real() const;         // Elapsed time, in seconds.
  double cpu() const;          // Simulated "single-thread" "user + system" CPU time, in seconds.
  double parallelism() const;  // Effective number of cores that multiply cpu() to obtain the total process CPU time.
  //
  static int show_times() { return _s_show; }
  static void set_show_times(int val) { _s_show = val; }

 private:
  string _name;
  EMode _mode;
  bool _started{false};
  bool _ever_started{false};
  // double _thread_cpu_time{0.}; // Main thread "user + system" time, in seconds.
  double _process_cpu_time{0.};   // Process "user + system" time, in seconds.
  int64_t _real_time_counter{0};  // Elapsed time, often in units of 100 ns.
  static int _s_show;
  friend class Timers;
  void update_counters(int delta_sign);
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
