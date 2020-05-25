// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_TIMER_H_
#define MESH_PROCESSING_LIBHH_TIMER_H_

#include "Hh.h"

#if 0
{
  procedure() {
    HH_TIMER(_proc);  // timing for entire procedure
    if (something) {
      HH_TIMER(__step1);
      step1();
    }  // sub-timings for substeps
    if (1) {
      HH_TIMER(__step2);
      step2();
    }
  }
  {
    HH_TIMER(atimer2);
    statements;
    HH_TIMER_END(atimer2);
    more_statements;
  }
  // getenv_int("SHOW_TIMES")==-1 : all -> noprint
  // getenv_int("SHOW_TIMES")==1  : all but noprint -> normal
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
    explicit Timer(string pname = "", EMode mode = EMode::noprint); // timer is automatically started
    ~Timer()                                    { terminate(); }
    void terminate();           // finish the timer earlier than its end of scope
    void stop();
    void start();
    double real() const;        // timer must be stopped
    double cpu() const;         // timer must be stopped
    //
    static int show_times()                     { return _s_show; }
    static void set_show_times(int val)         { _s_show = val; }
 private:
    string _name;
    EMode _mode;
    bool _started;
    bool _ever_started;
    double _thread_cpu_time;    // thread  user+system time
    double _process_cpu_time;   // process user+system time
    int64_t _real_counter;
    static int _s_show;
    void zero();
};

#if !defined(HH_NO_TIMERS)

#define HH_TIMER_VAR(id) Timer_##id   // variable name used internally
#define HH_TIMER_AUX(id, mode) hh::Timer HH_TIMER_VAR(id)(#id, mode)
#define HH_TIMER(id)            HH_TIMER_AUX(id, hh::Timer::EMode::normal)
#define HH_CTIMER(id, cond)     HH_TIMER_AUX(id, (cond) ? hh::Timer::EMode::normal : hh::Timer::EMode::noprint)
#define HH_DTIMER(id)           HH_TIMER_AUX(id, hh::Timer::EMode::diagnostic)
#define HH_ATIMER(id)           HH_TIMER_AUX(id, hh::Timer::EMode::abbrev)
#define HH_STIMER(id)           HH_TIMER_AUX(id, hh::Timer::EMode::summary)
#define HH_PTIMER(id)           HH_TIMER_AUX(id, hh::Timer::EMode::possibly)
#define HH_TIMER_END(id)        HH_TIMER_VAR(id).terminate() // terminate a HH_TIMER(id) earlier than its scope

#else

#define HH_TIMER(id)            HH_EAT_SEMICOLON
#define HH_CTIMER(id, cond)     HH_EAT_SEMICOLON
#define HH_DTIMER(id)           HH_EAT_SEMICOLON
#define HH_ATIMER(id)           HH_EAT_SEMICOLON
#define HH_STIMER(id)           HH_EAT_SEMICOLON
#define HH_PTIMER(id)           HH_EAT_SEMICOLON
#define HH_TIMER_END(id)        HH_EAT_SEMICOLON

#endif  // !defined(HH_NO_TIMERS)

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_TIMER_H_
