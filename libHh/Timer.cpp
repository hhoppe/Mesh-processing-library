// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Timer.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>               // GetThreadTimes(), GetProcessTimes(), FILETIME
HH_REFERENCE_LIB("advapi32.lib");  // RegOpenKeyExA()
#else
#include <time.h>  // clock_gettime()
#endif             // defined(_WIN32)

#include <array>
#include <cctype>  // isdigit()
#include <mutex>   // once_flag, call_once()
#include <thread>  // thread::hardware_concurrency()
#include <unordered_map>
#include <vector>

#if !defined(HH_NO_TIMERS_CLASS)
#include "libHh/Stat.h"
#endif

// Return the effective number of cores (or 0 if unknown).
inline unsigned std_thread_hardware_concurrency() { return std::thread::hardware_concurrency(); }

// Overhead for use of Timer:
// TTIMER_COUNT=1000000 Timer_test  # Look at oneabbrev and abbrev.
//  on mips R?3000/20Mhz Ultrix:
//           timer off by +82e-6  sec, overhead is 168e-6 sec
//  on sgi R4400/150Mhz Irix 5.2:
//           timer off by +39e-6  sec, overhead is  81e-6 sec
//  on P5/90 NT3.51:
//           timer off by +17e-6  sec, overhead is  39e-6 sec
//  on Pentium3/1GHz WinXP:
//           timer off by +1.0e-6 sec, overhead is 2.3e-6 sec
//  on Pentium4/3.06GHz WinXP:
//           timer off by +0.7e-6 sec, overhead is 1.5e-6 sec
//  on Core i7 3.2Gz Win7 (but debug!) (and now with GetProcessTimes()):
//           timer off by +0.5e-6 sec, overhead is 4.4e-6 sec
//  on AMD 2024-05-5 (debug):
//           timer off by +0.4e-6 sec, overhead is 1.4e-6 sec

namespace hh {

#if !defined(HH_NO_TIMERS_CLASS)

namespace {

#if defined(_WIN32)
// Use of QueryProcessCycleTime() should provide more precise CPU process times, but it comes with two drawbacks:
// (1) It involves approximating the (dynamic) CPU frequency using a static non-portable registry value, and
// (2) the call to QueryProcessCycleTime() surprisingly seems to have larger overhead than GetProcessTimes().
constexpr bool k_win32_use_process_cycle_time = false;
#endif

#if defined(_WIN32)
int cpu_mhz_speed() {
  // There is concern that CPU clock frequency can change dynamically.
  static int v = 0;
  static std::once_flag flag;
  const auto initialize_speed = [] {
    HKEY hkey;
    const char* subkey = R"(HARDWARE\DESCRIPTION\System\CentralProcessor\0)";
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, subkey, 0, KEY_QUERY_VALUE, &hkey)) return;
    int val;
    DWORD tdword = REG_DWORD, len = 4;
    if (RegQueryValueExA(hkey, "~MHz", nullptr, &tdword, reinterpret_cast<uchar*>(&val), &len)) return;
    assertx(len == 4 && val > 0);
    v = val;
  };
  std::call_once(flag, initialize_speed);
  return v;
}
#endif

string timing_host() {
  string rev;
#if defined(_WIN32)
  if (1) {
    HKEY hkey;
    const char* subkey = R"(HARDWARE\DESCRIPTION\System\CentralProcessor\0)";
    if (assertw(!RegOpenKeyExA(HKEY_LOCAL_MACHINE, subkey, 0, KEY_QUERY_VALUE, &hkey))) {
      if (rev == "") {
        int val;
        DWORD tdword = REG_DWORD, len = 4;
        if (!RegQueryValueExA(hkey, "~MHz", nullptr, &tdword, reinterpret_cast<uchar*>(&val), &len)) {
          assertx(len == 4 && val > 0);
          rev = sform("%dMHz", val);
        }
      }
      if (rev == "") {
        std::array<char, 200> buf{};
        DWORD len = DWORD(buf.size() - 2);
        if (!RegQueryValueExA(hkey, "ProcessorNameString", nullptr, nullptr, reinterpret_cast<uchar*>(buf.data()),
                              &len)) {
          const char* p = buf.data();     // e.g., "                   Intel(R) Xeon(TM) CPU 3.06GHz"
          while (*p == ' ') p++;          // skip initial whitespace (optional)
          while (!std::isdigit(*p)) p++;  // go right to CPU speed
          rev = p;
          for (auto& ch : rev)
            if (ch == ' ') ch = '_';
        }
      }
      assertx(!RegCloseKey(hkey));
    }
  }
  if (rev.size() < 3) rev = "";
#endif  // defined(_WIN32)
  const char* s1;
  const char* s2;
  if (rev == "" && (s1 = getenv("PROCESSOR_REVISION"), s1)) rev = sform("R%s", s1);
  string cpu;
  if ((s1 = getenv("PROCESSOR_ARCHITECTURE"), s1) && (s2 = getenv("PROCESSOR_LEVEL"), s2))
    cpu = sform("%s-L%s-%s", s1, s2, rev.c_str());
  if (cpu == "") cpu = getenv_string("CPU");
  if (cpu == "") cpu = getenv_string("PROCESSOR_IDENTIFIER");
  if (cpu == "") cpu = "?";
  const int ncores = std_thread_hardware_concurrency();
  if (ncores) cpu += sform("(%d)", ncores);
  const string host = get_hostname();
  return "cpu=" + cpu + " host=" + host;
}

string create_parallelism_string(double process_time, double real_time, int64_t num_calls) {
  string sparallel = "       ";  // e.g., "  x23.6".
  if (process_time && real_time) {
    const bool meaningful = process_time / num_calls > .02;  // CPU times are often quantized to 16 milliseconds.
    const int ncores = std_thread_hardware_concurrency();
    const double parallelism = min(process_time / real_time, double(ncores));
    if (meaningful && parallelism > 1.1) sparallel = sform("  x%4.1f", parallelism);
  }
  return sparallel;
}

}  // namespace

class Timers {
 public:
  static bool record(const Timer& timer, Timer::EMode cmode) {
    // Note: not threadsafe.  (Any timers must be created outside multithreading sections.)
    const auto& [it, is_new] =
        instance()._map.emplace(timer._name, narrow_cast<int>(instance()._vec_timer_info.size()));
    const auto& [_, i] = *it;
    if (is_new) instance()._vec_timer_info.emplace_back(Timers::TimerInfo(timer._name));
    Timers::TimerInfo& timer_info = instance()._vec_timer_info[i];
    timer_info.stat_cpu_times.enter(timer.cpu());
    timer_info.sum_process_times += timer._process_cpu_time;
    timer_info.sum_real_times += timer.real();
    if (cmode == Timer::EMode::abbrev && timer_info.stat_cpu_times.num() > 1) return true;
    if (cmode == Timer::EMode::summary) {
      instance()._have_some_mult = true;
      return true;
    }
    static std::once_flag flag;
    std::call_once(flag, [] { showff("(Timing on %s)\n", timing_host().c_str()); });
    return false;
  }
  static void flush() { instance().flush_internal(); }

 private:
  static Timers& instance() {
    static Timers& stats = *new Timers;
    return stats;
  }
  Timers() { hh_at_clean_up(Timers::flush); }
  ~Timers() = delete;
  void flush_internal() {
    if (_vec_timer_info.empty()) return;
    for (const auto& timer_info : _vec_timer_info)
      if (timer_info.stat_cpu_times.num() > 1) _have_some_mult = true;
    if (_have_some_mult || Timer::show_times() > 0) {
      const auto show_local = getenv_bool("HH_HIDE_SUMMARIES") ? showff : showdf;
      show_local("Summary of timers (%s):\n", timing_host().c_str());
      const int precision = getenv_int("HH_TIMER_PRECISION", 2, false);  // Number of fractional decimal digits.
      for (const auto& timer_info : _vec_timer_info) {
        const Stat& stat = timer_info.stat_cpu_times;
        const double sum_process_time = timer_info.sum_process_times;
        const double sum_cpu_time = stat.sum();
        const double sum_real_time = timer_info.sum_real_times;
        const string sparallel = create_parallelism_string(sum_process_time, sum_real_time, stat.num());
        const int64_t n = stat.num();
        const long long ln = n;  // because "long long" may be incompatible with int64_t in __CYGWIN__ LP64
        const string smin = n > 1 ? sform("%8.*f", precision, stat.min()).c_str() : "        ";
        const string smax = n > 1 ? sform("%-8.*f", precision, stat.max()).c_str() : "        ";
        show_local(" %-20.20s(%-6lld)%s:%s av=%9.*f   sum=%9.*f%s %9.*f\n",  //
                   sform("%.19s:", stat.name().c_str()).c_str(), ln, smin.c_str(), smax.c_str(), precision, stat.avg(),
                   precision, sum_cpu_time, sparallel.c_str(), precision, sum_real_time);
        if (stat.num() > 1000 && stat.num() * 1e-6 > sum_process_time)
          showdf("**Timer '%s' created more overhead than measured!\n", stat.name().c_str());
      }
      _have_some_mult = false;
    }
    _map.clear();
    _vec_timer_info.clear();
  }
  std::unordered_map<string, int> _map;
  struct TimerInfo {
    explicit TimerInfo(string name) : stat_cpu_times(std::move(name), false) {}
    Stat stat_cpu_times;
    double sum_process_times{0.};
    double sum_real_times{0.};
  };
  std::vector<TimerInfo> _vec_timer_info;  // in order encountered at runtime; avoid dependency on Array.h
  bool _have_some_mult{false};
};

#endif  // !defined(HH_NO_TIMERS_CLASS)

int Timer::_s_show = getenv_int("SHOW_TIMES");

Timer::Timer(string pname, EMode mode) : _name(std::move(pname)), _mode(mode) {
  if (_name == "") {
    _mode = EMode::noprint;
  } else {
    if (_s_show >= 2) showf(" (%-20.20s started)\n", sform("%.19s:", _name.c_str()).c_str());
    start();
  }
}

void Timer::terminate() {
  if (_s_show > 0 && _mode != EMode::noprint) _mode = EMode::normal;
  if (_mode == EMode::always)
    _mode = EMode::normal;
  else if (_s_show < 0)
    _mode = EMode::noprint;
  const EMode cmode = _mode;
  _mode = EMode::noprint;
  if (cmode == EMode::possibly || cmode == EMode::noprint || _name == "" || !_ever_started) return;
  if (!_started) {
    Warning("Timer not restarted");
  } else {
    stop();
  }
#if !defined(HH_NO_TIMERS_CLASS)
  if (Timers::record(*this, cmode)) return;
#endif
  const string sname = sform("%.19s:", _name.c_str());
  const double process_time = _process_cpu_time;
  const double cpu_time = cpu();
  const double real_time = real();
  const string sparallel = create_parallelism_string(process_time, real_time, 1);
  const int precision = getenv_int("HH_TIMER_PRECISION", 2, false);  // Number of fractional decimal digits.
  const string s = sform(" (%-20.20s %8.*f%s %8.*f)\n",              //
                         sname.c_str(), precision, cpu_time, sparallel.c_str(), precision, real_time);
  if (cmode == EMode::normal) {
    showdf("%s", s.c_str());
  } else {
    showf("%s%s", g_comment_prefix_string, s.c_str());
  }
}

#if defined(_WIN32)
static inline double to_seconds(const FILETIME& ft) {
  const double low_to_sec = 100e-9;  // 100 nanoseconds
  const double high_to_sec = low_to_sec * 4'294'967'296.0;
  return ft.dwLowDateTime * low_to_sec + ft.dwHighDateTime * high_to_sec;
}
#endif

inline void Timer::update_counters(int delta_sign) {
  double delta_process_time;
#if defined(_WIN32)
  if (k_win32_use_process_cycle_time) {
    ULONGLONG cycle;  // uint64_t
    // assertx(QueryThreadCycleTime(GetCurrentProcess(), &cycle));
    // _thread_cpu_time += delta_sign * (cycle / (1e6 * cpu_mhz_speed()));
    assertx(QueryProcessCycleTime(GetCurrentProcess(), &cycle));
    delta_process_time = cycle / (1e6 * cpu_mhz_speed());
  } else {
    FILETIME tcreat, texit, tkernel, tuser;
    // assertx(GetThreadTimes(GetCurrentThread(), &tcreat, &texit, &tkernel, &tuser));
    // _thread_cpu_time += delta_sign * (to_seconds(tuser) + to_seconds(tkernel));
    assertx(GetProcessTimes(GetCurrentProcess(), &tcreat, &texit, &tkernel, &tuser));
    delta_process_time = to_seconds(tuser) + to_seconds(tkernel);
  }
#else
  struct timespec ti;
  // assertx(!clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ti));
  // _thread_cpu_time += delta_sign * (double(ti.tv_sec) + double(ti.tv_nsec) * 1e-9);
  assertx(!clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ti));
  delta_process_time = double(ti.tv_sec) + double(ti.tv_nsec) * 1e-9;
#endif  // defined(_WIN32)
  _process_cpu_time += delta_sign * delta_process_time;
  _real_time_counter += delta_sign * get_precise_counter();
}

void Timer::start() {
  assertx(!_started);
  _started = true;
  _ever_started = true;
  update_counters(-1);
}

void Timer::stop() {
  assertx(_started);
  _started = false;
  update_counters(+1);
  _process_cpu_time = max(_process_cpu_time, 0.);
  _real_time_counter = max(_real_time_counter, int64_t{0});
}

double Timer::real() const {
  assertx(!_started);
  return _real_time_counter * get_seconds_per_counter();
}

double Timer::cpu() const {
  assertx(!_started);
  return min(_process_cpu_time, real());
}

double Timer::parallelism() const {
  assertx(!_started);
  return cpu() / max(real(), 1e-9);
}

}  // namespace hh
