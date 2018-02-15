// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Timer.h"

#include <vector>
#include <cctype>               // std::isdigit()
#include <array>
#include <thread>               // std::thread::hardware_concurrency()
#include <mutex>                // std::once_flag, std::call_once()

#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>            // GetThreadTimes(), GetProcessTimes(), FILETIME

#else

#include <time.h>               // clock_gettime()

#endif  // defined(_WIN32)

#include "Hh.h"
#if !defined(HH_NO_TIMERS_CLASS)
// #include "Map.h" // use std::unordered_map to avoid dependency on Map.h
#include <unordered_map>
#include "Stat.h"
#endif

// Return the effective number of cores (or 0 if unknown).
inline unsigned std_thread_hardware_concurrency() {
#if defined(__MINGW32__)
    return 0;
#else
    // _GLIBCXX_HAS_GTHREADS disabled for now on mingw32
    // http://stackoverflow.com/questions/5930826/how-to-enable-experimental-c11-concurrency-features-in-mingw
    return std::thread::hardware_concurrency();
#endif
}

// Overhead for use of Timer:
// (setenv TTIMER_COUNT 1000000; tTimer); look at oneabbrev and abbrev
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


namespace hh {

#if !defined(HH_NO_TIMERS_CLASS)

namespace {

HH_REFERENCE_LIB("advapi32.lib"); // RegOpenKeyEx, etc. require this library.

string timing_host() {
    string rev;
#if 1 && defined(_WIN32)
    if (1) {
        HKEY hkey;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, R"(HARDWARE\DESCRIPTION\System\CentralProcessor\0)",
                          0, KEY_QUERY_VALUE, &hkey)) {
            Warning("RegOpenKeyExA failed");
        } else {
            {
                int val; DWORD tdword = REG_DWORD, len = 4;
                if (rev=="" && !RegQueryValueExA(hkey, "~MHz", nullptr, &tdword,
                                                 reinterpret_cast<uchar*>(&val), &len)) {
                    assertx(len==4 && val>0);
                    rev = sform("%dMHz", val);
                }
            }
            {
                std::array<char,200> buf; buf[0] = '\0'; DWORD len = DWORD(buf.size()-2);
                if (rev=="" && !RegQueryValueExA(hkey, "ProcessorNameString", nullptr, nullptr,
                                                 reinterpret_cast<uchar*>(buf.data()), &len)) {
                    // "                   Intel(R) Xeon(TM) CPU 3.06GHz"
                    char* p = buf.data();
                    while (*p==' ') p++;           // skip initial whitespace (optional)
                    while (!std::isdigit(*p)) p++; // go right to CPU speed
                    rev = p;
                    // replace spaces by '_'
                    for_int(i, narrow_cast<int>(rev.size())) { if (rev[i]==' ') rev[i] = '_'; }
                }
            }
            assertx(!RegCloseKey(hkey));
        }
    }
    if (0) SHOW(rev);
    if (rev.size()<3) rev = "";
#endif  // 1 && defined(_WIN32)
    const char* s1; const char* s2;
    if (rev=="" && (s1 = getenv("PROCESSOR_REVISION"), s1)) rev = sform("R%s", s1);
    string cpu;
    if ((s1 = getenv("PROCESSOR_ARCHITECTURE"), s1) && (s2 = getenv("PROCESSOR_LEVEL"), s2))
        cpu = sform("%s-L%s-%s", s1, s2, rev.c_str());
    if (cpu=="") cpu = getenv_string("CPU");
    if (cpu=="") cpu = getenv_string("PROCESSOR_IDENTIFIER");
    if (cpu=="") cpu = "?";
    int ncores = std_thread_hardware_concurrency();
    if (ncores) cpu += sform("(%d)", ncores);
    string host = get_hostname();
    return "cpu=" + cpu + " host=" + host;
}

struct Timers {
    ~Timers()                                   { flush(); }
    void flush() {
        if (_vec_timer_info.empty()) return;
        for (const auto& timer_info : _vec_timer_info) {
            if (timer_info.stat.num()>1) _have_some_mult = true;
        }
        if (_have_some_mult || Timer::show_times()>0) {
            showdf("Summary of timers (%s):\n", timing_host().c_str());
            const int precision = getenv_int("HH_TIMER_PRECISION", 2, false);  // num fractional decimal digits
            for (const auto& timer_info : _vec_timer_info) {
                const Stat& stat = timer_info.stat;
                double stat_sum = stat.sum();
                string sparallel = "       ";
                if (0) SHOW(timer_info.stat, stat_sum, timer_info.sum_process_time, timer_info.sum_real_time);
                // bool meaningful = stat_sum>1e-8;
                if (timer_info.sum_process_time) {
                    const int ncores = std_thread_hardware_concurrency();
                    // bool meaningful = stat.avg()>.04f;
                    bool meaningful = timer_info.sum_process_time/stat.num()>.04;
                    if (meaningful) {
                        if (ncores && timer_info.sum_process_time/stat_sum>ncores)
                            stat_sum = timer_info.sum_process_time/ncores;
                        sparallel = sform("  x%-4.1f", timer_info.sum_process_time/stat_sum);
                    } else {
                        if (1 && timer_info.sum_process_time>stat_sum)
                            stat_sum = timer_info.sum_process_time;
                        if (0 && timer_info.sum_process_time/ncores>stat_sum)
                            stat_sum = timer_info.sum_process_time/ncores;
                    }
                }
                int64_t n = stat.num();
                long long ln = n;   // because "long long" may be incompatible with int64_t in __CYGWIN__ LP64
                showdf(" %-20.20s(%-6lld)%s:%s av=%9.*f   sum=%9.*f%s %9.*f\n",
                       sform("%.19s:", stat.name().c_str()).c_str(),
                       ln,
                       n>1 ? sform("%8.*f", precision, stat.min()).c_str() : "        ",
                       n>1 ? sform("%-8.*f", precision, stat.max()).c_str() : "        ",
                       precision, stat_sum/n, precision, stat_sum, sparallel.c_str(),
                       precision, timer_info.sum_real_time);
                if (stat.num()>1000 && stat.num()*1e-6>stat_sum) {
                    showdf("**Timer '%s' created more overhead than measured!\n", stat.name().c_str());
                }
            }
            _have_some_mult = false;
        }
        _map.clear();
        _vec_timer_info.clear();
    }
    // Map<string,int> _map; // avoid dependency on Map.h
    std::unordered_map<string,int> _map;
    struct TimerInfo {
        TimerInfo(string name) : stat(std::move(name), false) { }
        TimerInfo(TimerInfo&& t) : stat(std::move(t.stat)), sum_process_time(t.sum_process_time),
                                   sum_real_time(t.sum_real_time) { } // =default
        Stat stat;                                                    // thread CPU time
        double sum_process_time {0.};
        double sum_real_time {0.};
    };
    std::vector<TimerInfo> _vec_timer_info; // in order encountered at runtime; avoid dependency on Array.h
    bool _have_some_mult {false};
};

class Timers_init {
    Timers* _ptr;
    bool _post_destruct;
 public:
    // Unusual constructors and destructors to be robust even before or after static lifetime.
    Timers* get()                               { return !_ptr && !_post_destruct ? (_ptr = new Timers) : _ptr; }
    ~Timers_init()                              { delete _ptr; _ptr = nullptr; _post_destruct = true; }
} g_ptimers;

} // namespace

void flush_timers() { if (Timers* ptimers = g_ptimers.get()) ptimers->flush(); }

#else

void flush_timers() { }

#endif  // !defined(HH_NO_TIMERS_CLASS)

int Timer::_s_show = getenv_int("SHOW_TIMES");

Timer::Timer(string pname, EMode mode) : _name(std::move(pname)), _mode(mode) {
    if (_name!="" && _s_show>=2)
        showf(" (%-20.20s started)\n", sform("%.19s:", _name.c_str()).c_str());
    zero();
    start();
}

void Timer::terminate() {
    if (_s_show>0 && _mode!=EMode::noprint) _mode = EMode::normal;
    if (_s_show<0) _mode = EMode::noprint;
    EMode cmode = _mode;
    _mode = EMode::noprint;
    if (cmode==EMode::possibly || cmode==EMode::noprint || _name=="" || !_ever_started) return;
    if (!_started) {
        Warning("Timer not restarted");
    } else {
        stop();
    }
    double u = cpu();
#if !defined(HH_NO_TIMERS_CLASS)
    if (Timers* ptimers = g_ptimers.get()) {
        bool is_new; int i;
        // i = ptimers->_map.enter(_name, narrow_cast<int>(ptimers->_vec_timer_info.size()), is_new); // for hh::Map
        {
            // Note: not thread-safe.  (I assume that the timers are created outside multi-threading sections.)
            auto p = ptimers->_map.emplace(_name, narrow_cast<int>(ptimers->_vec_timer_info.size()));
            is_new = p.second; i = p.first->second;
        }
        if (is_new) {
            ptimers->_vec_timer_info.emplace_back(Timers::TimerInfo(_name));
        }
        Timers::TimerInfo& timer_info = ptimers->_vec_timer_info[i];
        timer_info.stat.enter(float(u));
        timer_info.sum_process_time += _process_cpu_time;
        if (0) SHOW(_name, timer_info.sum_real_time, real());
        timer_info.sum_real_time += real();
        if (cmode==EMode::abbrev && timer_info.stat.num()>1) return;
        if (cmode==EMode::summary) { ptimers->_have_some_mult = true; return; }
        static std::once_flag flag;
        std::call_once(flag, [] {
            if (!getenv_bool("NO_DIAGNOSTICS_IN_STDOUT"))
                showff("(Timing on %s)\n", timing_host().c_str());
        });
    } else {
        static int count = 0;
        if (++count<=5) showf("Timer '%s' terminating beyond lifetime of Timers\n", _name.c_str());
    }
#endif  // !defined(HH_NO_TIMERS_CLASS)
    string sparallel = "       ";
    // With GNU libc++, u is sometimes around -5.55112e-17 or  1.11022e-16, so I cannot test against zero.
    // bool meaningful = u>1e-8;
    // Moreover, all *cpu* times are quantized to 16 milliseconds, so need a minimum to make any sense.
    bool meaningful = u>.04;
    if (_process_cpu_time) {
        if (meaningful) {
            const int ncores = std_thread_hardware_concurrency();
            if (ncores && _process_cpu_time/u>ncores) u = _process_cpu_time/ncores;
            sparallel = sform("  x%-4.1f", _process_cpu_time/u);
        } else {
            if (_process_cpu_time>u) u = _process_cpu_time;
        }
    }
    const int precision = getenv_int("HH_TIMER_PRECISION", 2, false);  // num fractional decimal digits
    string s = sform(" (%-20.20s %8.*f%s %8.*f)\n",
                     sform("%.19s:", _name.c_str()).c_str(),
                     precision, u, sparallel.c_str(), precision, real());
    if (cmode==EMode::normal) {
        showdf("%s", s.c_str());
    } else {
        showf("%s%s", g_comment_prefix_string, s.c_str());
    }
}

void Timer::zero() {
    _ever_started = false;
    _started = false;
    _thread_cpu_time = 0.;
    _process_cpu_time = 0.;
    _real_counter = 0;
}

#if defined(_WIN32)
static inline double to_seconds(const FILETIME& ft) {
    const double low_to_sec = 100e-9; // 100 nanoseconds
    const double high_to_sec = low_to_sec*4294967296.0;
    return ft.dwLowDateTime*low_to_sec+ft.dwHighDateTime*high_to_sec;
}
#endif

void Timer::start() {
    assertx(!_started); _started = true;
    _ever_started = true;
#if defined(_WIN32)
    HANDLE cur_thread = GetCurrentThread(); // pseudo-handle
    HANDLE cur_process = GetCurrentProcess();
    FILETIME tcreat, texit, tkernel, tuser;
    assertx(GetThreadTimes(cur_thread, &tcreat, &texit, &tkernel, &tuser));
    _thread_cpu_time -= to_seconds(tuser)+to_seconds(tkernel);
    assertx(GetProcessTimes(cur_process, &tcreat, &texit, &tkernel, &tuser));
    _process_cpu_time -= to_seconds(tuser)+to_seconds(tkernel);
    // http://stackoverflow.com/questions/5532046/c-getthreadtimes-but-with-a-better-resolution
    // Replace GetThreadTimes() by QueryThreadCycleTime(), it counts cpu cycles.
    // However, CPU frequency can change dynamically!
#else
    struct timespec ti;
    assertx(!clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ti));
    _thread_cpu_time -= double(ti.tv_sec)+double(ti.tv_nsec)*1e-9;
    assertx(!clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ti));
    _process_cpu_time -= double(ti.tv_sec)+double(ti.tv_nsec)*1e-9;
#endif  // defined(_WIN32)
    _real_counter -= get_precise_counter();
}

void Timer::stop() {
    assertx(_started); _started = false;
#if defined(_WIN32)
    HANDLE cur_thread = GetCurrentThread(); // pseudo-handle
    HANDLE cur_process = GetCurrentProcess();
    FILETIME tcreat, texit, tkernel, tuser;
    assertx(GetThreadTimes(cur_thread, &tcreat, &texit, &tkernel, &tuser));
    _thread_cpu_time += to_seconds(tuser)+to_seconds(tkernel);
    assertx(GetProcessTimes(cur_process, &tcreat, &texit, &tkernel, &tuser));
    _process_cpu_time += to_seconds(tuser)+to_seconds(tkernel);
#else
    struct timespec ti;
    assertx(!clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ti));
    _thread_cpu_time += double(ti.tv_sec)+double(ti.tv_nsec)*1e-9;
    assertx(!clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ti));
    _process_cpu_time += double(ti.tv_sec)+double(ti.tv_nsec)*1e-9;
#endif  // defined(_WIN32)
    _real_counter += get_precise_counter();
}

double Timer::real() const {
    assertx(!_started);
    return max(_real_counter*get_seconds_per_counter(), 0.);
}

double Timer::cpu() const {
    assertx(!_started);
    return max(_thread_cpu_time, 0.);
}

} // namespace hh
