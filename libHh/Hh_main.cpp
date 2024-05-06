#include "libHh/Hh.h"

#include <fcntl.h>  // fcntl()

#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>              // QueryPerformanceCounter(), QueryPerformanceFrequency()
#include <direct.h>               // getcwd()
#include <shellapi.h>             // CommandLineToArgvW()
HH_REFERENCE_LIB("shell32.lib");  // CommandLineToArgvW()

#else

#include <time.h>    // clock_gettime()
#include <unistd.h>  // getcwd(), gethostname(), usleep(), etc.
#if !defined(__APPLE__)
#include <sys/sysinfo.h>  // struct sysinfo, sysinfo()
#endif

#endif  // defined(_WIN32)

#include <array>
#include <mutex>  // call_once()

#include "libHh/StringOp.h"  // get_canonical_path(), to_lower()

namespace hh {

double get_precise_time() {
#if defined(_WIN32)
  LARGE_INTEGER l;
  assertx(QueryPerformanceCounter(&l));
  return double(l.QuadPart) * get_seconds_per_counter();
#else
  struct timespec ti;
  assertx(!clock_gettime(CLOCK_MONOTONIC, &ti));
  return double(ti.tv_sec) + double(ti.tv_nsec) * 1e-9;
#endif
}

#if 0 && _MSC_VER >= 1900
#define USE_HIGH_RESOLUTION_CLOCK  // C++11 (slightly slower)
#endif

int64_t get_precise_counter() {
#if defined(USE_HIGH_RESOLUTION_CLOCK)
  using Clock = std::chrono::high_resolution_clock;
  static_assert(Clock::is_steady, "");  // should be monotonic, else we might get negative durations
  Clock::time_point t = Clock::now();
  Clock::duration duration = t.time_since_epoch();  // number of ticks, of type Clock::rep
  // SHOW(type_name<decltype(duration)>());
  // CONFIG=win: std::chrono::duration<int64, std::ratio<1, 1'000'000'000>>  (nanoseconds as signed 64-bit integer)
  return possible_cast<int64_t>(duration.count());
#elif defined(_WIN32)
  // https://stackoverflow.com/questions/2414359/microsecond-resolution-timestamps-on-windows
  // https://learn.microsoft.com/en-us/windows/win32/api/profileapi/nf-profileapi-queryperformancefrequency
  //  The high frequency counter need not be tied to the CPU frequency at all.  It will only resemble the CPU
  //  frequency is the system actually uses the TSC (TimeStampCounter) underneath.  As the TSC is generally
  //  unreliable on multicore systems it tends not to be used.  When the TSC is not used the ACPI Power
  //  Management Timer (pmtimer) may be used.  You can tell if your system uses the ACPI PMT by checking if
  //  QueryPerformanceFrequency returns the signature value of 3'579'545 (ie 3.57MHz).
  LARGE_INTEGER l;
  assertx(QueryPerformanceCounter(&l));
  return l.QuadPart;
#else
  struct timespec ti;
  assertx(!clock_gettime(CLOCK_MONOTONIC, &ti));
  return int64_t{ti.tv_sec} * 1'000'000'000 + ti.tv_nsec;
#endif
}

double get_seconds_per_counter() {
#if defined(USE_HIGH_RESOLUTION_CLOCK)
  using Clock = std::chrono::high_resolution_clock;
  constexpr std::chrono::duration<Clock::rep, std::ratio<1>> k_one_sec{1};  // 1 second
  using Duration = Clock::duration;
  constexpr Duration::rep nticks_per_sec = std::chrono::duration_cast<Duration>(k_one_sec).count();
  constexpr double sec_per_tick = 1. / nticks_per_sec;
  return sec_per_tick;  // == 1e-9 in VS2015
#elif defined(_WIN32)
  static double v;
  static std::once_flag flag;
  std::call_once(flag, [] {
    LARGE_INTEGER l;
    assertx(QueryPerformanceFrequency(&l));
    v = 1. / assertx(double(l.QuadPart));
  });
  return v;  // 3.01874e-07 (based on ACPI Power Management pmtimer)
#else
  return 1e-9;
#endif
}

void my_sleep(double sec) {
  // We sometimes get -5.8985e+307 in background thread of VideoViewer.
  if (sec < 0.) {
    SHOW("my_sleep", sec);
    sec = 0.;
  }
#if 0  // C++11
  // On Windows, only precise up to 1/60 sec.
  std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1>>{sec});
#elif defined(_WIN32)
  if (!sec) {
    // The aim is likely to give up time slice to another thread.
    SleepEx(0, TRUE);  // milliseconds; allow wake up for events
  } else {
    // Inspired from discussion at
    //  https://stackoverflow.com/questions/5801813/c-usleep-is-obsolete-workarounds-for-windows-mingw
    // Even better at https://www.geisswerks.com/ryan/FAQS/timing.html
    const bool use_1ms_time_resolution = false;
    if (use_1ms_time_resolution) {
      static std::once_flag flag;
      std::call_once(flag, [] {
        assertnever("");  // requires another library
#if 0
        // reduce Sleep/timer resolution from 16ms to <2ms (note: applies system-wide)
        timeBeginPeriod(1);
#endif
        // Note: should be matched with timeEndPeriod(1) but let program termination handle this.
        // https://stackoverflow.com/questions/7590475/
      });
    }
    const double sleep_threshold = use_1ms_time_resolution ? .002 : .03;  // seconds
    // int64_t freq; assertx(QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&freq)));
    LARGE_INTEGER freq;
    assertx(QueryPerformanceFrequency(&freq));
    LARGE_INTEGER count1;
    assertx(QueryPerformanceCounter(&count1));
    for (;;) {
      LARGE_INTEGER count2;
      assertx(QueryPerformanceCounter(&count2));
      double elapsed = (count2.QuadPart - count1.QuadPart) / double(freq.QuadPart);  // seconds
      double remaining = sec - elapsed;
      if (remaining <= 0.) break;
      if (remaining > sleep_threshold)
        SleepEx(int((remaining - sleep_threshold) * 1000. + .5), TRUE);  // in milliseconds; see note above
    }
  }
#else
  if (!assertw(!usleep(static_cast<useconds_t>(sec * 1e6)))) {
    assertx(errno == EINTR);  // possibly might be interrupted by a signal?
  }
#endif  // defined(_WIN32)
}

size_t available_memory() {
  const bool ldebug = getenv_bool("MEMORY_DEBUG");
#if defined(_WIN32)
  MEMORYSTATUSEX memory_status;
  memory_status.dwLength = sizeof(memory_status);
  assertx(GlobalMemoryStatusEx(&memory_status));
  auto physical_avail = memory_status.ullAvailPhys;
  auto virtual_avail = memory_status.ullAvailVirtual;
  if (ldebug) SHOW("win32", physical_avail, virtual_avail);
  size_t ret = assert_narrow_cast<size_t>(min(physical_avail, virtual_avail));
  return ret;
#elif defined(__APPLE__)
  if (ldebug) SHOW("available_memory() not implemented");
  // Perhaps could use https://developer.apple.com/library/ios/documentation/System/Conceptual/ManPages_iPhoneOS/man3/sysctlbyname.3.html
  return 0;
#else
  struct sysinfo sysi;
  assertx(!sysinfo(&sysi));  // https://linux.die.net/man/2/sysinfo
  uint64_t unit = sysi.mem_unit;
  uint64_t physical_avail = sysi.freeram * unit;
  uint64_t virtual_avail = static_cast<size_t>(-1);
  if (ldebug) SHOW("sysinfo", physical_avail, virtual_avail);
  size_t ret = min(physical_avail, virtual_avail);
  return ret;
#endif
}

string get_current_directory() {
#if defined(_WIN32)
  std::array<wchar_t, 2000> buffer;
  assertx(_wgetcwd(buffer.data(), int(buffer.size() - 1)));
  return get_canonical_path(utf8_from_utf16(buffer.data()));
#else
  std::array<char, 2000> buffer;
  assertx(HH_POSIX(getcwd)(buffer.data(), int(buffer.size() - 1)));
  return get_canonical_path(buffer.data());
#endif
}

string get_current_datetime() {
  // Note that the new standard library functions in <chrono> do not help with date processing.
  // One must still pass through localtime*().
  int year, month, day, hour, minute, second;
  {
#if defined(_WIN32)
    SYSTEMTIME system_time;
    GetLocalTime(&system_time);
    year = system_time.wYear;
    month = system_time.wMonth;
    day = system_time.wDay;
    hour = system_time.wHour;
    minute = system_time.wMinute;
    second = system_time.wSecond;
#else
    time_t ti = time(implicit_cast<time_t*>(nullptr));
    struct tm tm_result;
    struct tm& ptm = *assertx(localtime_r(&ti, &tm_result));  // POSIX
    year = ptm.tm_year + 1900;
    month = ptm.tm_mon + 1;
    day = ptm.tm_mday;
    hour = ptm.tm_hour;
    minute = ptm.tm_min;
    second = ptm.tm_sec;
#endif
  }
  return sform("%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
}

string get_hostname() {
  string host;
#if !defined(_WIN32)
  {
    char hostname[100];
    assertx(!gethostname(hostname, sizeof(hostname) - 1));
    host = hostname;
  }
#endif
  if (host == "") host = getenv_string("HOSTNAME");
  if (host == "") host = getenv_string("COMPUTERNAME");
  if (host == "") host = getenv_string("HOST");
  if (host == "") host = "?";
  host = to_lower(host);
  return host;
}

string get_header_info() {
  string datetime = get_current_datetime();
  string host = get_hostname();
  // Number of cores: std_thread_hardware_concurrency()
  string config;
#if defined(__clang__)
  // string __clang_version__ is longer and has space(s).
  config += sform("clang%d.%d.%d", __clang_major__, __clang_minor__, __clang_patchlevel__);
#elif defined(_MSC_VER)
  config += sform("msc%d", _MSC_VER);
#elif defined(__GNUC__)
  config += "gcc" __VERSION__;
#else
  config += "?";
#endif
#if defined(__CYGWIN__)
  config += "-cygwin";
#elif defined(__MINGW32__)
  config += "-mingw";
#elif defined(_MSC_VER)
  config += "-win";
#else
  config += "-unix";
#endif
#if defined(_M_X64) || defined(__x86_64)
  config += "-x64";
#elif defined(_M_IX86) || defined(__i386)
  config += "-x32";
#elif defined(_M_ARM)
  config += "-arm";
#else
  config += "-?";
#endif
  config += k_debug ? "-debug" : "-release";
#if defined(_DLL)
  config += "-dll";
#endif
  return datetime + " on " + host + " (" + config + ")";
}

void ensure_utf8_encoding(int& argc, const char**& argv) {
  assertx(argc > 0 && argv);
  {
    static int done = 0;
    assertx(!done++);
  }
#if defined(_WIN32)
  if (1) {  // See https://learn.microsoft.com/en-us/windows/win32/api/shellapi/nf-shellapi-commandlinetoargvw
    wchar_t** wargv;
    {
      int nargc;
      wargv = assertx(CommandLineToArgvW(GetCommandLineW(), &nargc));
      if (nargc != argc) SHOW(argc, nargc, utf8_from_utf16(GetCommandLineW()), utf8_from_utf16(wargv[0]));
      assertx(nargc == argc);
      using type = const char*;
      assertx(argc > 0);
      // Replace original argv array by a new one which contains UTF8-encoded arguments.
      argv = new type[intptr_t{argc + 1}];  // never deleted
      argv[argc] = nullptr;                 // extra nullptr is safest
      for_int(i, argc) argv[i] = make_unique_c_string(utf8_from_utf16(wargv[i]).c_str()).release();  // never deleted
    }
    LocalFree(wargv);
  }
#endif
}

bool set_fd_no_delay(int fd, bool nodelay) {
  dummy_use(fd, nodelay);
#if defined(__sgi)
  // on SGI, setting nodelay on terminal fd may cause window closure
  if (nodelay) assertx(!HH_POSIX(isatty)(fd));
#endif
    // 2014-07-04 CYGWIN64 this no longer works.  See also ~/git/hh_src/native/test_cygwin_nonblocking_read.cpp .
    // 2014-08-26 G3dcmp works again now.
#if defined(O_NONBLOCK)
  return fcntl(fd, F_SETFL, nodelay ? O_NONBLOCK : 0) != -1;
#elif defined(FNDELAY)
  return fcntl(fd, F_SETFL, nodelay ? FNDELAY : 0) != -1;
#elif defined(O_NDELAY)
  return fcntl(fd, F_SETFL, nodelay ? O_NDELAY : 0) != -1;
#elif defined(_WIN32)
  Warning("set_fd_no_delay() not implemented");
  return false;
#else
#error HH: unexpected environment.
#endif
}

}  // namespace hh
