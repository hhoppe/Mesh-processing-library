// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Hh.h"

#include <sys/stat.h>  // struct stat and fstat()

#if defined(__MINGW32__)
#include <malloc.h>  // __mingw_aligned_malloc()
#endif

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>  // WideCharToMultiByte(), WC_ERR_INVALID_CHARS, OutputDebugStringW()
#include <io.h>       // isatty(), setmode(), get_osfhandle()
#else
#include <unistd.h>  // isatty(), _exit(), etc.
#endif

#include <array>
#include <cctype>  // std::isdigit()
#include <cerrno>  // errno
#include <chrono>
#include <cstdarg>  // std::va_list
#include <cstring>  // std::memcpy(), strlen(), std::strerror()
#include <locale>   // std::use_facet<>, std::locale()
#include <map>
#include <mutex>  // std::once_flag, std::call_once()
#include <regex>
#include <unordered_map>
#include <vector>

#include "libHh/StringOp.h"  // replace_all(), remove_at_beginning(), remove_at_end()

#if !defined(_MSC_VER) && !defined(HH_NO_STACKWALKER)
#define HH_NO_STACKWALKER
#endif

#if !defined(HH_NO_STACKWALKER)
#include "libHh/StackWalker.h"
#endif

namespace hh {

// Compilation-time tests for assumptions present in my C++ code
static_assert(sizeof(int) >= 4, "");
static_assert(sizeof(char) == 1, "");
static_assert(sizeof(uchar) == 1, "");
static_assert(sizeof(short) == 2, "");
static_assert(sizeof(ushort) == 2, "");
static_assert(sizeof(int64_t) == 8, "");
static_assert(sizeof(uint64_t) == 8, "");

const char* g_comment_prefix_string = "# ";  // not string because cannot be destroyed before Timers destruction

int g_unoptimized_zero = 0;

namespace {

#if !defined(HH_NO_STACKWALKER)

// StackWalk64  https://learn.microsoft.com/en-us/windows/win32/api/dbghelp/nf-dbghelp-stackwalk  complicated
// comment: You can find article and good example of use at:
//  https://www.codeproject.com/Articles/11132/Walking-the-callstack-2
// CaptureStackBackTrace() https://learn.microsoft.com/en-us/previous-versions/windows/desktop/legacy/bb204633(v=vs.85)

// https://www.codeproject.com/Articles/11132/Walking-the-callstack-2
// "Walking the callstack"  by Jochen Kalmbach [MVP VC++]     2005-11-14   NICE!
//
// The goal for this project was the following:
// Simple interface to generate a callstack
// C++ based to allow overwrites of several methods
// Hiding the implementation details (API) from the class interface
// Support of x86, x64 and IA64 architecture
// Default output to debugger-output window (but can be customized)
// Support of user-provided read-memory-function
// Support of the widest range of development-IDEs (VC5-VC8)
// Most portable solution to walk the callstack

// See notes inside StackWalker.cpp with pointers to packages for Mingw.

// It is difficult to find a mingw-compatible version of StackWalker.
// Considered http://home.broadpark.no/~gvanem/misc/exc-abort.zip
//  which was compiled in ~/git/hh_src/_other/exc-abort.zip
// However, it does not show symbols in call stack.
// A correct implementation would have to combine the Windows-based StackWalker with the debug symbols of gcc.

// Simple implementation of an additional output to the console:
class MyStackWalker : public StackWalker {
 public:
  MyStackWalker() = default;
  // MyStackWalker(DWORD dwProcessId, HANDLE hProcess) : StackWalker(dwProcessId, hProcess) {}
  void OnOutput(LPCSTR szText) override {
    // no heap allocation!
    static std::array<char, 512> buf;  // static just in case stack is almost exhausted
    snprintf(buf.data(), int(buf.size() - 1), "%.*s", int(buf.size() - 6), szText);
    std::cerr << buf.data();
    // printf(szText);
    // StackWalker::OnOutput(szText);
  }
  void OnSymInit(LPCSTR, DWORD, LPCSTR) override {}
  void OnLoadModule(LPCSTR, LPCSTR, DWORD64, DWORD, DWORD, LPCSTR, LPCSTR, ULONGLONG) override {}
};

void show_call_stack_internal() {
  MyStackWalker sw;
  sw.ShowCallstack();
}

// Other possible stack-walking routines:

// https://www.codeproject.com/Articles/178574/Using-PDB-files-and-symbols-to-debug-your-applicat
//  "Using PDB files and symbols to debug your application" by Yanick Salzmann   2011-04-18   few downloads
//  "With the help of PDB files, you are able to recover the source code as it was before compilation
//   from the bits and bytes at runtime."

// https://stackoverflow.com/questions/6205981/windows-c-stack-trace-from-a-running-app

#else

void show_call_stack_internal() { std::cerr << "MyStackWalker is disabled, so call stack is not available.\n"; }

#endif  // !defined(HH_NO_STACKWALKER)

}  // namespace

#if defined(_WIN32)

std::string utf8_from_utf16(const std::wstring& wstr) {
  const unsigned flags = WC_ERR_INVALID_CHARS;
  // By specifying cchWideChar == -1, we include the null terminating character in nchars.
  int nchars = WideCharToMultiByte(CP_UTF8, flags, wstr.data(), -1, nullptr, 0, nullptr, nullptr);
  assertx(nchars > 0);
  string str(nchars - 1, '\0');  // does allocate space for an extra null-terminating character
  // Writing into std::string using &str[0] is arguably legal in C++11; see discussion at
  //  https://stackoverflow.com/questions/1042940/writing-directly-to-stdstring-internal-buffers
  assertx(WideCharToMultiByte(CP_UTF8, flags, wstr.data(), -1, &str[0], nchars, nullptr, nullptr));
  return str;
}

std::wstring utf16_from_utf8(const std::string& str) {
  const unsigned flags = MB_ERR_INVALID_CHARS;
  // By specifying str.size() + 1, we include the null terminating character in nwchars.
  int nwchars = MultiByteToWideChar(CP_UTF8, flags, str.data(), int(str.size() + 1), nullptr, 0);
  assertx(nwchars > 0);
  std::wstring wstr(nwchars - 1, wchar_t{0});
  assertx(MultiByteToWideChar(CP_UTF8, flags, str.data(), int(str.size() + 1), &wstr[0], nwchars));
  return wstr;
}

#endif

static string beautify_type_name(string s) {
  // SHOW(s);
  // ** general:
  s = replace_all(s, "class ", "");
  s = replace_all(s, "struct ", "");
  s = replace_all(s, " >", ">");
  s = replace_all(s, ", ", ",");
  s = replace_all(s, " *", "*");
  s = std::regex_replace(s, std::regex("std::_[A-Z_][A-Za-z0-9_]*::"), "std::");
  // ** win:
  s = replace_all(s, "std::basic_string<char,std::char_traits<char>,std::allocator<char>>", "std::string");
  // e.g. "class Map<class MVertex * __ptr64,float,struct std::hash<class MVertex * __ptr64>,struct std::equal_to<class MVertex * __ptr64>>"
  // s = replace_all(s, ",std::hash<int>,std::equal_to<int> ", "");
  s = std::regex_replace(s, std::regex(",std::hash<.*?>,std::equal_to<.*?>>"), ">");
  s = replace_all(s, "* __ptr64", "*");
  s = replace_all(s, "__int64", "int64");
  s = replace_all(s, "char const", "const char");
  // ** gcc:
  s = replace_all(s, "std::__cxx11::", "std::");  // GNUC 5.2; e.g. std::__cx11::string
  s = replace_all(s, "std::basic_string<char>", "std::string");
  s = replace_all(s, ",std::hash<int>,std::equal_to<int>>", ">");
  s = replace_all(s, ",std::hash<std::string>,std::equal_to<std::string>>", ">");
  s = replace_all(s, "long long", "int64");
  s = replace_all(s, "int64 int", "int64");
  s = replace_all(s, "int64 unsigned int", "unsigned int64");
  // ** clang:
  s = replace_all(s, "hh::Map<string,string>", "hh::Map<std::string,std::string>");
  // ** Apple clang:
  s = replace_all(s, "std::__1::basic_string<char>", "std::string");
  // ** cygwin 64-bit:
  if (sizeof(long) == 8) {  // defined(__LP64__)
    s = replace_all(s, "long unsigned int", "unsigned int64");
    s = replace_all(s, "long int", "int64");
  }
  // ** Google:
  s = replace_all(s, "basic_string<char,std::char_traits<char>,std::allocator<char>>", "std::string");
  // ** Google Forge:
  if (sizeof(long) == 8) {
    s = replace_all(s, "unsigned long", "unsigned int64");
    s = replace_all(s, "long", "int64");
  }
  // ** all:
  if (0) {
    s = replace_all(s, "std::", "");
    s = replace_all(s, "hh::", "");
    s = replace_all(s, "std::string", "string");
  }
  return s;
}

namespace details {

string forward_slash(const string& s) { return replace_all(s, "\\", "/"); }

string extract_function_type_name(string s) {
  // See experiments in ~/git/hh_src/test/misc/test_compile_time_type_name.cpp
  // Maybe "clang -std=gnu++11" was required for __PRETTY_FUNCTION__ to give adorned function name.
  s = replace_all(s, "std::__cxx11::", "std::");  // GNUC 5.2; e.g. std::__cx11::string
  // GOOGLE3: versioned libstdc++ or libc++
  s = std::regex_replace(s, std::regex("std::_[A-Z_][A-Za-z0-9_]*::"), "std::");
  if (remove_at_beginning(s, "hh::details::TypeNameAux<")) {  // VC
    if (!remove_at_end(s, ">::name")) {
      SHOW(s);
      assertnever("");
    }
    remove_at_end(s, " ");  // possible space for complex types
  } else if (remove_at_beginning(s, "static std::string hh::details::TypeNameAux<T>::name() [with T = ")) {  // GNUC
    if (!remove_at_end(s, "; std::string = std::basic_string<char>]")) {
      SHOW(s);
      assertnever("");
    }
  } else if (remove_at_beginning(s, "static string hh::details::TypeNameAux<T>::name() [with T = ")) {  // Google opt
    auto i = s.find("; ");
    if (i == string::npos) {
      SHOW(s);
      assertnever("");
    }
    s.erase(i);
    remove_at_end(s, " ");  // possible space
  } else if (remove_at_beginning(s, "static std::string hh::details::TypeNameAux<") ||
             remove_at_beginning(s, "static string hh::details::TypeNameAux<")) {  // clang
    auto i = s.find(">::name() [T = ");
    if (i == string::npos) {
      SHOW(s);
      assertnever("");
    }
    s.erase(i);
    remove_at_end(s, " ");  // possible space for complex types
  } else if (s == "name") {
    SHOW(s);
    assertnever("");
  }
  s = beautify_type_name(s);
  return s;
}

}  // namespace details

namespace {

// Maintains a set of functions, which are called upon program termination or when hh_clean_up() is called.
class CleanUp {
 public:
  using Function = void (*)();
  static void register_function(Function function) { instance()._functions.push_back(function); }
  static void flush() {
    for (auto function : instance()._functions) function();
  }

 private:
  static CleanUp& instance() {
    static CleanUp& object = *new CleanUp;
    return object;
  }
  CleanUp() { std::atexit(flush); }
  ~CleanUp() = delete;
  std::vector<Function> _functions;
};

class Warnings {
 public:
  static int increment_count(const char* s) { return ++instance()._map[s]; }
  static void flush() { instance().flush_internal(); }

 private:
  static Warnings& instance() {
    static Warnings& warnings = *new Warnings;
    return warnings;
  }
  Warnings() { hh_at_clean_up(Warnings::flush); }
  ~Warnings() = delete;
  void flush_internal() {
    if (_map.empty()) return;
    struct string_less {  // lexicographic comparison; deterministic, unlike pointer comparison
      bool operator()(const void* s1, const void* s2) const {
        return strcmp(static_cast<const char*>(s1), static_cast<const char*>(s2)) < 0;
      }
    };
    std::map<const void*, int, string_less> sorted_map(_map.begin(), _map.end());
    const bool show_in_cout = !getenv_bool("HH_HIDE_SUMMARIES");
    (show_in_cout ? showdf : showff)("Summary of warnings:\n");
    for (auto& kv : sorted_map) {
      const char* s = static_cast<const char*>(kv.first);
      int n = kv.second;
      (show_in_cout ? showdf : showff)(" %5d '%s'\n", n, details::forward_slash(s).c_str());
    }
    _map.clear();
  }
  std::unordered_map<const void*, int> _map;  // warning char* -> number of occurrences; void* for google3
};

}  // namespace

void hh_at_clean_up(void (*function)()) { CleanUp::register_function(function); }

void hh_clean_up() { CleanUp::flush(); }

void details::assertx_aux2(const char* s) {
  showf("Fatal assertion error: %s\n", details::forward_slash(s).c_str());
  if (errno) std::cerr << "possible error: " << std::strerror(errno) << "\n";
  show_possible_win32_error();
  abort();
}

// We use "const char*" rather than "string" for efficiency of hashing in Warnings.
// Ret: true if this is the first time the warning message is printed.
bool details::assertw_aux2(const char* s) {
  static const bool warn_just_once = !getenv_bool("ASSERTW_VERBOSE");
  int count = Warnings::increment_count(s);
  if (count > 1 && warn_just_once) return false;
  showf("assertion warning: %s\n", details::forward_slash(s).c_str());
  static const bool assertw_abort = getenv_bool("ASSERTW_ABORT") || getenv_bool("ASSERT_ABORT");
  if (assertw_abort) {
    my_setenv("ASSERT_ABORT", "1");
    assertx_aux2(s);
  }
  return true;
}

// may return nullptr
void* aligned_malloc(size_t size, int alignment) {
  // see https://stackoverflow.com/questions/3839922/aligned-malloc-in-gcc
#if defined(_MSC_VER)
  return _aligned_malloc(size, alignment);
#elif defined(__MINGW32__)
  return __mingw_aligned_malloc(size, alignment);
#else
  // Use: posix_memalign(void **memptr, size_t alignment, size_t size)
  void* p = nullptr;
  const int min_alignment = 8;  // else get EINVAL on Unix gcc 4.8.1
  if (alignment < min_alignment) {
    alignment = min_alignment;
    size = ((size - 1) / alignment + 1) * alignment;
  }
  if (0) SHOW(size, alignment);
  if (int ierr = posix_memalign(&p, alignment, size)) {
    if (0) SHOW(ierr, ierr == EINVAL, ierr == ENOMEM);
    return nullptr;
  }
  return p;
#endif
}

void aligned_free(void* p) {
#if defined(_MSC_VER)
  _aligned_free(p);
#elif defined(__MINGW32__)
  __mingw_aligned_free(p);
#else
  free(p);
#endif
}

std::istream& my_getline(std::istream& is, string& sline, bool dos_eol_warnings) {
  sline.clear();       // just to be safe, if the caller failed to test return value.
  getline(is, sline);  // already creates its own sentry project (with noskipws == true)
  if (is && sline.size() && sline.back() == '\r') {
    sline.pop_back();
    if (dos_eol_warnings) {
      static const bool ignore_dos_eol = getenv_bool("IGNORE_DOS_EOL");
      if (!ignore_dos_eol) Warning("my_getline: stripping out control-M from DOS file");
    }
  }
  return is;
}

namespace details {

void show_cerr_and_debug(const string& s) {
  std::cerr << s;
#if defined(_WIN32)
  // May display in debug window if such a window is present; otherwise does nothing.
  OutputDebugStringW(utf16_from_utf8(s).c_str());
#endif
}

}  // namespace details

// Should not define "sform(const string& format, ...)":
//  see: https://stackoverflow.com/questions/222195/are-there-gotchas-using-varargs-with-reference-parameters
// Varargs callee must have two versions:
//  see: https://www.c-faq.com/varargs/handoff.html   http://www.tin.org/bin/man.cgi?section=3&topic=vsnprintf
static HH_PRINTF_ATTRIBUTE(1, 0) string vsform(const char* format, std::va_list ap) {
  // Adapted from https://stackoverflow.com/questions/2342162/stdstring-formating-like-sprintf
  //  and https://stackoverflow.com/questions/69738/c-how-to-get-fprintf-results-as-a-stdstring-w-o-sprintf
  // asprintf() supported only on BSD/GCC
  const int stacksize = 256;
  char stackbuf[stacksize];  // stack-based buffer that is big enough most of the time
  int size = stacksize;
  std::vector<char> vecbuf;  // dynamic buffer just in case; do not take dependency on Array.h or PArray.h
  char* buf = stackbuf;
  bool promised = false;  // precise size was promised
  if (0) {
    std::cerr << "format=" << format << "\n";
  }
  std::va_list ap2;
  for (;;) {
    va_copy(ap2, ap);
    int n = vsnprintf(buf, size, format, ap2);  // NOLINT(clang-analyzer-valist.Uninitialized)
    va_end(ap2);
    // SHOW(size, promised, n, int(buf[size-1]));
    if (0) {
      std::cerr << "n=" << n << " size=" << size << " format=" << format << "\n";
    }
    if (promised) assertx(n == size - 1);
    if (n >= 0) {
      // if (n<size) SHOW(string(buf, n));
      if (n < size) return string(buf, n);  // it fit
      size = n + 1;
      promised = true;
    } else {
      assertx(n == -1);
      assertnever(string() + "vsform: likely a format error in '" + format + "'");
    }
    vecbuf.resize(size);
    buf = vecbuf.data();
  }
}

// Inspired from vinsertf() in https://stackoverflow.com/a/2552973
static HH_PRINTF_ATTRIBUTE(2, 0) void vssform(string& str, const char* format, std::va_list ap) {
  const size_t minsize = 40;
  if (str.size() < minsize) str.resize(minsize);
  bool promised = false;  // precise size was promised
  std::va_list ap2;
  for (;;) {
    va_copy(ap2, ap);
    // NOLINTNEXTLINE(clang-analyzer-valist.Uninitialized)
    int n = vsnprintf(&str[0], str.size(), format, ap2);  // string::data() returns const char*
    va_end(ap2);
    if (promised) assertx(n == narrow_cast<int>(str.size()) - 1);
    if (n >= 0) {
      if (n < narrow_cast<int>(str.size())) {  // It fit.
        str.resize(n);
        return;
      }
      str.resize(n + 1);
      promised = true;
    } else {
      assertx(n == -1);
      assertnever(string() + "ssform: likely a format error in '" + format + "'");
    }
  }
}

HH_PRINTF_ATTRIBUTE(1, 2) string sform(const char* format, ...) {
  std::va_list ap;
  va_start(ap, format);
  string s = vsform(format, ap);
  va_end(ap);
  return s;
}

string sform_nonliteral(const char* format, ...) {
  std::va_list ap;
  va_start(ap, format);
  // Disabling the diagnostic is unnecessary in gcc because it makes an exception when
  //  the call makes use of a std::va_list (i.e. "ap").
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
  string s = vsform(format, ap);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
  va_end(ap);
  return s;
}

HH_PRINTF_ATTRIBUTE(2, 3) const string& ssform(string& str, const char* format, ...) {
  std::va_list ap;
  va_start(ap, format);
  vssform(str, format, ap);
  va_end(ap);
  return str;
}

HH_PRINTF_ATTRIBUTE(2, 3) const char* csform(string& str, const char* format, ...) {
  std::va_list ap;
  va_start(ap, format);
  vssform(str, format, ap);
  va_end(ap);
  return str.c_str();
}

// Problem from https://stackoverflow.com/questions/3366978/what-is-wrong-with-this-recursive-va-arg-code ?
// See https://www.c-faq.com/varargs/handoff.html

HH_PRINTF_ATTRIBUTE(1, 2) void showf(const char* format, ...) {
  std::va_list ap;
  va_start(ap, format);
  string s = vsform(format, ap);
  va_end(ap);
  details::show_cerr_and_debug(s);
}

static bool isafile(int fd) {
#if defined(_WIN32)
  assertx(fd >= 0);
  HANDLE handle = reinterpret_cast<HANDLE>(_get_osfhandle(fd));
  BY_HANDLE_FILE_INFORMATION hfinfo = {};
  if (!GetFileInformationByHandle(handle, &hfinfo)) return false;
  // 2004-10-06 XPSP2: bug: now pipe returns success; detect this using the peculiar file information:
  if (hfinfo.dwVolumeSerialNumber == 0 && hfinfo.ftCreationTime.dwHighDateTime == 0 &&
      hfinfo.ftCreationTime.dwLowDateTime == 0 && hfinfo.nFileSizeHigh == 0 && hfinfo.nFileSizeLow == 0)
    return false;
  return true;
#else   // cygwin or Unix
  struct stat statbuf;
  assertx(!fstat(fd, &statbuf));
  return !HH_POSIX(isatty)(fd) && !S_ISFIFO(statbuf.st_mode) && !S_ISSOCK(statbuf.st_mode);
#endif  // defined(_WIN32)
}

// Scenarios:
//   command-line                       isatty1 isatty2 same    isf1    isf2    cout    cerr    want_ff
//   app                                1       1       1       0       0       0       1       0
//   app >file                          0       1       0       1       0       1       1       1
//   app | app2                         0       1       0       0       0       1       1       1
//   app 2>file                         1       0       0       0       1       1       1       0
//   app >&file                         0       0       1       1       1       0       1       1
//   app |& app2                        0       0       1       0       0       0       1       1(*A)
//   (app | app2) |& app3               0       0       0       0       0       1       1       1
//   (app >file) |& app2                0       0       0       1       0       1       1       1
//   (app >file) >&file2                0       0       0       1       1       1       1       1
//   blaze run app                      0       0       0       0       0       0       1       1(*B)
//
// Difficulty: for Cygwin bash shell in _WIN32, we always see isatty1=0 and isatty2=0.

static void determine_stdout_stderr_needs(bool& pneed_cout, bool& pneed_cerr, bool& pwant_ff) {
  bool need_cout, need_cerr, want_ff;
  bool isatty1 = !!HH_POSIX(isatty)(1), isatty2 = !!HH_POSIX(isatty)(2);  // _WIN32: isatty() often returns 64.
  bool same_cout_cerr;
  {
#if defined(_WIN32)
    BY_HANDLE_FILE_INFORMATION hfinfo1{}, hfinfo2{};
    same_cout_cerr = GetFileInformationByHandle(reinterpret_cast<HANDLE>(_get_osfhandle(1)), &hfinfo1) &&
                     GetFileInformationByHandle(reinterpret_cast<HANDLE>(_get_osfhandle(2)), &hfinfo2) &&
                     hfinfo1.dwVolumeSerialNumber == hfinfo2.dwVolumeSerialNumber &&
                     hfinfo1.nFileIndexHigh == hfinfo2.nFileIndexHigh &&
                     hfinfo1.nFileIndexLow == hfinfo2.nFileIndexLow;
    // You can compare the VolumeSerialNumber and FileIndex members returned in the
    //  BY_HANDLE_FILE_INFORMATION structure to determine if two paths map to the same target.
#else
    struct stat statbuf1 = {}, statbuf2 = {};
    same_cout_cerr = assertw(!fstat(1, &statbuf1)) && assertw(!fstat(2, &statbuf2)) &&
                     statbuf1.st_dev == statbuf2.st_dev && statbuf1.st_ino == statbuf2.st_ino;
    // SHOW(statbuf1.st_dev, statbuf2.st_dev, statbuf1.st_ino, statbuf2.st_ino);
#endif
  }
  need_cerr = true;
  need_cout = !same_cout_cerr;
  if (0 && !isatty1 && !isatty2 && !isafile(0) && !isafile(1)) need_cout = false;  // 2017-02-22 for "blaze run" (*B).
  if (same_cout_cerr) {
    want_ff = isafile(1) || isatty1;
    // On _WIN32, isatty1 is always false and we fail to set want_ff=1 for "app |& app2" (*A).
  } else {
    want_ff = isafile(1) || !isafile(2);
  }
  if (getenv_bool("NO_DIAGNOSTICS_IN_STDOUT")) {  // Could be set in main() by my_setenv().
    need_cout = false;
    want_ff = false;
  }
  if (getenv_bool("SHOW_NEED_COUT"))
    SHOW(isatty1, isatty2, same_cout_cerr, isafile(1), isafile(2), need_cout, need_cerr, want_ff);
  pneed_cout = need_cout;
  pneed_cerr = need_cerr;
  pwant_ff = want_ff;
}

HH_PRINTF_ATTRIBUTE(1, 2) void showdf(const char* format, ...) {
  static bool need_cout, need_cerr, want_ff;
  static std::once_flag flag;
  std::call_once(flag, determine_stdout_stderr_needs, std::ref(need_cout), std::ref(need_cerr), std::ref(want_ff));
  std::va_list ap;
  va_start(ap, format);
  string s = g_comment_prefix_string + vsform(format, ap);
  va_end(ap);
  if (need_cout) std::cout << s;
  if (need_cerr) std::cerr << s;
#if defined(_WIN32)
  OutputDebugStringW(utf16_from_utf8(s).c_str());
#endif
}

HH_PRINTF_ATTRIBUTE(1, 2) void showff(const char* format, ...) {
  static bool need_cout, need_cerr, want_ff;
  static std::once_flag flag;
  std::call_once(flag, determine_stdout_stderr_needs, std::ref(need_cout), std::ref(need_cerr), std::ref(want_ff));
  if (!want_ff) return;
  std::va_list ap;
  va_start(ap, format);
  string s = g_comment_prefix_string + vsform(format, ap);
  va_end(ap);
  std::cout << s;
}

unique_ptr<char[]> make_unique_c_string(const char* s) {
  if (!s) return nullptr;
  size_t size = strlen(s) + 1;
  auto s2 = make_unique<char[]>(size);
  // std::copy(s, s + size, s2.get());
  std::memcpy(s2.get(), s, size);  // safe for known element type (char)
  return s2;
}

static bool check_bool(const char* s) {
  if (!strcmp(s, "0")) return true;
  if (!strcmp(s, "1")) return true;
  if (!strcmp(s, "true")) return true;
  if (!strcmp(s, "false")) return true;
  return false;
}

static bool check_int(const char* s) {
  if (*s == '-' || *s == '+') s++;
  for (; *s; s++)
    if (!std::isdigit(*s)) return false;
  return true;
}

static bool check_float(const char* s) {
  for (; *s; s++)
    if (!std::isdigit(*s) && *s != '-' && *s != '+' && *s != '.' && *s != 'e') return false;
  return true;
}

int to_int(const char* s) {
  assertx(s);
  if (!check_int(s)) assertnever(string() + "'" + s + "' not int");
  return atoi(s);
}

#if defined(_WIN32)

static void unsetenv(const char* name) {
  // Note: In Unix, deletion would use sform("%s", name).
  const char* s = make_unique_c_string(sform("%s=", name).c_str()).release();  // never deleted
  assertx(!HH_POSIX(putenv)(s));  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)
}

static void setenv(const char* name, const char* value, int change_flag) {
  assertx(change_flag);
  assertx(value);
  if (!value) {
    // Note: In Unix, deletion would use sform("%s", name).
    unsetenv(name);
  } else {
    const char* s = make_unique_c_string(sform("%s=%s", name, value).c_str()).release();  // never deleted
    assertx(!HH_POSIX(putenv)(s));  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)
  }
}

#endif  // defined(_WIN32)

void my_setenv(const string& varname, const string& value) {
  assertx(varname != "");
  if (value == "")
    unsetenv(varname.c_str());
  else
    setenv(varname.c_str(), value.c_str(), 1);
}

bool getenv_bool(const string& varname, bool vdefault, bool warn) {
  const char* s = getenv(varname.c_str());
  if (!s) return vdefault;
  if (!*s) return true;
  assertx(check_bool(s));
  if (warn) showf("Environment variable '%s=%s' overrides default value '%d'\n", varname.c_str(), s, vdefault);
  return !strcmp(s, "1") || !strcmp(s, "true");
}

int getenv_int(const string& varname, int vdefault, bool warn) {
  const char* s = getenv(varname.c_str());
  if (!s) return vdefault;
  if (!*s) return 1;
  int v = to_int(s);
  if (warn) showf("Environment variable '%s=%d' overrides default value '%d'\n", varname.c_str(), v, vdefault);
  return v;
}

float getenv_float(const string& varname, float vdefault, bool warn) {
  const char* s = getenv(varname.c_str());
  if (!s) return vdefault;
  assertx(*s && check_float(s));
  float v = float(atof(s));
  // static std::unordered_map<string, int> map; if (warn && !map[varname]++) ..
  if (warn) showf("Environment variable '%s=%g' overrides default value '%g'\n", varname.c_str(), v, vdefault);
  return v;
}

string getenv_string(const string& varname, const string& vdefault, bool warn) {
#if 0 && defined(_WIN32)
  assertnever("Would likely have to never use getenv() or putenv().");
  // https://learn.microsoft.com/en-us/cpp/c-runtime-library/reference/getenv-wgetenv :
  // When two copies of the environment (MBCS and Unicode) exist simultaneously in a program, the run-time
  //  system must maintain both copies, resulting in slower execution time. For example, whenever you call
  //  _putenv, a call to _wputenv is also executed automatically, so that the two environment strings correspond.
  const wchar_t* ws = _wgetenv(utf16_from_utf8(varname).c_str());
  return ws ? utf8_from_utf16(ws) : vdefault;
#else
  const char* s = getenv(varname.c_str());
  if (!s) return vdefault;
  if (warn) showf("Environment variable '%s=%s' overrides default value '%s'\n", varname.c_str(), s, vdefault.c_str());
  return s;
#endif
}

void show_possible_win32_error() {
#if defined(_WIN32)
  if (k_debug && GetLastError() != NO_ERROR) {
    unsigned last_error = GetLastError();
    std::array<char, 2000> msg;
    int result = 0;
    // https://social.msdn.microsoft.com/Forums/vstudio/en-US/08b25925-4d40-4b59-bd71-0e20519e312f/formatmessage-fails-to-return-error-description
    if (last_error >= 12000 && last_error <= 12175)
      result =
          FormatMessageA(FORMAT_MESSAGE_FROM_HMODULE | FORMAT_MESSAGE_IGNORE_INSERTS, GetModuleHandleA("wininet.dll"),
                         last_error, 0, msg.data(), int(msg.size() - 1), nullptr);
    else
      result = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, last_error,
                              MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), msg.data(), int(msg.size() - 1), nullptr);
    if (!result) {
      SHOW(last_error);  // numeric code
      if (GetLastError() == ERROR_MR_MID_NOT_FOUND) {
        strncpy(msg.data(), "(error code not found)", msg.size() - 1);
        msg.back() = '\0';
      } else {
        strncpy(msg.data(), "(FormatMessage failed)", msg.size() - 1);
        msg.back() = '\0';
        SHOW(GetLastError());  // numeric codes
      }
    }
    showf("possible win32 error: %s", msg.data());
  }
#endif
}

void show_call_stack() { show_call_stack_internal(); }

HH_NORETURN void exit_immediately(int code) { _exit(code); }

}  // namespace hh
