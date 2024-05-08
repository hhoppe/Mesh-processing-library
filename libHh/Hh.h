// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HH_H_
#define MESH_PROCESSING_LIBHH_HH_H_

//       1         2         3         4         5         6         7         8         9        10        11
// 45678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789

// *** Pre-header.

// These macro definitions have no effect if #include "libHh/Hh.h" is after #include <system_files>,
// so instead one should adjust project/makefile build settings.

#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS  // Do not suggest use of *_s() secure function calls.
#endif

#if defined(_MSC_VER) && !defined(_SCL_SECURE_NO_WARNINGS)
#define _SCL_SECURE_NO_WARNINGS  // Allow calls to std::copy(const T*, const T*, T*) and std::move(T*, T*, T*).
#endif

#if defined(_WIN32) && !defined(_WIN32_WINNT)
// "#define NTDDI_VERSION NTDDI_WINXP" is no-op on __MINGW32__.
// For <windows.h>; e.g. 0x0501 == WinXP; 0x0601 == WIN7; latest constants _WIN32_WINNT_* not defined in __MINGW32__.
#define _WIN32_WINNT 0x0603  // == _WIN32_WINNT_WINBLUE; required for SetProcessDpiAwareness() in libHWin/HW.cpp
#endif

#if defined(_MSC_VER)
// Disable some nitpicky level4 warnings (for -W4).
#pragma warning(disable : 4127)  // Conditional expression is constant, e.g. "if (0)", "if (1)".
#pragma warning(disable : 4464)  // Allow #include paths containing ".." relative folders (e.g. "../libHh/Video.h").
#pragma warning(disable : 4512)  // In Release config, assignment operator could not be generated.
// Code analysis:
#pragma warning(disable : 6237)   // <zero> && <expression> is always zero.
#pragma warning(disable : 6286)   // <non-zero constant> || <expression> is always a non-zero constant.
#pragma warning(disable : 6993)   // Code analysis ignores OpenMP constructs; analyzing single-threaded code.
#pragma warning(disable : 26439)  // Declare a function noexcept.
#pragma warning(disable : 26444)  // Avoid unnamed objects with custom construction and destruction.
#pragma warning(disable : 26451)  // Using operator on a 4 byte value and then casting the result to a 8 byte value.
#pragma warning(disable : 26495)  // Always initialize a member variable.
#endif

#if defined(_WIN32)
// If later include <windef.h>, disable macros min and max, e.g. which interfere with std::numeric_limits<T>::max().
#define NOMINMAX  // Prevent min() and max() macros in <windows.h>.
// If already defined, undefine them.
#undef min
#undef max
#endif

#if defined(_DEBUG) || defined(DEBUG) || (!defined(_MSC_VER) && !defined(NDEBUG))
#define HH_DEBUG
#endif

// *** Standard headers.

#include <algorithm>  // min(), max()
#include <cmath>      // sqrt(), cos(), pow(), etc.
#include <cstdint>    // int64_t, uint64_t, int32_t, etc.
#include <iostream>   // ostream, cout, operator<<(ostream&), etc.
#include <limits>     // std::numeric_limits<>
#include <memory>     // unique_ptr<>
#include <sstream>    // stringstream
#include <stdexcept>  // runtime_error
#include <string>     // string
#include <utility>    // swap(), forward(), move(), declval<>, pair<>, index_sequence<>

// *** Variadic macros.

#include "libHh/VariadicMacros.h"  // HH_MAP_REDUCE()

// *** Language portability.

#define HH_EAT_SEMICOLON static_assert(true)  // Redundant declaration to swallow subsequent semicolon.

// Safest to indirect once through these.  https://www.parashift.com/c++-faq-lite/macros-with-token-pasting.html
#define HH_STR(e) #e
#define HH_STR2(e) HH_STR(e)
#define HH_CAT(a, b) a##b
#define HH_CAT2(a, b) HH_CAT(a, b)

#if defined(_MSC_VER)
#define HH_PRAGMA(...) __pragma(__VA_ARGS__)  // _Pragma() causes compiler internal error with OpenMP in VS 2019.
#else
#define HH_PRAGMA(...) _Pragma(HH_STR(__VA_ARGS__))
#endif

#if defined(_MSC_VER)
#define HH_POSIX(x) _##x  // On Windows, Unix functions like open(), read(), dup() have a leading underscore.
#else
#define HH_POSIX(x) x
#endif

#if defined(_MSC_VER) && !defined(HH_NO_LIB_REFERENCES)
#define HH_REFERENCE_LIB(libstring) HH_PRAGMA(comment(lib, libstring))  // e.g.: HH_REFERENCE_LIB("user32.lib");
#else
#define HH_REFERENCE_LIB(libstring) HH_EAT_SEMICOLON
#endif

#if defined(__clang__)
#define HH_PRINTF_ATTRIBUTE(...) __attribute__((format(printf, __VA_ARGS__)))
#elif defined(__GNUC__)
#define HH_PRINTF_ATTRIBUTE(...) __attribute__((format(gnu_printf, __VA_ARGS__)))
#else
#define HH_PRINTF_ATTRIBUTE(...)
#endif

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wassume"  // (Assumed expression can have side effects which will be discarded.)
#define HH_ASSUME(...) __builtin_assume(__VA_ARGS__)
#elif defined(_MSC_VER)
#define HH_ASSUME(...) __assume(__VA_ARGS__)  // Implies __analysis_assume() but is expression rather than statement.
#elif 0
// #define HH_ASSUME(...) do { if (!(__VA_ARGS__)) __builtin_unreachable(); } while (false)  // Maybe for gcc.
// #define HH_ASSUME(...) void(__builtin_expect(!(__VA_ARGS__), 0))   // In gcc but intended for branch prediction.
#else
#define HH_ASSUME(...) (void(0))
#endif

#if defined(_MSC_VER)
#define HH_UNREACHABLE __assume(0)  // This path is never taken.
#else
#define HH_UNREACHABLE __builtin_unreachable()
#endif

#define HH_ID(x) HH_CAT(_hh_id_, x)  // Private identifier in a macro definition.
#define HH_UNIQUE_ID(x) HH_CAT2(HH_CAT2(HH_CAT2(_hh_id_, __COUNTER__), _), x)

// *** Syntactic sugar.

#define traditional_for_T_aux(T, i, start, stop, stop_var) for (T stop_var = stop, i = start; i < stop_var; i++)
#define traditional_for_T(T, i, start, stop) traditional_for_T_aux(T, i, start, stop, HH_UNIQUE_ID(stop_var))
#if !defined(HH_DEBUG)
#define for_T(T, i, start, stop) traditional_for_T(T, i, start, stop)
#else
#if defined(_MSC_VER)
#pragma warning(disable : 4701 4703)  // Several: warning C4701: potentially uninitialized local variable.
#endif
#define for_T(T, i, start, stop) for (const T i : hh::range<T>(start, stop))  // In Debug, check "const T i" works.
#endif
#define for_int(i, stop) for_T(int, i, 0, stop)
#define for_intL(i, start, stop) for_T(int, i, start, stop)
#define for_size_t(i, stop) for_T(std::size_t, i, 0, stop)

// *** Check for identifier conflicts.

#if defined(TEST_IF_MY_IDENTIFIERS_CONFLICT_WITH_STD_NAMESPACE)
#if defined(__clang__)
#pragma clang diagnostic ignored "-Wheader-hygiene"
#endif
using namespace std;
namespace hh {}
using namespace hh;
#endif

// *** Ensure hh::details::hh_init() is called.

#if !defined(HH_NO_HH_INIT)
#include "libHh/Hh_init.h"
#endif

// *** Begin namespace.

namespace hh {

// *** Import some standard C++ names into the hh namespace.

// Common types:
using std::make_unique;
using std::size_t;  // (It may be already imported.)
using std::string;  // Almost a new fundamental type.
using std::unique_ptr;
// Useful general functions:
using std::clamp;
// Math functions that are overloaded for vectors:
using std::abs;  // From <cmath>; else non-templated defined only for int from <cstdlib>; abs(1.5) == 1 is too scary.
using std::ceil;
using std::floor;
using std::max;  // Avoid fmax().
using std::min;  // Avoid fmin().
// Other common math functions:
using std::pow;
using std::sqrt;

// *** Useful type abbreviations.

using uchar = unsigned char;
using ushort = unsigned short;

// *** Forward declaration of implementation details.

namespace details {
template <typename T> struct identity { using type = T; };
template <typename T> struct sum_type;
template <typename T> class Range;
}  // namespace details

// *** Generalized casting.

// For use in upcasting to a base class, converting nullptr, or declaring type in ternary operand.
template <typename Dest> Dest implicit_cast(typename details::identity<Dest>::type t) { return t; }

// For use in downcasting to a derived class.
template <typename Dest, typename Src> Dest down_cast(Src* f) {
  static_assert(std::is_base_of_v<Src, std::remove_pointer_t<Dest>>);
  return static_cast<Dest>(f);
}

// Conversion with bounds-checking in Debug configuration.
template <typename Target, typename Source> constexpr Target narrow_cast(Source v);

// Bounds-safe conversion, checked even in Release configuration.
template <typename Target, typename Source> constexpr Target assert_narrow_cast(Source v);

// Type conversion, but avoiding a warning in the case that Source is already of the same type as Target.
template <typename Target, typename Source> constexpr Target possible_cast(Source v) { return static_cast<Target>(v); }

// Cast a temporary as an lvalue; be careful; only use when safe.
template <typename T> T& as_lvalue(T&& e) { return const_cast<T&>(static_cast<const T&>(e)); }

// *** Constants.

constexpr float BIGFLOAT = 1e30f;                // note: different from FLT_MAX or (INFINITY == HUGE_VALF)
constexpr float TAU = 6.2831853071795864769f;    // Mathematica: N[2 Pi, 20]; see https://tauday.com/
constexpr double D_TAU = 6.2831853071795864769;  // Mathematica: N[2 Pi, 20]; see https://tauday.com/
// #undef PI  // instead, use TAU / 2

// *** Utility classes.

// Derive from this class to disable copy constructor and copy assignment.
struct noncopyable;

// Specialize<i> is a dummy type for function specialization.
template <int> struct Specialize {};

// *** Assertions, warnings, errors, debug.

#if defined(HH_DEBUG)
constexpr bool k_debug = true;  // Convenience variable to avoid introducing "#if defined(HH_DEBUG)".
#else
constexpr bool k_debug = false;  // Convenience variable to avoid introducing "#if defined(HH_DEBUG)".
#endif

// Value used to prevent compiler optimizations; it is always zero but unknown to the compiler.
extern int g_unoptimized_zero;

#define HH_FL " in line " HH_STR2(__LINE__) " of file " __FILE__

// Always abort.
#define assertnever(...) hh::details::assertx_aux2(hh::details::add_fl((__VA_ARGS__), HH_FL))

// Always abort; omit warning about any subsequent unreachable code.
#define assertnever_ret(...) \
  (hh::g_unoptimized_zero ? void() : hh::details::assertx_aux2(hh::details::add_fl((__VA_ARGS__), HH_FL)))

// if !expr, exit program (abort); otherwise return expr.
#define assertx(...) hh::details::assertx_aux((__VA_ARGS__), "assertx(" #__VA_ARGS__ ")" HH_FL)

// If !expr, throw std::runtime_error exception; otherwise return expr.
#define assertt(...) hh::details::assertt_aux((__VA_ARGS__), "assertt(" #__VA_ARGS__ ")" HH_FL)

// If !expr, report warning once.  Returns expr.
#define assertw(...) hh::details::assertw_aux((__VA_ARGS__), "assertw(" #__VA_ARGS__ ")" HH_FL)

// Reports string expr as warning once.  Return true if this is the first time the warning is reported.
#define Warning(...) hh::details::assertw_aux2(__VA_ARGS__ HH_FL)

#if defined(HH_DEBUG)
#define ASSERTX(...) assertx(__VA_ARGS__)   // In release, do not evaluate expression.
#define ASSERTXX(...) assertx(__VA_ARGS__)  // In release, do not even see expression -- maximum optimization.
#define HH_CHECK_BOUNDS(i, n) ((i >= 0 && i < n) ? (void(0)) : assertnever(sform("bounds i=%d n=%d", i, n)))
#else
#define ASSERTX(...) ((false ? void(__VA_ARGS__) : void(0)), HH_ASSUME(__VA_ARGS__))
#define ASSERTXX(...) (void(0))
#define HH_CHECK_BOUNDS(i, n) (void(0))
#endif  // defined(HH_DEBUG)

// *** Output functions.

#if 0
{
  showf("%s: Argument '%s' ambiguous, '%s' assumed\n", argv[0], arg.c_str(), assumed.c_str());
  std::cerr << sform(" Endprogram: %dgons %dlines\n", ngons, nlines);
  SHOW(x, y, x * y);
  SHOW_PRECISE(point + vector);  // Show value with greater precision.
  SHOWL;                         // Show current filename and line number.
}
#endif

// With one expression, show "expr = value" on stderr and return expr; may require parentheses: SHOW((ntimes<3>(1))).
// With multiple expressions, show on stderr a sequence of "expr=value" on a single line.
#define SHOW(...) HH_PRIMITIVE_CAT((HH_SHOW_, HH_GT1_ARGS(__VA_ARGS__)))(#__VA_ARGS__, false, __VA_ARGS__)

// Show expression(s) like SHOW(expr) but with more digits of floating-point precision.
#define SHOW_PRECISE(...) HH_PRIMITIVE_CAT((HH_SHOW_, HH_GT1_ARGS(__VA_ARGS__)))(#__VA_ARGS__, true, __VA_ARGS__)

// Show current file and line number.
#define SHOWL \
  hh::details::show_cerr_and_debug(hh::details::forward_slash("Now in " __FILE__ " at line " HH_STR2(__LINE__) "\n"))

// Write formatted string to std::cerr.
HH_PRINTF_ATTRIBUTE(1, 2) void showf(const char* format, ...);

// Write formatted string (with g_comment_prefix_string == "# ") to std::cerr and possibly to std::cout.
HH_PRINTF_ATTRIBUTE(1, 2) void showdf(const char* format, ...);

// Write formatted string (with g_comment_prefix_string == "# ") to std::cout only if it is a file or pipe.
HH_PRINTF_ATTRIBUTE(1, 2) void showff(const char* format, ...);

// C-string prefix for formatted std::cout output in showdf() and showff(); default is "# ".
extern const char* g_comment_prefix_string;

// Override "char*" output to detect nullptr in C-string.
inline std::ostream& operator<<(std::ostream& os, const char* s);

// Nice formatted output of a std::pair<> of objects.
template <typename A, typename B> std::ostream& operator<<(std::ostream& os, const std::pair<A, B>& p) {
  return os << "[" << p.first << ", " << p.second << "]";
}

// By default, assume that types do not end their stream output with a newline character.
template <typename T> struct has_ostream_eol_aux { static constexpr bool value = false; };

// Declares that the specified type ends its stream output with a newline character; must be placed in namespace hh.
#define HH_DECLARE_OSTREAM_EOL(...)         \
  struct has_ostream_eol_aux<__VA_ARGS__> { \
    static constexpr bool value = true;     \
  }

// Identifies if a type T ends its stream output (i.e. operator<<(std::ostream)) with a newline character "\n".
template <typename T> constexpr bool has_ostream_eol() {
  // (function template cannot partially specialize, e.g. Array<T>)
  return has_ostream_eol_aux<std::remove_cv_t<std::remove_reference_t<T>>>::value;
}

// For the specified container type, define an ostream operator<<() that iterates over the container's range.
#define HH_DECLARE_OSTREAM_RANGE(...)                                                                    \
  std::ostream& operator<<(std::ostream& os, const __VA_ARGS__& c) { return os << hh::stream_range(c); } \
  HH_EAT_SEMICOLON

// *** Inline definitions.

// Avoid warnings of unused variables.
template <typename... A> constexpr void dummy_use(const A&...) {}
// Note: any compilation errors about redefinition of dummy_use(), etc. on MS VS may be due to the current
// directory being different from that in precompiled header due to symbol links,
// or to an explicit MeshRoot environment variable that does not match the current tree.

// Avoid warnings of uninitialized variables.
template <typename T> void dummy_init(T& v) { v = T(); }
template <typename T, typename... A> void dummy_init(T& v, A&... a) { v = T{}, dummy_init(a...); }

// Zero-out the variable e; this function is specialized for float, double, Vector4, Vector4i.
template <typename T> void my_zero(T& e);

// Returns T{-1} or T{+1} based on sign of expression.
template <typename T> constexpr T sign(const T& e) { return e >= T{0} ? T{1} : T{-1}; }

// Returns { T{-1}, T{0}, T{+1} } based on sign of expression.
template <typename T> constexpr T signz(const T& e) { return e > T{0} ? T{1} : e < T{0} ? T{-1} : T{0}; }

// Returns a value times itself.
template <typename T> constexpr T square(const T& e) { return e * e; }

// Like clamp() but slightly less efficient; however, it works on values like Vector4.
template <typename T> constexpr T general_clamp(const T& v, const T& a, const T& b) { return min(max(v, a), b); }

// Linearly interpolate between two values (f == 1.f returns v1; f == 0.f returns v2).
inline constexpr float interp(float v1, float v2, float f = 0.5f);

// Linearly interpolate between two values (f == 1. returns v1; f == 0. returns v2).
inline constexpr double interp(double v1, double v2, double f = 0.5);

// Returns v clamped to range [0, 255].
inline uint8_t clamp_to_uint8(int v);

// Returns j%3 (where j is in [0, 5]).
inline int mod3(int j);

// Rounds floating-point value v to the nearest 1/fac increment (by default, fac == 1e5f).
template <typename T> T round_fraction_digits(T v, T fac = 1e5f) { return floor(v * fac + .5f) / fac; }

// Higher-precision type to represent the sum of a set of elements.
template <typename T> using sum_type_t = typename details::sum_type<T>::type;

// Range of integers as in Python range(stop):  e.g.: for (const int i : range(5)) { SHOW(i); } gives 0..4 .
template <typename T> details::Range<T> range(T stop) { return details::Range<T>(stop); }

// Range of integers as in Python range(start, stop):  e.g.: for (const int i : range(2, 5)) { SHOW(i); } gives 2..4 .
template <typename T> details::Range<T> range(T start, T stop) { return details::Range<T>(start, stop); }

// *** Functions defined in Hh.cpp.

// Register a function to be called by hh_clean_up(); used by timers, statistics, and warnings.
void hh_at_clean_up(void (*function)());

// Flush summaries of timers, statistics, and warnings before program termination -- perhaps before output.
void hh_clean_up();

#if defined(_WIN32)
// Convert Windows UTF-16 std::wstring to UTF-8 std::string.
std::string utf8_from_utf16(const std::wstring& wstr);

// Convert UTF-8 std::string to Windows UTF-16 std::wstring.
std::wstring utf16_from_utf8(const std::string& str);
#endif

// e.g. SHOW(type_name<T>());
template <typename T> string type_name();
template <typename T> string type_name(const T&) { return type_name<T>(); }

// Write an object e to its string representation using operator<<(ostream&, const T&).
template <typename T> string make_string(const T& e);

// Format a string like sprintf() but directly return an adaptively-sized std::string.
HH_PRINTF_ATTRIBUTE(1, 2) string sform(const char* format, ...);

// Format a string like sform() but disable warnings concerning a non-literal format string.
string sform_nonliteral(const char* format, ...);

// Format a string like sprintf() but directly to a reused adaptively-resized output buffer named str.
HH_PRINTF_ATTRIBUTE(2, 3) const string& ssform(string& str, const char* format, ...);

// Shortcut for ssform(str, format, ...).c_str(); like sprintf() but using an adaptively resized buffer str.
HH_PRINTF_ATTRIBUTE(2, 3) const char* csform(string& str, const char* format, ...);

// Allocate a duplicate of a C "char*" string, using make_unique<char[]>.
unique_ptr<char[]> make_unique_c_string(const char* s);

// Convert string to integer value, or crash if invalid.
int to_int(const char* s);

// Convert string to integer value, or crash if invalid.
inline int to_int(const string& s) { return to_int(s.c_str()); }

// Allocate an aligned memory block (returns nullptr if fails).  Portable std::aligned_alloc().
void* aligned_malloc(size_t alignment, size_t size);

// Deallocate an aligned memory block previously created by aligned_malloc().  Portable std::free().
void aligned_free(void* p);

// Allocate n elements of type T, with appropriate memory alignment based on T.
template <typename T> T* aligned_new(size_t n);

// Deallocate aligned memory.
template <typename T> void aligned_delete(T* p);

// Read a line of input (trailing "\n" is discarded).
std::istream& my_getline(std::istream& is, string& sline, bool dos_eol_warnings = true);

// Set an environment variable; the variable is removed from the environment if value == "".
void my_setenv(const string& varname, const string& value);

// Return false if environment variable is {undefined, "0", or "false"}, true if {"", "1", or "true"), else abort.
bool getenv_bool(const string& varname, bool vdefault = false, bool warn = false);

// Return vdefault if environment variable varname is not defined, 1 if "", value if integer, else abort.
int getenv_int(const string& varname, int vdefault = 0, bool warn = false);

// Return vdefault if environment variable varname is not defined, value if float, else abort.
float getenv_float(const string& varname, float vdefault, bool warn = false);

// Return string value of environment variable varname, or "" if not defined.
string getenv_string(const string& varname, const string& vdefault = "", bool warn = false);

// Return typed value of environment variable varname.
template <typename T> T getenv_type(const string& varname, T vdefault, bool warn = false);
template <> inline bool getenv_type<bool>(const string& var, bool vdefault, bool warn) {
  return getenv_bool(var, vdefault, warn);
}
template <> inline int getenv_type<int>(const string& var, int vdefault, bool warn) {
  return getenv_int(var, vdefault, warn);
}
template <> inline float getenv_type<float>(const string& var, float vdefault, bool warn) {
  return getenv_float(var, vdefault, warn);
}
template <> inline string getenv_type<string>(const string& var, string vdefault, bool warn) {
  return getenv_string(var, vdefault, warn);
}

// Show any Win32 error if on _WIN32.
void show_possible_win32_error();

// Show the call stack if possible.
void show_call_stack();

// For Unix _exit(code).
[[noreturn]] void exit_immediately(int code);

// *** Hh_main.cpp.

// Return absolute time, in secs (accuracy at least ~.001).
double get_precise_time();

// Return absolute time, in cycle counts.
int64_t get_precise_counter();

// Return seconds/cycle.
double get_seconds_per_counter();

// Delay for some number of seconds.
void my_sleep(double sec);

// Get number of bytes of available memory (min of free virtual and physical space), or 0 if unavailable.
size_t available_memory();

// Return current directory.
string get_current_directory();

// String like "2016-02-15 18:02:28".
string get_current_datetime();

// Return machine name, in lowercase.
string get_hostname();

// String with date, time, machine, build parameters.
string get_header_info();

// On Windows, replace the command-line argv with a new one that contains UTF-8 encoded strings; else do nothing.
void ensure_utf8_encoding(int& argc, const char**& argv);

// Use fcntl() to make read() calls non-blocking; returns false if failure.
bool set_fd_no_delay(int fd, bool nodelay);

//----------------------------------------------------------------------------

// HH_REFERENCE_LIB("libHh.lib");

namespace details {

// Evaluates to false in boolean context for use in macro as:
// "if (details::false_capture<int> i = stop) { HH_UNREACHABLE; } else".
template <typename T> struct false_capture {
  template <typename... Args> false_capture(Args&&... args) : _e(args...) {}
  operator bool() const { return false; }
  const T& operator()() const { return _e; }
  T _e;
};

string forward_slash(const string& s);

[[noreturn]] void assertx_aux2(const char* s);

[[noreturn]] inline void assertx_aux2(const std::string& s) { assertx_aux2(s.c_str()); }

bool assertw_aux2(const char* s);

inline string add_fl(string s, const char* file_line) { return s + file_line; }

template <typename T> constexpr T assertx_aux(T&& val, const char* s) {
  if (!val) assertx_aux2(s);
  return std::forward<T>(val);
}

template <typename T> T assertt_aux(T&& val, const char* s) {
  if (!val) throw std::runtime_error(forward_slash(s));
  return std::forward<T>(val);
}

template <typename T> T assertw_aux(T&& val, const char* s) {
  if (!val) assertw_aux2(s);
  return std::forward<T>(val);
}

void show_cerr_and_debug(const string& s);

#define HH_SHOW_0(sargs, prec, arg1) hh::details::show_aux(sargs, arg1, hh::has_ostream_eol<decltype(arg1)>(), prec)

#define HH_SHOW_1(sargs, prec, ...)                                            \
  do {                                                                         \
    std::ostringstream HH_ID(oss);                                             \
    if (prec) HH_ID(oss).precision(std::numeric_limits<double>::max_digits10); \
    HH_ID(oss) << HH_MAP_REDUCE((HH_SHOW_M, << " " <<, __VA_ARGS__)) << "\n";  \
    hh::details::show_cerr_and_debug(assertx(HH_ID(oss)).str());               \
  } while (false)

#define HH_SHOW_M(x) (#x[0] == '"' ? "" : #x "=") << (x)

template <typename T> T show_aux(string str, T&& val, bool has_eol, bool high_precision) {
  std::ostringstream oss;
  if (high_precision) oss.precision(std::numeric_limits<double>::max_digits10);
  oss << (str[0] == '"' ? "" : str + " = ") << val << (has_eol ? "" : "\n");
  // Single write for better behavior under parallelism.  For guaranteed atomic write, use std::lock_guard<>.
  show_cerr_and_debug(assertx(oss).str());
  return std::forward<T>(val);
}

string extract_function_type_name(string s);

template <typename T> struct TypeNameAux {
#if defined(__GNUC__) || defined(__clang__)
  // https://www.gamedev.net/forums/topic/608203-c-get-type-name-without-rtti/
  // https://stackoverflow.com/questions/4384765/whats-the-difference-between-pretty-function-function-func
  static string name() { return extract_function_type_name(__PRETTY_FUNCTION__); }
#elif defined(_MSC_VER)
  static string name() { return extract_function_type_name(__FUNCTION__); }  // Also __FUNCSIG__, __FUNCDNAME__.
#else
  static string name() { return extract_function_type_name(__func__); }  // C++11 (C99); unadorned so likely to fail!
#endif
};

template <typename T> struct sum_type {
  using type = std::conditional_t<!std::is_arithmetic_v<T>, T,
                                  std::conditional_t<std::is_floating_point_v<T>, double,
                                                     std::conditional_t<std::is_signed_v<T>, int64_t, uint64_t>>>;
};

// Range of integral elements defined as in Python range(start, stop), where step is 1 and stop is not included.
template <typename T> class Range {
  static_assert(std::is_integral_v<T>);  // Must have exact arithmetic for equality testing.
 public:
  class iterator {
    using type = iterator;

   public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = int64_t;  // (It may be larger than std::ptrdiff_t.)
    using pointer = value_type*;
    using reference = value_type&;
    iterator(T start, T stop) : _v(start), _stop(stop) {}
    iterator(const type& iter) = default;
    bool operator==(const type& rhs) const { return _v == rhs._v; }
    bool operator!=(const type& rhs) const { return !(*this == rhs); }
    bool operator<(const type& rhs) const { return _v < rhs._v; }
    bool operator<=(const type& rhs) const { return _v <= rhs._v; }
    bool operator>(const type& rhs) const { return _v > rhs._v; }
    bool operator>=(const type& rhs) const { return _v >= rhs._v; }
    difference_type operator-(const type& rhs) const { return difference_type(_v) - rhs._v; }
    type operator+(difference_type n) const { return iterator(*this) += n; }
    type& operator+=(difference_type n) {
      ASSERTXX(_v < _stop);
      _v = T(_v + n);
      ASSERTXX(_v <= _stop);
      return *this;
    }
    const T& operator*() const { return (ASSERTXX(_v < _stop), _v); }
    type& operator++() { return *this += T{1}; }
    T operator[](size_t i) const { return (ASSERTXX(T(_v + i) < _stop), T(_v + i)); }

   private:
    T _v, _stop;
  };

  using value_type = T;
  using const_iterator = iterator;
  using size_type = size_t;
  explicit Range(T stop) : Range(T{0}, stop) {}
  Range(T start, T stop) : _start(min(start, stop)), _stop(stop) {}
  iterator begin() const { return iterator(_start, _stop); }
  iterator end() const { return iterator(_stop, _stop); }
  iterator cbegin() const { return iterator(_start, _stop); }
  iterator cend() const { return iterator(_stop, _stop); }
  size_t size() const { return _stop - _start; }
  bool empty() const { return _stop == _start; }

 private:
  T _start, _stop;
};

}  // namespace details

template <typename Target, typename Source> constexpr Target narrow_cast(Source v) {
  Target v2 = static_cast<Target>(v);
  ASSERTXX(static_cast<Source>(v2) == v);
  return v2;
}

template <typename Target, typename Source> constexpr Target assert_narrow_cast(Source v) {
  Target v2 = static_cast<Target>(v);
  assertx(static_cast<Source>(v2) == v);
  return v2;
}

struct noncopyable {
 protected:
  noncopyable(const noncopyable&) = delete;
  noncopyable& operator=(const noncopyable&) = delete;
  noncopyable() = default;
};

inline std::ostream& operator<<(std::ostream& os, const char* s) { return std::operator<<(os, s ? s : "<nullptr>"); }

// User-defined ostream manipulator, which is constructed from a container and writes out its elements.
template <typename C> class stream_range {
 public:
  stream_range(const C& c) : _c(c) {}
  friend std::ostream& operator<<(std::ostream& os, const stream_range& sc) {
    os << type_name<C>() << "={\n";
    for (const auto& e : sc._c) {
      os << "  " << e << (has_ostream_eol<decltype(e)>() ? "" : "\n");
    }
    return os << "}\n";
  }

 private:
  const C& _c;
};

template <typename T> void my_zero(T& e) {
  // e = T{};  // Bad because default constructor can leave object uninitialized, e.g. Vector, Vec<T>, Vector4.
  static constexpr T k_dummy_zero_object{};
  e = k_dummy_zero_object;
}
template <> inline void my_zero(float& e) { e = 0.f; }
template <> inline void my_zero(double& e) { e = 0.; }

inline constexpr float interp(float v1, float v2, float f) {
  return f * v1 + (1.f - f) * v2;  // or v2 + (v1 - v2) * f
}

inline constexpr double interp(double v1, double v2, double f) {
  return f * v1 + (1. - f) * v2;  // or v2 + (v1 - v2) * f
}

inline uint8_t clamp_to_uint8(int v) {
  // return clamp(v, 0, 255);
  // https://codereview.stackexchange.com/questions/6502/fastest-way-to-clamp-an-integer-to-the-range-0-255
  v &= -(v >= 0);
  return static_cast<uint8_t>(v | ((255 - v) >> 31));
}

inline int mod3(int j) {
  static const int ar_mod3[6] = {0, 1, 2, 0, 1, 2};
  ASSERTX(j >= 0 && j < 6);
#if defined(_MSC_VER)
#pragma warning(suppress : 6385)
#endif
  return ar_mod3[j];
}

template <typename T> string type_name() { return details::TypeNameAux<std::decay_t<T>>::name(); }

template <typename T> string make_string(const T& e) {
  std::ostringstream oss;
  oss << e;
  return assertx(oss).str();
}

template <typename T> T* aligned_new(size_t n) {
  return alignof(T) <= 8 ? new T[n] : static_cast<T*>(aligned_malloc(alignof(T), n * sizeof(T)));
}

template <typename T> void aligned_delete(T* p) {
  if (alignof(T) <= 8)
    delete[] p;
  else
    aligned_free(p);
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_HH_H_
