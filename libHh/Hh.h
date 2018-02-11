// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HH_H_
#define MESH_PROCESSING_LIBHH_HH_H_

//       1         2         3         4         5         6         7         8         9        10        11
//345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789

// *** Pre-header

// These macro definitions have no effect if #include "Hh.h" is after #include <>,
//  so instead one should adjust project/makefile build settings.

#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS // do not suggest use of *_s() secure function calls
#endif

#if defined(_MSC_VER) && !defined(_SCL_SECURE_NO_WARNINGS)
#define _SCL_SECURE_NO_WARNINGS // allow calls to std::copy(const T*, const T*, T*) and std::move(T*, T*, T*)
// avoiding warning C4996 http://msdn.microsoft.com/en-us/library/ttcz0bys.aspx
#endif

#if defined(_MSC_VER)
#define HH_POSIX(x) _ ## x
#else
#define HH_POSIX(x) x
#endif

#if defined(_WIN32) && !defined(_WIN32_WINNT)
// "#define NTDDI_VERSION NTDDI_WINXP" is no-op on __MINGW32__
// For <windows.h>; e.g. 0x0501==WinXP; 0x0601==WIN7; latest constants _WIN32_WINNT_* not defined in __MINGW32__
#define _WIN32_WINNT 0x0603 // ==_WIN32_WINNT_WINBLUE; required for SetProcessDpiAwareness() in libHWin/HW.cpp
#endif

#if defined(_MSC_VER)
// Disable some nitpicky level4 warnings (for -W4).
#pragma warning(disable:4127)   // conditional expression is constant, e.g. "if (0)", "if (1)"
#pragma warning(disable:4512)   // in Release config: assignment operator could not be generated
#pragma warning(disable:4592)   // bug in VS2015 update 1; http://stackoverflow.com/questions/34013930/
#pragma warning(disable:4464)   // #include paths containing ".." relative folders (e.g. "../libHh/Video.h")
// Workarounds for warning 4702 (unreachable code) (pragma would have to appear before function body):
//  for (; ; ++iter) { f(); break; }    // replace "break;" by "if (1) break;"
//  { assertnever("abandonned"); f(); } // use "assertnever_ret(..);"
// Code analysis
#pragma warning(disable:6993)   // Code analysis ignores OpenMP constructs; analyzing single-threaded code
#pragma warning(disable:6237)   // (<zero> && <expression>) is always zero
#pragma warning(disable:6286)   // (<non-zero constant> || <expression>) is always a non-zero constant
#endif

#if defined(HH_DEFINE_FLOAT128)  // workaround for current clang with mingw32 4.7.2
typedef struct { long double x, y; } __float128;
#endif

#if defined(_WIN32)
// If later include <windef.h>, disable macros min and max, e.g. which interfere with std::numeric_limits<T>::max().
// If already defined, undefine them.
#undef min
#undef max
#undef NOMINMAX                 // (defined as "1" in __MINGW32__ but "" in vt_basetypes.h)
#define NOMINMAX 1              // prevent min() and max() macros in <windows.h>
#endif

// Examine _DEBUG if _MSC_VER, or DEBUG if __APPLE__, else !NDEBUG
#if defined(_DEBUG) || defined(DEBUG) || (!defined(_MSC_VER) && !defined(NDEBUG))
//#pragma message "HH_DEBUG enabled"
#define HH_DEBUG
#endif


// *** Variadic macros

#include "VariadicMacros.h"     // HH_MAP_REDUCE()


// *** Standard headers

#include <cmath>                // sqrt(), cos(), pow(), etc.
#include <iostream>             // ostream, cout, operator<<(ostream&), etc.
#include <string>               // string
#include <memory>               // unique_ptr<>
#include <cstdint>              // int64_t, uint64_t, int32_t, etc.
#include <algorithm>            // min(), max()
#include <sstream>              // stringstream
#include <stdexcept>            // runtime_error (already included except with __clang__)
#include <climits>              // INT_MAX       (already included except with __clang__)

// Already included from above:
// #include <utility>           // swap(), forward(), move(), declval(), pair<>, index_sequence<>


// *** Non-standard headers

#if !defined(_WIN32)
#include <unistd.h>             // read(), write(), _exit(), etc.
#endif


// *** Language portability

#if 0                      // temporarily test that my identifiers do not conflict with those in std namespace
#if defined(__clang__)
#pragma clang diagnostic ignored "-Wheader-hygiene"
#endif
using namespace std;
namespace hh { }
using namespace hh;
#endif

#define HH_EAT_SEMICOLON static_assert(true, "") // redundant declaration to swallow subsequent semicolon

// Safest to indirect once through these.  http://www.parashift.com/c++-faq-lite/macros-with-token-pasting.html
#define HH_STR(e) #e
#define HH_STR2(e) HH_STR(e)
#define HH_CAT(a, b) a ## b
#define HH_CAT2(a, b) HH_CAT(a, b)

#if defined(_MSC_VER)           // _Pragma() still not defined in Visual Studio 2017 (_MSC_VER==1910)
#define HH_PRAGMA(...) __pragma(__VA_ARGS__)
#else
#define HH_PRAGMA(...) _Pragma(HH_STR(__VA_ARGS__)) // C++11; http://stackoverflow.com/a/15864723
#endif

#if defined(__GNUC__) && __GNUC__*100+__GNUC_MINOR__<408
#define HH_ALIGNAS(num) __attribute__((aligned(num)))
#else
#define HH_ALIGNAS(num) alignas(num) // C++11
#endif

#if defined(_MSC_VER) && !defined(HH_NO_LIB_REFERENCES)
#define HH_REFERENCE_LIB(libstring) HH_PRAGMA(comment(lib, libstring)) // e.g.: HH_REFERENCE_LIB("user32.lib");
#else
#define HH_REFERENCE_LIB(libstring) HH_EAT_SEMICOLON
#endif

#if defined(__GNUC__)
#define HH_ATTRIBUTE(...) __attribute__((__VA_ARGS__)) // see also __declspec(x)
#else
#define HH_ATTRIBUTE(...) [[__VA_ARGS__]]   // C++11
#endif

#if defined(__GNUC__)
#define HH_PRINTF_ATTRIBUTE(...) __attribute__((__VA_ARGS__))
#else
#define HH_PRINTF_ATTRIBUTE(...)
#endif

#if defined(_MSC_VER) && !defined(__clang__)
#define HH_ASSUME(...) __assume(__VA_ARGS__) // implies __analysis_assume() but is expression rather than statement
#else
// #define HH_ASSUME(...) do { if (!(__VA_ARGS__)) __builtin_unreachable(); } while (false) // gcc
// #define HH_ASSUME(...) void(__builtin_expect(!(__VA_ARGS__), 0))   // gcc; weaker
#define HH_ASSUME(...) (void(0))
#endif

#if defined(_MSC_VER) || __GNUC__*100+__GNUC_MINOR__>=409
#define HH_HAVE_REGEX           // C++11
#endif

#if defined(_WIN32)
#define HH_NORETURN __declspec(noreturn) // it gets converted to __attribute__((noreturn)) under __GNUC__
#else
#define HH_NORETURN HH_ATTRIBUTE(noreturn)
#endif

#if defined(_MSC_VER)
#define HH_UNREACHABLE __assume(0)    // this path is never taken
#else
#define HH_UNREACHABLE __builtin_unreachable()
#endif

#if defined(__GNUC__)
#define HH_UNUSED HH_ATTRIBUTE(unused)
#else
#define HH_UNUSED
#endif

#define HH_ID(x) HH_CAT(_hh_id_, x) // private identifier in a macro definition
#define HH_UNIQUE_ID(x) HH_CAT2(HH_CAT2(HH_CAT2(_hh_id_, __COUNTER__), _), x)

#if defined(__clang__)
#define gnu_printf printf       // for use in HH_PRINTF_ATTRIBUTE(format(gnu_printf, x, y))
#endif


// *** Add some potentially missing C++11 features.

namespace std {

#if defined(HH_DEFINE_STD_ONCE)  // workaround for current clang with mingw32 4.7.2
struct once_flag { int done{0}; };
template<typename Callable, typename... Args> void call_once(std::once_flag& flag, Callable&& func, Args&&... args) {
    if (!flag.done++) func(std::forward<Args>(args)...);
}
#endif

} // namespace std


// *** Add some easy C++14 features if not already present.

#if defined(__GNUC__) && (__GNUC__*100+__GNUC_MINOR__<409 || __cplusplus<201300L) && !defined(HH_NO_DEFINE_STD_CXX14)
namespace std {
// C++14 http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2013/n3546.pdf
template<typename T> using remove_const_t = typename remove_const<T>::type;
template<typename T> using remove_volatile_t = typename remove_volatile<T>::type;
template<typename T> using remove_cv_t = typename remove_cv<T>::type;
template<typename T> using add_const_t = typename add_const<T>::type;
template<typename T> using add_volatile_t = typename add_volatile<T>::type;
template<typename T> using add_cv_t = typename add_cv<T>::type;
template<typename T> using remove_reference_t = typename remove_reference<T>::type;
template<typename T> using add_lvalue_reference_t = typename add_lvalue_reference<T>::type;
template<typename T> using add_rvalue_reference_t = typename add_rvalue_reference<T>::type;
template<typename T> using make_signed_t = typename make_signed<T>::type;
template<typename T> using make_unsigned_t = typename make_unsigned<T>::type;
template<typename T> using remove_extent_t = typename remove_extent<T>::type;
template<typename T> using remove_all_extents_t = typename remove_all_extents<T>::type;
template<typename T> using remove_pointer_t = typename remove_pointer<T>::type;
template<typename T> using add_pointer_t = typename add_pointer<T>::type;
template<typename T> using decay_t = typename decay<T>::type;
template<bool b, typename T = void>             using enable_if_t = typename enable_if<b,T>::type;
template<bool b, typename T, typename F>        using conditional_t = typename conditional<b, T, F>::type;
template<typename... T>                         using common_type_t = typename common_type<T...>::type;
template<typename T>                            using underlying_type_t = typename underlying_type<T>::type;
template<typename F, typename... ArgTypes>      using result_of_t = typename result_of<F(ArgTypes...)>::type;
// C++14
// http://stackoverflow.com/questions/13883824/make-unique-does-not-compile
// http://stackoverflow.com/questions/10149840/c-arrays-and-make-unique
template<typename T, typename... Args> std::enable_if_t<!std::is_array<T>::value, unique_ptr<T> >
make_unique(Args&&... args) {
    return unique_ptr<T>(new T(std::forward<Args>(args)...));
}
template<typename T> std::enable_if_t<std::is_array<T>::value, unique_ptr<T> > make_unique(std::size_t n) {
    using RT = std::remove_extent_t<T>;
    return unique_ptr<T>(new RT[n]);
}
} // namespace std
#endif

#if (defined(__GNUC__) && (__GNUC__*100+__GNUC_MINOR__<409 || __cplusplus<201300L)) && !defined(HH_NO_DEFINE_STD_INDEX_SEQUENCE)
namespace std {
// C++14 http://en.cppreference.com/w/cpp/utility/integer_sequence
// Useful template for creating and processing parameter packs.
// http://stackoverflow.com/questions/20966264/templated-array-of-duplicate-elements-without-default-constructor
template<std::size_t... N> struct index_sequence { };
//
// Usage: make_index_sequence<4> gives <3,3> -> <2,2,3> -> <1,1,2,3> -> <0,0,1,2,3> -> index_sequence<0,1,2,3>
template<std::size_t I, std::size_t... Is> struct make_index_sequence : make_index_sequence<I-1, I-1, Is...> { };
template<std::size_t... Is> struct make_index_sequence<0, Is...> : index_sequence<Is...> { };
// C++14 http://en.cppreference.com/w/cpp/utility/exchange
template<typename T, typename U> T exchange(T& obj, U&& new_value) {
    T old_value = std::move(obj); obj = std::forward<U>(new_value); return old_value;
}
} // namespace std
#endif


// *** Syntactic sugar

#define bcase break; case
#define ocase case
#define bdefault break; default

#define for_range_aux(T, i, lb, ub, u) for (T u = ub, i = lb; i<u; i++)
#define for_range(T, i, lb, ub) for_range_aux(T, i, lb, ub, HH_UNIQUE_ID(u))
#define for_int(i, ub)      for_range(int, i, 0, ub)
#define for_intL(i, lb, ub) for_range(int, i, lb, ub)
#define for_size_t(i, ub)   for_range(std::size_t, i, 0, ub)


// *** Begin namespace

namespace hh {


// *** Import some frequently used standard C++ names into the hh namespace.

#if !defined(HH_NO_USING_STD_STRING)
using std::string;                             // almost a new fundamental type
#endif // !defined(HH_NO_USING_STD_STRING)
using std::size_t;                             // (it seems to be already defined)
using std::unique_ptr; using std::make_unique; // very useful
using std::min; using std::max;                // very useful
using std::abs; // from <cmath>; else non-templated defined only for int from <cstdlib> (abs(1.5)==1 is scary).
// Use the following templated functions; else defined only for double (performance overhead).
using std::sqrt; using std::pow; using std::exp; using std::log; using std::log2; using std::log10;
using std::sin; using std::cos; using std::tan; using std::asin; using std::acos; using std::atan;
using std::atan2; using std::hypot;
using std::floor; using std::ceil;
// Possibly use fmod, exp2.
// Avoid fabs, fmin, fmax.

using uchar = unsigned char;
using ushort = unsigned short;

// Note: if I get compilation errors about redefinition of dummy_use(), etc. on MS VS,
//  it may be due to current directory being different from that in precompiled header due to symbol links,
//  or to an explicit HhRoot environment variable that does not match the current tree.

// Avoid warnings of unused variables
template<typename... A> void dummy_use(const A&...) { }  // constexpr in C++14

// Avoid warnings of uninitialized variables
template<typename T> void dummy_init(T& v) { v = T(); }
template<typename T, typename... A> void dummy_init(T& v, A&... a) { v = T{}; dummy_init(a...); }

// Evaluates to false in boolean context for "if (false_capture<int> i = ub) { HH_UNREACHABLE; } else" within macros.
template<typename T> struct false_capture {
    template<typename... Args> false_capture(Args&&... args) : _e(args...) { }
    operator bool() const { return false; }
    const T& operator()() const { return _e; }
    T _e;
};

// Derive from this class to disable copy constructor and copy assignment.
struct noncopyable {
 protected:
    noncopyable(const noncopyable&)                     = delete;
    noncopyable& operator=(const noncopyable&)          = delete;
    noncopyable() = default;
};

// Specialize<i> is a dummy type for function specialization.
template<int> struct Specialize { };


// *** Assertions, warnings, errors, debug

#if defined(HH_DEBUG)
constexpr bool k_debug = true;  // convenience variable to avoid introducing "#if defined(HH_DEBUG)"
#else
constexpr bool k_debug = false; // convenience variable to avoid introducing "#if defined(HH_DEBUG)"
#endif

extern int g_unoptimized_zero;  // always zero, but the compiler does not know; used to disable optimizations.

#define HH_FL " in line " HH_STR2(__LINE__) " of file " __FILE__

// Always abort.
#define assertnever(...) hh::details::assertnever_aux(hh::details::add_fl((__VA_ARGS__), HH_FL))

// Always abort; omit warning about any subsequent unreachable code.
#define assertnever_ret(...) (hh::g_unoptimized_zero ? void() :                 \
                              hh::details::assertnever_aux(hh::details::add_fl((__VA_ARGS__), HH_FL)))

// if !expr, exit program (abort); otherwise return expr.
#define assertx(...) hh::details::assertx_aux((__VA_ARGS__), "assertx(" #__VA_ARGS__ ")" HH_FL)

// If !expr, throw std::runtime_error exception; otherwise return expr.
#define assertt(...) hh::details::assertt_aux((__VA_ARGS__), "assertt(" #__VA_ARGS__ ")" HH_FL)

// If !expr, report warning once.  Returns expr.
#define assertw(...) hh::details::assertw_aux((__VA_ARGS__), "assertw(" #__VA_ARGS__ ")" HH_FL)

// Reports string expr as warning once.  Return true if this is the first time the warning is reported.
#define Warning(...) hh::details::assert_aux(false, __VA_ARGS__ HH_FL)

#if defined(HH_DEBUG)
#define ASSERTX(...) assertx(__VA_ARGS__)  // In release, do not evaluate expression
#define ASSERTXX(...) assertx(__VA_ARGS__) // In release, do not even see expression -- maximum optimization
#define HH_CHECK_BOUNDS(i, n) ((i>=0 && i<n) ? (void(0)) : assertnever(sform("bounds i=%d n=%d", i, n)))
#else
// Added "0" for clang use in constexpr
#define ASSERTX(...) ((false ? void(__VA_ARGS__) : void(0)), HH_ASSUME(__VA_ARGS__))
// The next two became necessary for VC12 optimization of Vec::operator[] in GradientDomainLoop.cpp
#define ASSERTXX(...) (void(0))
#define HH_CHECK_BOUNDS(i, n) (void(0))
#endif  // defined(HH_DEBUG)

namespace details {
HH_NORETURN void assertnever_aux(const char* s);
inline HH_NORETURN void assertnever_aux(const std::string& s) { assertnever_aux(s.c_str()); }
bool assert_aux(bool b_assertx, const char* s);
inline string add_fl(string s, const char* file_line) { return s + file_line; }
template<typename T> constexpr T assertx_aux(T&& val, const char* s) {
    return ((!val ? assertnever_aux(s) : void(0)), std::forward<T>(val));
}
} // namespace details


// *** Output functions

// showf("%s: Argument '%s' ambiguous, '%s' assumed\n", argv[0], arg.c_str(), assumed.c_str());
// os << sform(" Endprogram: %dgons %dlines\n", ngons, nlines);
// SHOW(x, y, x*y);
// SHOWP(point+vector); // show with greater precision
// SHOWL; // show current file and line

// Write formatted string to std::cerr.
HH_PRINTF_ATTRIBUTE(format(gnu_printf, 1, 2)) void showf(const char* format, ...);

// Write formatted string (prefixed with g_comment_prefix_string=="# ") to std::cerr, and to std::cout if it is a file.
HH_PRINTF_ATTRIBUTE(format(gnu_printf, 1, 2)) void showdf(const char* format, ...);

// Write formatted string (prefixed with g_comment_prefix_string=="# ") to std::cout only if it is a file.
HH_PRINTF_ATTRIBUTE(format(gnu_printf, 1, 2)) void showff(const char* format, ...);

// C-string prefix for formatted output in showdf() and showff(); default is "# ".
extern const char* g_comment_prefix_string;

// Override "char*" output to detect nullptr in C-string.
inline std::ostream& operator<<(std::ostream& os, const char* s) {
    return std::operator<<(os, s ? s : "<nullptr>");
}

// Nice formatted output of a std::pair<> of objects.
template<typename A, typename B> std::ostream& operator<<(std::ostream& os, const std::pair<A,B>& p) {
    return os << "[" << p.first << ", " << p.second << "]";
}

// By default, assume that types do not end their stream output with a newline character.
template<typename T> struct has_ostream_eol_aux { static constexpr bool value() { return false; } };

// Declares that the specified type ends its stream output with a newline character; must be placed in namespace hh.
#define HH_DECLARE_OSTREAM_EOL(...)                                                             \
    struct has_ostream_eol_aux<__VA_ARGS__> { static constexpr bool value() { return true; } }

// Identifies if a type T ends its stream output (i.e. operator<<(std::ostream)) with a newline character "\n".
template<typename T> constexpr bool has_ostream_eol() {
    // (function template cannot partially specialize, e.g. Array<T>)
    return has_ostream_eol_aux<std::remove_cv_t<std::remove_reference_t<T>>>::value();
}

// For the specified container type, define an ostream operator<<() that iterates over the container's range.
#define HH_DECLARE_OSTREAM_RANGE(...)                                         \
    std::ostream& operator<<(std::ostream& os, const __VA_ARGS__& c) {        \
        return os << hh::stream_range<decltype(c)>(c);                        \
    } HH_EAT_SEMICOLON

// With single expression, show "expr = value" and return expr (may require parentheses as in "SHOW((ntimes<3>(1)))".
// With multiple expressions, show a sequence of "expr=value" on a single line.
#define SHOW(...) HH_PRIMITIVE_CAT((HH_SHOW_, HH_HAVE_GT1_ARGS(__VA_ARGS__)))(#__VA_ARGS__, false, __VA_ARGS__)

// Show expression(s) like SHOW(expr) but with more digits of floating-point precision.
#define SHOWP(...) HH_PRIMITIVE_CAT((HH_SHOW_, HH_HAVE_GT1_ARGS(__VA_ARGS__)))(#__VA_ARGS__, true, __VA_ARGS__)

// Show current file and line number.
#define SHOWL hh::details::show_cerr_and_debug("Now in " __FILE__ " at line " HH_STR2(__LINE__) "\n")


// *** Constants

constexpr float    BIGFLOAT = 1e30f;                  // note: different from FLT_MAX or (INFINITY==HUGE_VALF)
constexpr float    TAU      = 6.2831853071795864769f; // Mathematica: N[2 Pi, 20] ;  see http://tauday.com/
constexpr double   D_TAU    = 6.2831853071795864769;  // Mathematica: N[2 Pi, 20] ;  see http://tauday.com/
// #undef PI // instead, use TAU/2
// constexpr float    PI = 3.14159265358979323846f; (==float(M_PI) if "#define _USE_MATH_DEFINES" before <cmath>)
// constexpr double D_PI = 3.14159265358979323846;  (==M_PI)


// *** Hh.cpp

// Print summaries of timers, statistics, and warnings now, rather than at exit(), perhaps before program output.
void flush_timers();
void flush_stats();
void flush_warnings();
void hh_clean_up();


#if defined(_WIN32)
// Convert Windows UTF-16 std::wstring to UTF-8 std::string (or encoding based on locale if defined(HH_NO_UTF8))
std::string narrow(const std::wstring& wstr);
// Convert UTF-8 std::string (or encoding based on locale if defined(HH_NO_UTF8)) to Windows UTF-16 std::wstring.
std::wstring widen(const std::string& str);
#endif


// e.g. SHOW(type_name<T>());
#if defined(HH_USE_RTTI) // not default; best to avoid taking dependency on run-time type information (RTTI).

namespace details {
// e.g. convert 5QueueI5PointE to Queue<Point>
string demangle_type_name(string tname);
} // namespace details
template<typename T> string type_name() { return details::demangle_type_name(typeid(T).name()); }

#else  // !defined(HH_USE_RTTI); this is best

namespace details {
string extract_function_type_name(string s);
template<typename T> struct TypeNameAux {
#if defined(__GNUC__) || defined(__clang__)
    // http://shaderop.com/2010/09/uniquely-identifying-types-in-c-without-using-rtti/
    // http://www.gamedev.net/topic/608203-c-get-type-name-without-rtti/
    // http://stackoverflow.com/questions/4384765/whats-the-difference-between-pretty-function-function-func
    static string name() { return extract_function_type_name(__PRETTY_FUNCTION__); }
#elif defined(_MSC_VER)
    static string name() { return extract_function_type_name(__FUNCTION__); } // also __FUNCSIG__, __FUNCDNAME__
#else
    static string name() { return extract_function_type_name(__func__); } // C++11 (C99); unadorned so likely to fail!
#endif
};
} // namespace details
template<typename T> string type_name() { return details::TypeNameAux<std::decay_t<T>>::name(); }

#endif


// Write an object e to its string representation using operator<<(ostream&, const T&).
template<typename T> string make_string(const T& e) { std::ostringstream oss; oss << e; return assertx(oss).str(); }

// Format a string like sprintf() but directly return an adaptively-sized std::string.
HH_PRINTF_ATTRIBUTE(format(gnu_printf, 1, 2)) string sform(const char* format, ...);

// Format a string like sform() but disable warnings concerning a non-literal format string.
string sform_nonliteral(const char* format, ...);

// Format a string like sprintf() but directly to a reused adaptively-resized output buffer named str.
HH_PRINTF_ATTRIBUTE(format(gnu_printf, 2, 3)) const string& ssform(string& str, const char* format, ...);

// Shortcut for ssform(str, format, ...).c_str(); like sprintf() but using an adaptively resized buffer str.
HH_PRINTF_ATTRIBUTE(format(gnu_printf, 2, 3)) const char* csform(string& str, const char* format, ...);

// Allocate a duplicate of a C "char*" string, using make_unique<char[]>.
unique_ptr<char[]> make_unique_c_string(const char* s);

// Convert string to integer value, or crash if invalid.
int to_int(const char* s);

// Allocate an aligned memory block (returns nullptr if fails).
void* aligned_malloc(size_t size, int alignment);

// Deallocate an aligned memory block previously created by aligned_malloc().
void aligned_free(void* p);

// Allocate n elements of type T, with appropriate memory alignment based on T.
template<typename T> T* aligned_new(size_t n) {
    return alignof(T)<=8 ? new T[n] : static_cast<T*>(aligned_malloc(n*sizeof(T), alignof(T)));
}

// Deallocate aligned memory.
template<typename T> void aligned_delete(T* p) {
    if (alignof(T)<=8) delete[] p; else aligned_free(p);
}

// Read a line of input (trailing "\n" is discarded).
std::istream& my_getline(std::istream& is, string& sline, bool dos_eol_warnings = true);

// Set an environment variable; the variable is removed from the environment if value=="".
void my_setenv(const string& varname, const string& value);

// Return false if environment variable is {undefined, "0", or "false"}, true if {"", "1", or "true"), else abort.
bool getenv_bool(const string& varname);

// Return vdefault if environment variable varname is not defined, 1 if "", value if integer, else abort.
int getenv_int(const string& varname, int vdefault = 0, bool warn = false);

// Return vdefault if environment variable varname is not defined, value if float, else abort.
float getenv_float(const string& varname, float vdefault, bool warn = false);

// Return string value of environment variable varname, or "" if not defined.
string getenv_string(const string& varname);

// Return machine name, in lowercase.
string get_hostname();

// Use fcntl() to make read() calls non-blocking; returns false if failure.
bool set_fd_no_delay(int fd, bool nodelay);

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

// Return user login name.
string get_user_name();

// Return current directory.
string get_current_directory();

// String like "2016-02-15 18:02:28".
string get_current_datetime();

// String with date, time, machine, build parameters.
string get_header_info();

// On Windows, replace the command-line argv with a new one that contains UTF-8 encoded strings; else do nothing.
void ensure_utf8_encoding(int& argc, const char**& argv);


// *** Inlines

// Returns T{-1} or T{+1} based on sign of expression.
template<typename T> constexpr T sign(const T& e)        { return e>=T{0} ? T{1} : T{-1}; }

// Returns { T{-1}, T{0}, T{+1} } based on sign of expression.
template<typename T> constexpr T signz(const T& e)       { return e>T{0} ? T{1} : e<T{0} ? T{-1} : T{0}; }

// Returns a value times itself.
template<typename T> constexpr T square(const T& e)      { return e*e; }

// Returns v clamped to the range [a, b].
template<typename T> constexpr T clamp(const T& v, const T& a, const T& b) {
    return (ASSERTXX(!(v<a && v>b)), v<a ? a : v>b ? b : v);
}

// Like clamp() but slightly less efficient; however works on values like Vector4.  (min() is constexpr in C++14)
template<typename T> T general_clamp(const T& v, const T& a, const T& b) { return min(max(v, a), b); }

// Linearly interpolate between two values (f==1.f returns v1; f==0.f returns v2).
inline float interp(float v1, float v2, float f = 0.5f) { return f*v1 + (1.f-f)*v2; } // or v2 + (v1-v2)*f

// Linearly interpolate between two values (f==1. returns v1; f==0. returns v2).
inline double interp(double v1, double v2, double f = 0.5) { return f*v1 + (1.-f)*v2; } // or v2 + (v1-v2)*f

// Returns v clamped to range [0, 255].
inline uchar clamp_to_uchar(int v) {
    // return clamp(v, 0, 255);
    // http://codereview.stackexchange.com/questions/6502/fastest-way-to-clamp-an-integer-to-the-range-0-255
    v &= -(v>=0); return static_cast<uchar>(v | ((255-v)>>31));
}

// Returns j%3 (where j is in [0, 5]).
inline int mod3(int j) {
    static const int ar_mod3[6] = { 0, 1, 2, 0, 1, 2 };
    ASSERTX(j>=0 && j<6); return ar_mod3[j];
}

// Rounds floating-point value v to the nearest 1/fac increment (by default fac==1e5f).
template<typename T> T round_fraction_digits(T v, T fac = 1e5f) { return floor(v*fac+.5f)/fac; }

// Zero-out the variable e; this function is specialized for float, double, Vector4, Vector4i.
template<typename T> void my_zero(T& e) {
    // e = T{}; // bad because default constructor can leave object uninitialized, e.g. Vector, Vec<T>, Vector4
    static constexpr T k_dummy_zero_object{};
    e = k_dummy_zero_object;
}
template<> inline void my_zero(float& e) { e = 0.f; }
template<> inline void my_zero(double& e) { e = 0.; }

namespace details {
template<typename T> struct identity { using type = T; };
}  // namespace details
// For use in upcasting to a base class, converting nullptr, or declaring type in ternary operand.
template<typename Dest> Dest implicit_cast(typename details::identity<Dest>::type t) { return t; }

// For use in downcasting to a derived class.
template<typename Dest, typename Src> Dest down_cast(Src* f) {
    static_assert(std::is_base_of<Src, typename std::remove_pointer<Dest>::type>::value, "");
    return static_cast<Dest>(f);
}

// Conversion with bounds-checking in Debug configuration.
template<typename Target, typename Source> constexpr Target narrow_cast(Source v) {
#if defined(_MSC_VER) && _MSC_VER<1910
#pragma warning(push)
#pragma warning(disable:4800) // C4800: 'int' : forcing value to bool 'true' or 'false' (performance warning)
#pragma warning(disable:6319) // Use of the comma-operator in a tested expression causes the left argument to be ignored when it has no side-effects.
#endif
    // auto r = static_cast<Target>(v); ASSERTXX(static_cast<Source>(r)==v); return r;
    return (ASSERTXX(static_cast<Source>(static_cast<Target>(v))==v), static_cast<Target>(v));
#if defined(_MSC_VER) && _MSC_VER<1910
#pragma warning(pop)
#endif
}

// Bounds-safe conversion, checked even in Release configuration.
template<typename Target, typename Source> constexpr Target assert_narrow_cast(Source v) {
#if defined(_MSC_VER) && _MSC_VER<1910
#pragma warning(suppress:4800) // C4800: 'int' : forcing value to bool 'true' or 'false' (performance warning)
#endif
    return (assertx(static_cast<Source>(static_cast<Target>(v))==v), static_cast<Target>(v));
}

// Type conversion, but avoiding a warning in the case that Source is already of the same type as Target.
template<typename Target, typename Source> constexpr Target possible_cast(Source v) {
    return static_cast<Target>(v);
}

// Higher-precision type to represent the sum of a set of elements.
template<typename T> struct sum_type {
    using type = std::conditional_t<!std::is_arithmetic<T>::value, T,
                                    std::conditional_t<std::is_floating_point<T>::value, double,
                                                       std::conditional_t<std::is_signed<T>::value, int64_t,
                                                                          uint64_t> > >;
};
template<typename T> using sum_type_t = typename sum_type<T>::type;

namespace details {
// Range of integral elements defined as in Python range(lb, ub), where step is 1 and ub is not included.
template<typename T> class Range {
    static_assert(std::is_integral<T>::value, ""); // must have exact arithmetic for equality testing
    class Iterator {
        using type = Iterator;
     public:
        using value_type = T;
        using reference = T&;
        using pointer = T*;
        using difference_type = int64_t;
        using iterator_category = std::random_access_iterator_tag;
        Iterator(T lb, T ub)                    : _v(lb), _ub(ub) { }
        Iterator(const type& iter)              = default;
        bool operator==(const type& rhs) const  { return _v==rhs._v; }
        bool operator!=(const type& rhs) const  { return !(*this==rhs); }
        difference_type operator-(const type& rhs) const { return _v - rhs._v; }
        type& operator+=(difference_type n)     { ASSERTXX(_v<_ub); _v += n; ASSERTXX(_v<=_ub); return *this; }
        T operator*() const                     { ASSERTXX(_v<_ub); return _v; }
        type& operator++()                      { ASSERTXX(_v<_ub); _v += T{1}; return *this; }
     private:
        T _v, _ub;
    };
 public:
    using value_type = T;
    using iterator = Iterator;
    using const_iterator = Iterator;
    using size_type = size_t;
    explicit Range(T ub)                        : Range(T{0}, ub) { }
    Range(T lb, T ub)                           : _lb(min(lb, ub)), _ub(ub) { }
    Iterator begin() const                      { return Iterator(_lb, _ub); }
    Iterator end() const                        { return Iterator(_ub, _ub); }
    Iterator cbegin() const                     { return Iterator(_lb, _ub); }
    Iterator cend() const                       { return Iterator(_ub, _ub); }
    size_t size() const                         { return _ub-_lb; }
    bool empty() const                          { return _ub==_lb; }
 private:
    T _lb, _ub;
};
} // namespace details

// Range of integral elements as in Python range(ub):  e.g.: for (int i : range(5)) { SHOW(i); } gives 0..4 .
template<typename T> details::Range<T> range(T ub) { return details::Range<T>(ub); }

// Range of integral elements as in Python range(lb, ub):  e.g.: for (int i : range(2, 5)) { SHOW(i); } gives 2..4 .
template<typename T> details::Range<T> range(T lb, T ub) { return details::Range<T>(lb, ub); }


//----------------------------------------------------------------------------

// HH_REFERENCE_LIB("libHh.lib");

namespace details {

#if !defined(HH_NO_HH_INIT)
int hh_init();
static HH_UNUSED int dummy_init_hh = hh_init();
#endif

void show_cerr_and_debug(const string& s);

template<typename T> T assertt_aux(T&& val, const char* s) {
    if (!val) throw std::runtime_error(s);
    return std::forward<T>(val);
}

template<typename T> T assertw_aux(T&& val, const char* s) {
    return ((!val ? void(hh::details::assert_aux(false, s)) : void(0)), std::forward<T>(val));
}

#define HH_SHOW_0(sargs, prec, arg1) hh::details::SHOW_aux(sargs, arg1, hh::has_ostream_eol<decltype(arg1)>(), prec)
#define HH_SHOW_1(sargs, prec, ...) do { std::ostringstream HH_ID(oss);            \
        if (prec) HH_ID(oss).precision(std::numeric_limits<double>::max_digits10); \
        HH_ID(oss) << HH_MAP_REDUCE((HH_SHOWM_,  << " " <<,  __VA_ARGS__)) << "\n"; \
        hh::details::show_cerr_and_debug(assertx(HH_ID(oss)).str()); } while (false)
#define HH_SHOWM_(x) (#x[0]=='"' ? "" : #x "=") << (x)

template<typename T> T SHOW_aux(string str, T&& val, bool has_eol, bool high_precision) {
    std::ostringstream oss;
    if (high_precision) oss.precision(std::numeric_limits<double>::max_digits10);
    oss << (str[0]=='"' ? "" : str + " = ") << val << (has_eol ? "" : "\n");
    // Single write for better behavior under parallelism.  For guaranteed atomic write, use HH_LOCK { SHOW(...); }
    show_cerr_and_debug(assertx(oss).str());
    return std::forward<T>(val);
}

} // namespace details

// User-defined ostream manipulator, which is constructed from a container and writes out its elements.
template<typename C> class stream_range {
 public:
    stream_range(const C& c) : _c(c) { }
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

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_HH_H_
