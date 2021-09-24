// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Hh_init.h"

#include <fcntl.h>  // O_BINARY

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <io.h>  // _setmode()
#endif

#include <csignal>  // signal()
#include <cstring>  // std::strerror
#include <mutex>    // std::once_flag, std::call_once()
#include <new>      // set_new_handler()

#include "Hh.h"

#if !defined(HH_NO_INIT)

namespace hh {

namespace {

HH_NORETURN void my_new_handler() { assertnever("new is out of memory"); }

#if defined(_WIN32)
void possibly_sleep() {
  if (getenv_bool("ASSERT_SLEEP")) {
    std::cerr << "Now doing my_sleep(60.)\n";
    my_sleep(60.);
    // Give myself time to enter in a different window:
    //  vsjitdebugger -p `ps | grep FilterPM | perl -ane 'print "$F[0]\n"; last;'`
    // or open Visual Studio and then Debug -> Attach to Process.
  }
}
#endif

#if defined(_WIN32)

// For SetUnhandledExceptionFilter(my_top_level_exception_filter);
// Note: This custom filter for unhandled exceptions is not called when running under debugger.
//  See http://www.debuginfo.com/articles/debugfilters.html
//  The solution is to enable show_message_box, and attach the debugger to debug.
//
// long __stdcall my_top_level_exception_filter(_EXCEPTION_POINTERS* ExceptionInfo) {
LONG WINAPI my_top_level_exception_filter(EXCEPTION_POINTERS* ExceptionInfo) {
  if (0) std::cerr << "my_top_level_exception_filter\n";
    // Unfortunately, STACK_OVERFLOW does not seem to reach here in _WIN32 DEBUG mode.
#if 0
  if (getenv_bool("ASSERTX_MESSAGE_BOX"))
    MessageBox(nullptr, "Attach debugger, set breakpoint in my_top_level_exception_filter, then close this box",
               "Hh.cpp", MB_OK);
  HH_REFERENCE_LIB("user32.lib");  // MessageBoxA()
#endif
  const unsigned int MSFT_CPP_EXCEPT = 0xE06d7363;  // c++ exception
  unsigned ExceptionCode = ExceptionInfo->ExceptionRecord->ExceptionCode;
  if (0) SHOW("have", ExceptionCode);
  switch (ExceptionCode) {
#define E(x) \
  case EXCEPTION_##x: std::cerr << "Exception error: " #x "\n"; break
    E(ACCESS_VIOLATION);
    E(DATATYPE_MISALIGNMENT);
    // E(BREAKPOINT);
    E(SINGLE_STEP);
    E(ARRAY_BOUNDS_EXCEEDED);
    E(FLT_DENORMAL_OPERAND);
    E(FLT_DIVIDE_BY_ZERO);
    E(FLT_INEXACT_RESULT);
    E(FLT_INVALID_OPERATION);
    E(FLT_OVERFLOW);
    E(FLT_STACK_CHECK);
    E(FLT_UNDERFLOW);
    E(INT_DIVIDE_BY_ZERO);
    E(INT_OVERFLOW);
    E(PRIV_INSTRUCTION);
    E(IN_PAGE_ERROR);
    E(ILLEGAL_INSTRUCTION);
    E(NONCONTINUABLE_EXCEPTION);
    E(STACK_OVERFLOW);
    E(INVALID_DISPOSITION);
    E(GUARD_PAGE);
    E(INVALID_HANDLE);
#undef E
    case EXCEPTION_BREAKPOINT:
      // No need to show a message since an assertion error was likely already reported.
      break;
    case MSFT_CPP_EXCEPT: {  // uncaught c++ exception
      EXCEPTION_RECORD& er = *ExceptionInfo->ExceptionRecord;
      // If this crashes, it may be best to delay until after show_call_stack() below.
      const std::runtime_error& ex = *reinterpret_cast<std::runtime_error*>(er.ExceptionInformation[1]);
      std::cerr << "Uncaught c++ exception: " << ex.what() << "\n";
      break;
    }
    default: SHOW("Unrecognized exception code", ExceptionCode);
  }
  if (errno) std::cerr << "possible error: " << std::strerror(errno) << "\n";
  show_possible_win32_error();
  // want to report assertion errors from C++ standard library (dialog box pops up, and reach here on "Retry")
  if (k_debug && ExceptionCode == EXCEPTION_BREAKPOINT && !IsDebuggerPresent()) {
    std::cerr << "EXCEPTION_BREAKPOINT in Debug version without debugger present\n";
    show_call_stack();
    // return EXCEPTION_CONTINUE_EXECUTION;
    if (getenv_int("ASSERTX_ABORT")) {
      SHOWL;
      abort();
    }
    exit_immediately(1);
  }
  if (ExceptionCode != EXCEPTION_BREAKPOINT) {  // otherwise we have already shown the call stack previously
    if (1) {
      // Sometimes does not show any useful information
      // Bad case: Filterimage image1.png -filter i -boundaryrule r -resamplemesh 128_mesh2_v1.m -noo
      // This also fails on win32 (32-bit).
      show_call_stack();
    }
  }
  if (!k_debug) exit_immediately(1);
  return EXCEPTION_CONTINUE_SEARCH;  // or EXCEPTION_EXECUTE_HANDLER, EXCEPTION_CONTINUE_EXECUTION
}

#endif  // defined(_WIN32)

HH_NORETURN void my_terminate_handler() {
  // The function shall not return and shall terminate the program.
  // Here, trust stderr more than std::cerr.
  if (0) {
    fprintf(stderr, "my_terminate_handler\n");
    fflush(stderr);
  }
#if defined(__GNUC__) || defined(__clang__)
  // http://stackoverflow.com/questions/3774316/c-unhandled-exceptions
  // http://stackoverflow.com/questions/17258733/how-to-customize-uncaught-exception-termination-behavior
  // This works on mingw, clang.
  // On cygwin, my_terminate_handler() is never called (bug).  http://stackoverflow.com/questions/24402412
  // On win, SetUnhandledExceptionFilter(my_top_level_exception_filter) is called instead.
  try {
    throw;
  } catch (const std::exception& ex) {
    fprintf(stderr, "Terminate: Uncaught c++ exception: %s\n", ex.what());
    fflush(stderr);
  } catch (...) {
  }
#endif
  if (errno) std::cerr << "possible error: " << std::strerror(errno) << "\n";
  show_possible_win32_error();
  assertnever("my_terminate_handler");
}

HH_NORETURN void my_abort_handler(int signal_num) {
  dummy_use(signal_num);
#if defined(_MSC_VER) || defined(__MINGW32__)
  if (1) {
    show_call_stack();
  }
#endif
#if defined(_WIN32)
  if (IsDebuggerPresent()) {
    DebugBreak();
  }
  possibly_sleep();
#else
  bool want_abort = getenv_bool("ASSERT_ABORT") || getenv_bool("ASSERTX_ABORT");
  if (want_abort) {
    std::cerr << "Signaling true abort\n";
    signal(SIGABRT, SIG_IGN);
    abort();
  }
#endif
  exit_immediately(1);
}

// #include <execinfo.h> // backtrace()
HH_NORETURN void my_signal_handler(int signal_num) {
  // Avoid issuing low-level or STDIO.H I/O routines (such as printf and fread).
  // Avoid heap routines or any routine that uses the heap routines (such as malloc, strdup, putenv).
  // Avoid any function that generates a system call (e.g., getcwd(), time()).
  // exit_immediately(213);  // does not show up at all.
  SHOW("my_signal_handler", signal_num);
  if (0) {
    // http://stackoverflow.com/questions/77005/how-to-generate-a-stacktrace-when-my-gcc-c-app-crashes
    // unfortunately, cygwin does not have backtrace/execinfo:  http://comments.gmane.org/gmane.os.cygwin/91196
    // void *ar[10]; size_t size = backtrace(ar, 10);  // get void*'s for all entries on the stack
    // backtrace_symbols_fd(ar, size, 2);  // print out all the frames to stderr
    //
    // mingw uses the Windows calling stack, so obviously incompatible with libgcc backtrace()
  }
  assertnever("my_signal_handler");
}

#if defined(_MSC_VER)
int __cdecl my_CrtDbgHook(int nReportType, char* szMsg, int* pnRet) {
  // The heap may be corrupt, so cannot do any dynamic allocation here.
  std::cerr << "my_CrtDbgHook with no debugger present\n";
  std::cerr << "Failure message: " << szMsg << "\n";
  if (1) {
    // I used to often get "Segmentation fault" due to corrupt heap,
    //  but now StackWalker has its own local memory buffer.
    show_call_stack();
  }
  std::cerr << "Now after show_call_stack()\n";
  if (0) std::cerr << "nReportType=" << nReportType << "\n";
  dummy_use(pnRet);
  possibly_sleep();
  if (0) assertnever("my_CrtDbgHook with !IsDebuggerPresent()");
  exit_immediately(1);
  // return 0;  // commented because exit_immediately() does not return
  // Return true - {Abort, Retry, Ignore} dialog will *not* be displayed
  // Return false - {Abort, Retry, Ignore} dialog *will* be displayed
}
#endif

void assign_my_signal_handler() {
  // using SignalHandlerPointer = void (*)(int);
  // c:/cygwin/usr/include/sys/signal.h
  // This does not seem to work under CYGWIN
  // And it does not seem to work under WIN32; cannot catch "Segmentation fault" $status=140
  //
  // The SIGILL, SIGSEGV, and SIGTERM signals are not generated under Windows NT. They are included
  // for ANSI compatibility. ... you can also explicitly generate these signals by calling raise.
  SHOWL;
  // signal(SIGFPE, my_signal_handler);
  signal(SIGILL, my_signal_handler);
  signal(SIGSEGV, my_signal_handler);
  // signal(SIGTRAP, my_signal_handler);
}

void setup_exception_hooks() {
  dummy_use(show_call_stack, my_new_handler, my_terminate_handler, my_signal_handler);
#if defined(_WIN32)
  dummy_use(my_top_level_exception_filter);
#endif
#if !defined(HH_NO_EXCEPTION_HOOKS)
  if (getenv_bool("HH_NO_EXCEPTION_HOOKS")) return;
#if defined(__CYGWIN__)
  std::set_new_handler(&my_new_handler);  // the default behavior is to throw std::bad_alloc
                                          // else on Cygwin, no diagnostic is reported (other than nonzero exit code)
#endif
#if defined(_MSC_VER)
  if (!IsDebuggerPresent()) {
    // Because the "Just-in-time debugging" no longer seems to work.
    _CrtSetReportHook2(_CRT_RPTHOOK_INSTALL, my_CrtDbgHook);  // only in Debug
    dummy_use(&my_CrtDbgHook);                                // otherwise unreferenced in Release
  }
#endif
#if defined(_WIN32)
  if (1) {
    // SEM_FAILCRITICALERRORS      0x0001
    // unsigned v = SetErrorMode(SEM_FAILCRITICALERRORS); dummy_use(v);
    // assertx(v == SEM_FAILCRITICALERRORS);  // already default!
    if (1) SetErrorMode(SetErrorMode(0) | SEM_FAILCRITICALERRORS);
    // It is not the default for Windows apps?
    // Also consider from http://stackoverflow.com/a/467652/1190077 :
    if (1) SetErrorMode(SetErrorMode(0) | SEM_NOGPFAULTERRORBOX);  // yes, useful e.g. for mingw32
  }
  if (1) {
    // LPTOP_LEVEL_EXCEPTION_FILTER WINAPI SetUnhandledExceptionFilter(
    //   _In_  LPTOP_LEVEL_EXCEPTION_FILTER lpTopLevelExceptionFilter);
    LPTOP_LEVEL_EXCEPTION_FILTER v = SetUnhandledExceptionFilter(my_top_level_exception_filter);
    dummy_use(v);
    if (0) SHOW(reinterpret_cast<size_t>(v));
    // 0x0041DE18 -- already is an exception filter.  what did it do?
  }
#endif
  if (1) std::set_terminate(my_terminate_handler);
  if (1) signal(SIGABRT, my_abort_handler);
  if (0) assign_my_signal_handler();
#endif  // !defined(HH_NO_EXCEPTION_HOOKS)
}

void use_standard_exponent_format_in_io() {
  // Use standard format for float/double printf("%e"): 2 instead of 3 digits for exponent if possible.
#if defined(__MINGW32__)
  // For mingw32, using _set_output_format requires modifying __MSVCRT_VERSION__ on
  //   *all* compiled files (libpng.a etc.), and then getting a dependency on msvcr100.dll .
  // Instead, the my_setenv() works even with the old standard msvcrt.dll .
  my_setenv("PRINTF_EXPONENT_DIGITS", "2");
#endif
}

void use_binary_io() {
#if defined(_WIN32)
  _fmode = O_BINARY;  // <stdlib.h>; same as: assertx(!_set_fmode(O_BINARY));
  assertx(_setmode(0, O_BINARY) >= 0);
  assertx(_setmode(1, O_BINARY) >= 0);
  assertx(_setmode(2, O_BINARY) >= 0);
  // There is no global variable for default iostream binary mode.
  // With new iostream, it appears that std::cin, std::cout, std::cerr adjust
  //  (fortunately) to the settings of stdin, stdout, stderr.
#endif
}

void unsynchronize_stream_and_stdio() {
  // We expect no interleaved unbuffered use of iostream and FILE stdio.
  std::ios_base::sync_with_stdio(false);
}

void untie_cin_and_cout() {
  // We expect no use of std::cin without first explicitly flushing any unbuffered std::cout.
  std::cin.tie(nullptr);
}

void warn_if_running_debug_version() {
#if defined(_MSC_VER)
  if (k_debug) showf("Running debug version.\n");
#endif
}

void change_default_io_precision() {
  // Change default precision to 8 digits to approximate single-precision float numbers "almost" exactly.
  // Verify that the precision was unchanged from its default value of 6.
  //  (similar code both in Hh.cpp and FileIO.cpp)
  assertx(std::cout.precision(8) == 6);
  assertx(std::cerr.precision(8) == 6);
  // This is an interesting idea.
  // However, it results in many nice round numbers like "0.18" being approximated by many digits,
  //  such as "0.18000001", which just looks ugly.  Similarly "0.090000004" and "0.059999999".
  // This is likely the motivation for the default precision of 6.
  //  == std::numeric_limits<T>::digits10, the number of digits reliably encoded/decoded as float.
  //
  // See experiments in ~/git/hh_src/test/misc/test_float_discrepancy.cpp
  //
  // See my answer at http://stackoverflow.com/a/23437425/1190077
  //  to the question of the precision necessary to exactly save and retrieve floating-point numbers :
  //
  // See the nice detailed discussion in
  //  http://randomascii.wordpress.com/2012/03/08/float-precisionfrom-zero-to-100-digits-2/
  //
  // The short answer is that the minimum precision is the following:
  //
  // printf("%1.8e", d);   // Round-trippable float, always with an exponent
  // printf("%.9g", d);    // Round-trippable float, shortest possible
  // printf("%1.16e", d);  // Round-trippable double, always with an exponent
  // printf("%.17g", d);   // Round-trippable double, shortest possible
  //
  // Or equivalently, with a std::ostream& os:
  //
  // os << scientific << setprecision(8) << d;     // float; always with an exponent
  // os << defaultfloat << setprecision(9) << d;   // float; shortest possible
  // os << scientific << setprecision(16) << d;    // double; always with an exponent
  // os << defaultfloat << setprecision(17) << d;  // double; shortest possible
  //
  // os << defaultfloat << setprecision(std::numeric_limits<T>::max_digits10) << d;  // 9 or 17
  //
  // I feel that the difference between 8 and 9 digits of precision (mainly for numbers between 1000 and 1023)
  //  is not worth it.
  //
  // See also:
  //  http://stackoverflow.com/questions/10357192/printf-rounding-behavior-for-doubles
  //  http://www.exploringbinary.com/inconsistent-rounding-of-printed-floating-point-numbers/
  // Visual C++ uses the round-half-away-from-zero rule,
  //  and gcc (actually, glibc) uses the round-half-to-even rule, also known as bankers' rounding.
  // glibc printf() has been updated to take the current IEEE rounding mode into account. This was done
  //  in version 2.17; I just tested it on version 2.18. In doing it this way of course,
  //  round-to-nearest/round-half-away-from-zero is still not an option, so this doesn't help you make its
  //  output consistent with other platforms.
  //
  // MinGW gcc uses the Microsoft C runtime, so it's not really going to show any different results than
  //  MSVC as far as printf() is concerned.  (It may be using an older version of the runtime.)
}

void exercise_errors() {
  SHOWL;
  if (0) {
    float b = 1.f / float(g_unoptimized_zero);  // silently produces infinity
    SHOW(b);
  }
  if (0) {
    int a = 123456789;
    int b = a * a;  // silently overflows
    SHOW(b);
  }
  if (0) {
    // *implicit_cast<int*>(nullptr) = 1;  // error: access violation; CYGWIN+release just crashes
  }
  if (0) {
    throw 0;  // unhandled exception
  }
  if (0) {
    int b = 1 / g_unoptimized_zero;  // error: integer division by zero
    SHOW(b);
  }
  exit(0);
}

void hh_init_aux() {
  setup_exception_hooks();
  use_standard_exponent_format_in_io();
  use_binary_io();
  unsynchronize_stream_and_stdio();
  untie_cin_and_cout();
  warn_if_running_debug_version();
  if (0) change_default_io_precision();
  if (0) exercise_errors();
}

}  // namespace

namespace details {
// ret: bogus integer
int hh_init() {
  if (0) {
    // This fails intermittently in "reverselines" on Unix gcc 4.8.2;
    //  it is likely not intended for use in static initialization (of dummy_init_hh).
    static std::once_flag flag;
    std::call_once(flag, hh_init_aux);
  } else {
    static bool is_init = false;
    if (!is_init) {
      is_init = true;
      hh_init_aux();
    }
  }
  return 0;
}
}  // namespace details

}  // namespace hh

#endif  // !defined(HH_NO_INIT)
