// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Hh_init.h"

#include <fcntl.h>  // O_BINARY

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <io.h>  // _setmode()
#endif

#include <array>
#include <clocale>  // setlocale()
#include <csignal>  // signal()
#include <cstring>  // strerror
#include <mutex>    // once_flag, call_once()
#include <new>      // set_new_handler()

#include "Hh.h"

#if !defined(HH_NO_INIT)

namespace hh {

namespace {

[[noreturn]] void my_new_handler() { assertnever("new is out of memory"); }

#if defined(_WIN32)
void possibly_sleep() {
  if (getenv_bool("ASSERT_SLEEP")) {
    std::cerr << "Now sleeping 60 s.\n";
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
//  See https://www.debuginfo.com/articles/debugfilters.html
//  The solution is to enable show_message_box, and attach the debugger to debug.
//
// long __stdcall my_top_level_exception_filter(_EXCEPTION_POINTERS* ExceptionInfo) {
LONG WINAPI my_top_level_exception_filter(EXCEPTION_POINTERS* ExceptionInfo) {
  if (0) std::cerr << "my_top_level_exception_filter\n";
  // Unfortunately, STACK_OVERFLOW does not seem to reach here in _WIN32 DEBUG mode.
#if 0
  if (getenv_bool("ASSERTX_MESSAGE_BOX"))
    MessageBoxA(nullptr, "Attach debugger, set breakpoint in my_top_level_exception_filter, then close this box",
                "Hh.cpp", MB_OK);
  HH_REFERENCE_LIB("user32.lib");  // MessageBoxA()
#endif
  const unsigned int MSFT_CPP_EXCEPT = 0xE06d7363;  // C++ exception.
  unsigned ExceptionCode = ExceptionInfo->ExceptionRecord->ExceptionCode;
  if (0) SHOW("have", ExceptionCode);
  switch (ExceptionCode) {
#define E(x) \
  case EXCEPTION_##x: std::cerr << "Fatal exception error: " #x "\n"; break
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
    case MSFT_CPP_EXCEPT: {  // Uncaught C++ exception.
      EXCEPTION_RECORD& er = *ExceptionInfo->ExceptionRecord;
      // If this crashes, it may be best to delay until after show_call_stack() below.
      const std::runtime_error& ex = *reinterpret_cast<std::runtime_error*>(er.ExceptionInformation[1]);
      std::cerr << "Fatal uncaught C++ exception: " << ex.what() << "\n";
      break;
    }
    default: SHOW("Unrecognized exception code", ExceptionCode);
  }
  if (errno) std::cerr << "possible error: " << std::strerror(errno) << "\n";
  show_possible_win32_error();
  // We want to report assertion errors from C++ standard library (dialog box pops up, and reach here on "Retry").
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
  if (ExceptionCode != EXCEPTION_BREAKPOINT) {  // Otherwise we have already shown the call stack previously.
    show_call_stack();
    exit_immediately(1);
  }
  if (!k_debug) exit_immediately(1);
  return EXCEPTION_CONTINUE_SEARCH;  // Or EXCEPTION_EXECUTE_HANDLER, EXCEPTION_CONTINUE_EXECUTION.
}

#endif  // defined(_WIN32)

[[noreturn]] void my_terminate_handler() {
  // The function shall not return and shall terminate the program.
  // Here, trust stderr more than std::cerr.
  if (0) {
    fprintf(stderr, "my_terminate_handler\n");
    fflush(stderr);
  }
#if defined(__GNUC__) || defined(__clang__)
  // https://stackoverflow.com/questions/3774316/c-unhandled-exceptions
  // https://stackoverflow.com/questions/17258733/how-to-customize-uncaught-exception-termination-behavior
  // This works on mingw, clang.
  // On cygwin, my_terminate_handler() is never called (bug).  https://stackoverflow.com/questions/24402412
  // On win, SetUnhandledExceptionFilter(my_top_level_exception_filter) is called instead.
  try {
    throw;
  } catch (const std::exception& ex) {
    fprintf(stderr, "Fatal uncaught C++ exception: %s\n", ex.what());
    fflush(stderr);
  } catch (...) {
  }
#endif
  if (errno) std::cerr << "possible error: " << std::strerror(errno) << "\n";
  show_possible_win32_error();
  abort();
}

[[noreturn]] void my_abort_handler(int signal_num) {
  dummy_use(signal_num);
  show_call_stack();
#if defined(_WIN32)
  if (IsDebuggerPresent()) DebugBreak();
  possibly_sleep();
#else
  bool want_abort = getenv_bool("ASSERT_ABORT") || getenv_bool("ASSERTX_ABORT");
  if (want_abort) {
    std::cerr << "Signaling true abort\n";
    signal(SIGABRT, SIG_DFL);
    abort();
  }
#endif
  exit_immediately(1);
}

#if !defined(_WIN32)

// Note: the functions called here are not async-signal-safe, but the process is terminating anyway.
void my_signal_handler(int signal_num, siginfo_t* info, void* context) {
  dummy_use(info, context);
  const char* name = (signal_num == SIGSEGV  ? "SIGSEGV"
                      : signal_num == SIGBUS ? "SIGBUS"
                      : signal_num == SIGILL ? "SIGILL"
                      : signal_num == SIGFPE ? "SIGFPE"
                                             : "signal");
  std::cerr << "Fatal signal error: " << name << "\n";
  show_call_stack();
  if (getenv_bool("ASSERT_ABORT") || getenv_bool("ASSERTX_ABORT")) {
    std::cerr << "Signaling true abort\n";
    return;  // Re-execute the faulting instruction; SA_RESETHAND then gives the default action (core dump).
  }
  exit_immediately(1);
}

void assign_my_signal_handler() {
  // An alternate stack lets the handler run even when the fault is a stack overflow.
  // Its size is a constant because SIGSTKSZ is no longer a compile-time constant in recent glibc.
  static std::array<char, 64 * 1024> alt_stack_buffer;
  stack_t alt_stack{};
  alt_stack.ss_sp = alt_stack_buffer.data();
  alt_stack.ss_size = alt_stack_buffer.size();
  assertx(sigaltstack(&alt_stack, nullptr) == 0);
  struct sigaction action{};
  action.sa_sigaction = my_signal_handler;
  action.sa_flags = SA_ONSTACK | SA_SIGINFO | SA_RESETHAND;
  assertx(sigemptyset(&action.sa_mask) == 0);
  for (int signal_num : {SIGSEGV, SIGBUS, SIGILL, SIGFPE}) assertx(sigaction(signal_num, &action, nullptr) == 0);
}

#endif

#if defined(_MSC_VER)
int __cdecl my_CrtDbgHook(int nReportType, char* szMsg, int* pnRet) {
  // The heap may be corrupt, so cannot do any dynamic allocation here.
  if (0) std::cerr << "my_CrtDbgHook with no debugger present\n";
  if (strcmp(szMsg, "abort() has been called")) std::cerr << "No debugger present; failure message: " << szMsg << "\n";
  show_call_stack();
  if (0) std::cerr << "Now after show_call_stack()\n";
  if (0) std::cerr << "nReportType=" << nReportType << "\n";
  dummy_use(pnRet);
  possibly_sleep();
  if (0) assertnever("my_CrtDbgHook with !IsDebuggerPresent()");
  exit_immediately(1);
  // return 0;  // Commented because exit_immediately() does not return.
  // Return true - {Abort, Retry, Ignore} dialog will *not* be displayed.
  // Return false - {Abort, Retry, Ignore} dialog *will* be displayed.
}
#endif

void setup_exception_hooks() {
  dummy_use(my_new_handler, my_terminate_handler);
#if defined(_WIN32)
  dummy_use(my_top_level_exception_filter);
#endif
#if !defined(HH_NO_EXCEPTION_HOOKS)
  if (getenv_bool("HH_NO_EXCEPTION_HOOKS")) return;
#if !defined(_WIN32)
  assign_my_signal_handler();
#endif
#if defined(__CYGWIN__)
  // The default behavior is to throw std::bad_alloc
  std::set_new_handler(&my_new_handler);  // Else on Cygwin, no diagnostic is reported (other than nonzero exit code).
#endif
#if defined(_MSC_VER)
  if (!IsDebuggerPresent()) {
    // Because the "Just-in-time debugging" no longer seems to work.
    _CrtSetReportHook2(_CRT_RPTHOOK_INSTALL, my_CrtDbgHook);  // Only in Debug.
    dummy_use(my_CrtDbgHook);                                 // Otherwise unreferenced in Release.
  }
#endif
#if defined(_WIN32)
  if (1) {
    // Pin the symbol path so DbgEng cannot stall on a network symbol server during a crash.
    SetEnvironmentVariableA("_NT_SYMBOL_PATH", "");
    // SEM_FAILCRITICALERRORS      0x0001
    // unsigned v = SetErrorMode(SEM_FAILCRITICALERRORS); dummy_use(v);
    // assertx(v == SEM_FAILCRITICALERRORS);  // already default!
    SetErrorMode(SetErrorMode(0) | SEM_FAILCRITICALERRORS);
    // It is not the default for Windows apps?
    // Also consider from https://stackoverflow.com/a/467652 :
    SetErrorMode(SetErrorMode(0) | SEM_NOGPFAULTERRORBOX);  // Yes, useful e.g. for mingw32.
  }
  if (1) {
    LPTOP_LEVEL_EXCEPTION_FILTER v = SetUnhandledExceptionFilter(my_top_level_exception_filter);
    dummy_use(v);
    if (0) SHOW(reinterpret_cast<size_t>(v));
    // 0x0041DE18 -- already is an exception filter.  what did it do?
  }
#endif
  std::set_terminate(my_terminate_handler);
  signal(SIGABRT, my_abort_handler);
#endif  // !defined(HH_NO_EXCEPTION_HOOKS)
}

void use_standard_exponent_format_in_io() {
  // Use standard format for float/double printf("%e"): 2 instead of 3 digits for exponent if possible.
#if defined(__MINGW32__)
  // For mingw, using _set_output_format requires modifying __MSVCRT_VERSION__ on
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
  // There is no global variable for default iostream binary mode.  With new iostream, it appears that
  //  std::cin, std::cout, std::cerr adjust (fortunately) to the settings of stdin, stdout, stderr.
#endif
}

void set_utf8_locale() {
#if defined(_WIN32)
  // Starting in Windows 10 version 1803 (10.0.17134.0), the Universal C Runtime supports using a UTF-8 code page.
  // The change means that char strings passed to C runtime functions can expect strings in the UTF-8 encoding.
  // To enable UTF-8 mode, use ".UTF8" as the code page when using setlocale.
  //
  // Note that if we use FindFirstFileA() (with WIN32_FIND_DATAA), we do not get UTF8-encoded strings!
  // The setlocale(LC_ALL, ".UTF8") only applies to the CRT functions, not the Win32 functions.
  // It works; it allows some changes (not that many) in FileIO.cpp, and one change in Hh_main.cpp.
  //
  // A separate problem is that CONFIG=mingw uses a custom CRT which does not support UTF8 (see
  // https://www.perlmonks.org/?node_id=11153441), so for now we leave this disabled.
  assertx(setlocale(LC_ALL, ".UTF8"));  // Or "en_US.UTF8".
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
  // See my answer at https://stackoverflow.com/a/23437425
  //  to the question of the precision necessary to exactly save and retrieve floating-point numbers :
  //
  // See the nice detailed discussion in
  //  https://randomascii.wordpress.com/2012/03/08/float-precisionfrom-zero-to-100-digits-2/
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
  // The difference between 8 and 9 digits of precision (mainly for numbers between 1000 and 1023) is not worth it.
  //
  // See also:
  //  https://stackoverflow.com/questions/10357192/printf-rounding-behavior-for-doubles
  //  https://www.exploringbinary.com/inconsistent-rounding-of-printed-floating-point-numbers/
  // Visual C++ uses the round-half-away-from-zero rule,
  //  and gcc (actually, glibc) uses the round-half-to-even rule, also known as bankers' rounding.
  // glibc printf() has been updated to take the current IEEE rounding mode into account. This was done
  //  in version 2.17.
  //  round-to-nearest/round-half-away-from-zero is still not an option, so this doesn't help making the
  //  output consistent with other platforms.
  //
  // MinGW gcc uses the Microsoft C runtime, so it's not really going to show any different results than
  //  MSVC as far as printf() is concerned.  (It may be using an older version of the runtime.)
}

void exercise_errors() {
  // See also HTest.
  SHOWL;
  if (0) {
    if (!g_unoptimized_zero) assertnever("Here in do_assertnever");
  }
  if (0) {
    assertx(g_unoptimized_zero == 1);
  }
  if (0) {
    int* p = reinterpret_cast<int*>(size_t(g_unoptimized_zero));
    g_unoptimized_zero = *p;
  }
  if (0) {
    g_unoptimized_zero = 1 / g_unoptimized_zero;
  }
  if (0) {
    float b = 1.f / float(g_unoptimized_zero);  // Silently produces infinity.
    SHOW(b);
  }
  if (0) {
    int a = 123'456'789;
    int b = a * a;  // Silently overflows.
    SHOW(b);
  }
  if (0) {
    throw 0;  // unhandled exception
  }
  SHOW(g_unoptimized_zero);
  exit(0);
}

void hh_init() {
  setup_exception_hooks();
  use_standard_exponent_format_in_io();
  use_binary_io();
  if (0) set_utf8_locale();
  unsynchronize_stream_and_stdio();
  untie_cin_and_cout();
  warn_if_running_debug_version();
  if (0) change_default_io_precision();
  if (0) exercise_errors();
}

}  // namespace

details::HhInit::HhInit() { [[maybe_unused]] static const bool done = (hh_init(), true); }

}  // namespace hh

#endif  // !defined(HH_NO_INIT)
