// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Hh.h"

#if 0
{
    // critical section
    parallel_for_int(i, 100) { something(); HH_LOCK { something_synchronized(): } }
}
#endif

// #define HH_LOCK_USING_OPENMP    // use HH_PRAGMA(omp critical); else use std::mutex and std::lock_guard
// No longer necessary since VC12 is happy with s_per_file_mutex.

#if defined(__clang__) && defined(__GNUC__)
#undef HH_LOCK_USING_OPENMP
#endif

#if defined(HH_LOCK_USING_OPENMP)
#include <omp.h>                // OpenMP
#else
#include <mutex>                // std::mutex
#endif

namespace hh {


//----------------------------------------------------------------------------
// *** No support for locks.
#if defined(__clang__) && defined(__GNUC__)

// workaround: no locking.  clang does not support OpenMP, and mingw target used by clang does not support <mutex>.
#define HH_LOCK


//----------------------------------------------------------------------------
// *** Use OpenMP for synchronization (its default is a globally defined mutex).
#elif defined(HH_LOCK_USING_OPENMP)

#define HH_LOCK HH_PRAGMA(omp critical)
// A thread waits at the beginning of a critical region until no other thread is executing a critical region
//  (anywhere in the program) with the same name.  All unnamed critical directives map to the same unspecified
//  name.
// Drawbacks of OpenMP critical sections:
//  (http://www.thinkingparallel.com/2006/08/21/scoped-locking-vs-critical-in-openmp-a-personal-shootout/)
//  - cannot leave scope using break, continue, goto, or return
//  - cannot leave scope with exception (e.g. gcc does implicit catch and reports error)!
//  - cannot use a separate lock guard per-task in a thread-safe function (rare need).


//----------------------------------------------------------------------------
// *** All critical sections across the program share the same globally defined mutex -- like OpenMP default.
#elif 0

class MyGlobalLock {
 public:
    MyGlobalLock() : _lock_guard(s_f_global_mutex()) { }
 private:
    std::lock_guard<std::mutex> _lock_guard;
    static std::mutex& s_f_global_mutex() { static std::mutex m; return m; } // singleton pattern function
};
#define HH_LOCK if (hh::false_capture<hh::MyGlobalLock> HH_UNIQUE_ID(lock){}) { HH_UNREACHABLE; } else


//----------------------------------------------------------------------------
// *** The critical sections in each compilation unit (*.cpp file) share the same mutex.
#elif 1

namespace {
#if defined(_MSC_VER) && _MSC_VER<1900 // known bug with static std::mutex
// otherwise causes crash on: ~/src/bin/win/G3dOGL -pm_mode ~/data/simplify/banklamp.pm
//  (~RFile from static destruction of "unique_ptr<RFile> g_fi;")
std::mutex& s_per_file_mutex = *new std::mutex(); // never deleted
#else
std::mutex s_per_file_mutex;
#endif
class MyPerFileLock {
 public:
    MyPerFileLock() : _lock_guard(s_per_file_mutex) { }
 private:
    std::lock_guard<std::mutex> _lock_guard;
};
} // namespace
#define HH_LOCK if (hh::false_capture<hh::MyPerFileLock> HH_UNIQUE_ID(lock){}) { HH_UNREACHABLE; } else


//----------------------------------------------------------------------------
// *** Each critical section has its own mutex.
#elif 0

// I can't see a way to implement that using an HH_LOCK { } type macro,
//  because there is no way to declare/allocate a static variable in the middle of a statement.
// Maybe use a false_capture of a templated class with template argument based on __COUNTER__
//  (and singleton pattern function)?


//----------------------------------------------------------------------------
// *** Old paired macros (all critical sections use a separately defined mutex).
#else

// Apparently it is unsafe to do a static allocation of static std::mutex;
//  tConsoleProgress crashes under Visual Studio VC11 and VC12.
// However, it should be safe:
//  http://stackoverflow.com/questions/14106653/are-function-local-static-mutexes-thread-safe
// It should be fixed in future; see my bug report:
//  https://connect.microsoft.com/VisualStudio/feedback/details/808030/runtime-crash-with-static-std-mutex
// Also, <mutex> appears to not be supported in MSC /clr
//  http://stackoverflow.com/questions/15821942/how-to-implement-a-unmanaged-thread-safe-collection-when-i-get-this-error-mute
#error These paired macros are no longer supported.
#define HH_BEGIN_LOCK { static std::mutex my_mutex1; std::lock_guard<std::mutex> HH_UNIQUE_ID(lock){my_mutex1};
#define HH_END_LOCK } HH_EAT_SEMICOLON

#endif


} // namespace hh


//----------------------------------------------------------------------------

// Good discussion:
//
// http://stackoverflow.com/questions/23519630/are-there-c11-critical-sections
//  The C++11 std::mutex does not require cross-processing locking, so a reasonable implementation
//   should avoid cross-process objects like named semaphores (or win32 Mutex).
//
// http://stackoverflow.com/questions/800383/what-is-the-difference-between-mutex-and-critical-section/
//
// For Windows, critical sections are lighter-weight than mutexes.   [here mutex is a win32 Mutex, not std::mutex]
// - Mutexes can be shared between processes, but always result in a system call to the kernel which has some overhead.
// - Critical sections can only be used within one process, but have the advantage that they only switch to
//    kernel mode in the case of contention - Uncontended acquires, which should be the common case, are
//    incredibly fast. In the case of contention, they enter the kernel to wait on some synchronization primitive
//    (like an event or semaphore).

// The following details are specific to critical sections on windows:
// - in the absence of contention, acquiring a critical section is as simple as an InterlockedCompareExchange operation
// - the critical section structure holds room for a mutex. It is initially unallocated
// - if there is contention between threads for a critical section, the mutex will be allocated and used. The
//     performance of the critical section will degrade to that of the mutex
// - if you anticipate high contention, you can allocate the critical section specifying a spin count.
// - if there is contention on a critical section with a spin count, the thread attempting to acquire the
//     critical section will spin (busy-wait) for that many processor cycles. This can result in better
//     performance than sleeping, as the number of cycles to perform a context switch to another thread can be
//     much higher than the number of cycles taken by the owning thread to release the mutex
// - if the spin count expires, the mutex will be allocated
// - when the owning thread releases the critical section, it is required to check if the mutex is allocated,
//    if it is then it will set the mutex to release a waiting thread

// http://stackoverflow.com/questions/7798010/openmp-atomic-vs-critical/
//  In OpenMP all unnamed critical sections are considered identical (if you prefer, there's only one lock for
//    all unnamed critical sections).
// #pragma omp critical [(name)]
//   Note that name must be enclosed in parentheses.

// Also use the following, which is faster than a critical section:
//  HH_PRAGMA(omp atomic) g_nslow++;
// It works for x+=, x-=, x*=, x&=, etc., and --x and ++x.
// Better yet, use std::atomic<T>.
// Possibly could use atomic_operate() defined in AtomicOperate.h
