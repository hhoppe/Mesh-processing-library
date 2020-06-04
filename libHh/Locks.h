// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_LOCKS_H_
#define MESH_PROCESSING_LIBHH_LOCKS_H_

#include <mutex>  // mutex, lock_guard

#include "libHh/Hh.h"

#if 0
{
  // critical section
  parallel_for_each(range(100), [&](const int i) {
    something();
    HH_LOCK { something_synchronized(); }
  });
  // alternate
  std::mutex g_mutex;
  parallel_for_each(range(100), [&](const int i) {
    something();
    {
      std::lock_guard<std::mutex> lg(g_mutex);
      something_synchronized();
    }
  });
}
#endif

namespace hh {

//----------------------------------------------------------------------------
// *** No support for locks.
#if defined(HH_DEFINE_STD_MUTEX)

#define HH_LOCK

//----------------------------------------------------------------------------
// *** Use OpenMP for synchronization (its default is a globally defined mutex).
#elif 0

#include <omp.h>  // OpenMP
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
  MyGlobalLock() : _lock_guard(global_mutex_instance()) {}

 private:
  std::lock_guard<std::mutex> _lock_guard;
  static std::mutex& global_mutex_instance() {
    static auto& m = *new std::mutex;
    return m;
  }
};
#define HH_LOCK                                                            \
  if (hh::details::false_capture<hh::MyGlobalLock> HH_UNIQUE_ID(lock){}) { \
    HH_UNREACHABLE;                                                        \
  } else

//----------------------------------------------------------------------------
// *** The critical sections in each compilation unit (*.cpp file) share the same mutex.
#elif 1

namespace {
std::mutex s_per_file_mutex;
class MyPerFileLock {
 public:
  MyPerFileLock() : _lock_guard(s_per_file_mutex) {}

 private:
  std::lock_guard<std::mutex> _lock_guard;
};
}  // namespace
#define HH_LOCK                                                             \
  if (hh::details::false_capture<hh::MyPerFileLock> HH_UNIQUE_ID(lock){}) { \
    HH_UNREACHABLE;                                                         \
  } else

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

#error These paired macros are no longer supported.
#define HH_BEGIN_LOCK            \
  {                              \
    static std::mutex my_mutex1; \
    std::lock_guard<std::mutex> HH_UNIQUE_ID(lock){my_mutex1};
#define HH_END_LOCK \
  }                 \
  HH_EAT_SEMICOLON

#endif

}  // namespace hh

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
// It works for x += y, x -= y, x *= y, x &=  y, etc., and --x and ++x.
// Better yet, use std::atomic<T>.
// Possibly could use atomic_operate() defined in AtomicOperate.h

#endif  // MESH_PROCESSING_LIBHH_LOCKS_H_
