// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PARALLEL_H_
#define MESH_PROCESSING_LIBHH_PARALLEL_H_


#include "Hh.h"

// This macro can be set here, in Makefile, or at the top of any source file (if careful with precompiled headers).
// #define HH_NO_OPENMP

#if 0
{
    parallel_for_int(i, n) { func(i); }
    cond_parallel_for_int(n*1000, i, n) { func_1000_instruction_cycles(i); } // only parallelize if beneficial
    int sum = 0; omp_parallel_for_range(reduction(+:sum) if(ar.num()>k_omp_thresh), int, i, 0, ar.num()) sum += ar[i];

}
#endif

#if defined(_MSC_VER) && defined(HH_DEBUG)
// Bug in Visual Studio 2015 update 1; compilation creates buggy code in x64-Debug with OpenMP in:
//  MatrixOp.h: invert(CMatrixView<T> mi, MatrixView<T> mo)  (from Frame::invert() from tGeom.cpp);
//  Multigrid.h: dual_downsample_aux(Specialize<2>, CGridView<D,T> grid)  (from tMultigrid.cpp);
#define HH_DISABLE_OPENMP
#endif

#if !defined(_OPENMP) && !defined(HH_NO_OPENMP) // if compiler feature is absent,  we disable the macros as well
#define HH_NO_OPENMP
#endif

#if !defined(HH_NO_OPENMP) || defined(__WIN32)
#include <omp.h>                // OpenMP
#else
// Note: clang 3.5 and 3.7 do not support OpenMP.
// There is an initial attempt to incorporate OpenMP in clang: http://clang-omp.github.io/
//   but it is preliminary and does not support Windows.
inline int omp_get_max_threads() { return 1; }
inline void omp_set_num_threads(int) { }
inline void omp_set_dynamic(bool) { }
#endif

#if defined(HH_NO_OPENMP)
#define HH_DISABLE_OPENMP
#endif

namespace hh {

// Notes on using OpenMP:
// - Any OpenMP pragma must immediately precede the "for" statement,
//    so I must embed the OpenMP pragma inside the omp_parallel_for_range() macro.
// - With OpenMP, the "for" statement initializer may only declare a single variable,
//    so I must evaluate the upper-bound ub earlier in the omp_parallel_for_range() macro.
// - OpenMP version 2.0 (VS2010, 2013, 2015) only allows signed integral index types
//    (version 3.0 allows unsigned types),
//    so for now I must override the macros for parallel_for_size_t() on VS.


#if defined(HH_DISABLE_OPENMP)
 
#define HH_PRAGMA_OMP(...)
#define omp_parallel_for_range(omp_args, T, i, lb, ub) for_range(T, i, lb, ub)

#else  // defined(HH_DISABLE_OPENMP)

#define HH_PRAGMA_OMP(...) HH_PRAGMA(omp __VA_ARGS__)

#if defined(HH_DEBUG) && 1      // allows more warnings but does not pre-evaluate the upper-bound ub

#define omp_parallel_for_range(omp_args, T, i, lb, ub)                        \
    HH_PRAGMA_OMP(parallel for omp_args) for (T i = lb; i<(ub); i++)

#else

#if defined(__clang__)
// for "add explicit braces to avoid dangling else"
#define HH_OMP_PUSH_WARNINGS HH_PRAGMA(clang diagnostic push) HH_PRAGMA(clang diagnostic ignored "-Wdangling-else")
#define HH_OMP_POP_WARNINGS HH_PRAGMA(clang diagnostic pop)
#elif defined(__GNUC__)
#define HH_OMP_PUSH_WARNINGS HH_PRAGMA(GCC diagnostic ignored "-Wparentheses") // "suggest explicit braces"
#define HH_OMP_POP_WARNINGS // With gcc, the pop would have to occur later outside the macro.
#pragma GCC diagnostic ignored "-Wparentheses" // strange; this alone does not work in a precompiled header
#else
#define HH_OMP_PUSH_WARNINGS
#define HH_OMP_POP_WARNINGS
#pragma warning(disable:4701)   // "potentially uninitialized local variable 'xx' used"
#endif
#define omp_parallel_for_range(omp_args, T, i, lb, ub)                        \
    omp_parallel_for_range_aux(omp_args, T, i, lb, ub, HH_UNIQUE_ID(u))
#define omp_parallel_for_range_aux(omp_args, T, i, lb, ub, u) \
    HH_OMP_PUSH_WARNINGS if (hh::false_capture<T> u = ub) { HH_UNREACHABLE; } else          \
        HH_OMP_POP_WARNINGS HH_PRAGMA_OMP(parallel for omp_args) for (T i = lb; i<u(); i++)

#endif  // defined(HH_DEBUG)

#endif  // defined(HH_DISABLE_OPENMP)


#define parallel_for_range(T, i, lb, ub)         omp_parallel_for_range(, T, i, lb, ub)
#define cond_parallel_for_range(c, T, i, lb, ub) omp_parallel_for_range(if((c)>=hh::k_omp_thresh), T, i, lb, ub)

#define parallel_for_int(i, ub)      parallel_for_range(int, i, 0, ub)
#define parallel_for_intL(i, lb, ub) parallel_for_range(int, i, lb, ub)
#define parallel_for_size_t(i, ub)   parallel_for_range(size_t, i, 0, ub)

// (Note: name parallel_for() conflicts with that in VC <ppl.h> )
#define hh_parallel_for(...) HH_PRAGMA_OMP(parallel for) for (__VA_ARGS__)

#define cond_parallel_for_int(c, i, ub)    cond_parallel_for_range(c, int, i, 0, ub)
#define cond_parallel_for_size_t(c, i, ub) cond_parallel_for_range(c, size_t, i, 0, ub)

const uint64_t k_omp_thresh = 100*1000; // number of instruction cycles above which a loop should be parallelized
const uint64_t k_omp_many_cycles_per_elem = 400; // number of instruction cycles for larger-overhead parallelism
const int k_omp_min_iterations = 8;              // sufficient number of loop iterations for parallelism

#if _OPENMP<200805 // (2.0==200203; 3.0==200805; 4.0==201307; VS2015 is still 2.0)
// http://dautovri.blogspot.com/2015/04/check-your-openmp-version.html
// Loop variable cannot be unsigned, so use a signed one instead.
#undef parallel_for_size_t
#define parallel_for_size_t(i, ub)  parallel_for_range(intptr_t, i, 0, static_cast<intptr_t>(ub))
#undef cond_parallel_for_size_t
#define cond_parallel_for_size_t(c, i, ub) cond_parallel_for_range(c, intptr_t, i, 0, static_cast<intptr_t>(ub))
#endif

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_PARALLEL_H_
