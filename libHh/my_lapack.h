// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MY_LAPACK_H_
#define MESH_PROCESSING_LIBHH_MY_LAPACK_H_
#include "libHh/Hh.h"  // HH_REFERENCE_LIB()

#if defined(HH_HAVE_LAPACK)

#if defined(HH_USE_LAPACK_INTEGER)  // Google

#include <lapack.h>
using lapack_int = lapack::integer;

#elif !(defined(_WIN32) || defined(__CYGWIN__))  // unix

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wredundant-decls"
#endif
#include <lapacke.h>  // in either /usr/include/ or /usr/include/lapacke/

#else  // cygwin or WIN32

#if defined(__CYGWIN__)
using lapack_int = int;
#else  // _WIN32
using lapack_int = long;  // was defined as "integer" in f2c.h
#endif

// Continue: cygwin or WIN32

// clapack routines in liblapack
extern "C" {

// SGELSS solves overdetermined or underdetermined systems for GE matrices, using SVD.
// ~/git/lib_src/CLAPACK/SRC/sgelss.c
lapack_int sgelss_(lapack_int* m, lapack_int* n, lapack_int* nrhs, float* a, lapack_int* lda, float* b,
                   lapack_int* ldb, float* s, float* rcond, lapack_int* irank, float* work, lapack_int* lwork,
                   lapack_int* info);

// same for double-precision
lapack_int dgelss_(lapack_int* m, lapack_int* n, lapack_int* nrhs, double* a, lapack_int* lda, double* b,
                   lapack_int* ldb, double* s, double* rcond, lapack_int* irank, double* work, lapack_int* lwork,
                   lapack_int* info);

// https://www.netlib.org/lapack/explore-html/d0/d2a/sgelsx_8f.html
// SGELSX solves overdetermined or underdetermined systems for GE matrices, using QR.
// This routine is deprecated and has been replaced by routine SGELSY.
// ~/git/lib_src/CLAPACK/SRC/sgelsx.c
// lapack_int sgelsx_(lapack_int* m, lapack_int* n, lapack_int* nrhs, float* a, lapack_int* lda,
//                   float* b, lapack_int* ldb, lapack_int* jpvt, float* rcond, lapack_int* irank,
//                   float* work, lapack_int* info);

// https://www.netlib.org/lapack/explore-html/dc/db6/sgelsy_8f.html
lapack_int sgelsy_(lapack_int* m, lapack_int* n, lapack_int* nrhs, float* a, lapack_int* lda, float* b,
                   lapack_int* ldb, lapack_int* jpvt, float* rcond, lapack_int* irank, float* work, lapack_int* lwork,
                   lapack_int* info);

// subroutine SGESVD (JOBU, JOBVT, M, N, A, LDA, S, U, LDU, VT, LDVT, WORK, LWORK, INFO)
// SGESVD computes the singular value decomposition (SVD) for GE matrices
// ~/git/lib_src/CLAPACK/SRC/sgesvd.c
lapack_int sgesvd_(char* jobu, char* jobvt, lapack_int* m, lapack_int* n, float* a, lapack_int* lda, float* s,
                   float* u, lapack_int* ldu, float* vt, lapack_int* ldvt, float* work, lapack_int* lwork,
                   lapack_int* info);

// subroutine SGEQRF (M, N, A, LDA, TAU, WORK, LWORK, INFO)
// SGEQRF computes a QR factorization of a real M-by-N matrix A:
// A = Q * R.
// ~/git/lib_src/CLAPACK/SRC/sgeqrf.c
lapack_int sgeqrf_(lapack_int* m, lapack_int* n, float* a, lapack_int* lda, float* tau, float* work, lapack_int* lwork,
                   lapack_int* info);
}

#if defined(_MSC_VER)
HH_REFERENCE_LIB("liblapack.lib");
HH_REFERENCE_LIB("libBLAS.lib");
#if 1
HH_REFERENCE_LIB("libf2c.lib");  // CLAPACK
#else
HH_REFERENCE_LIB("libF77.lib");  // liblapack
HH_REFERENCE_LIB("libI77.lib");
#endif
#endif  // defined(_MSC_VER)

#endif  // !(defined(_WIN32) || defined(__CYGWIN__))

#endif  // defined(HAVE_LAPACK)

#endif  // MESH_PROCESSING_LIBHH_MY_LAPACK_H_
