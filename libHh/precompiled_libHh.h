// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt

// The contents are used for several purposes:
// (1) With the _MSC_VER compiler,  to build precompiled_libHh.pch during the compilation of precompiled_libHh.cpp
// (2) With the __clang__ compiler, to build precompiled_libHh.h.pch during the compilation of precompiled_libHh.h
// (3) With the __GNUC__ compiler,  to build precompiled_libHh.h.gch during the compilation of precompiled_libHh.h
// (4) To build the dependences in Makefile.dep using rule $(makedep)

// Thus we can omit the contents if defined(PRECOMPILED_LIBHH_CPP) from precompiled_libHh.cpp and !defined(_MSC_VER)
//  and !defined(MAKEFILE_DEP).

#if !defined(MAKEFILE_DEP) && \
    (defined(PRECOMPILED_LIBHH_CPP) && !defined(_MSC_VER) || (defined(__clang__) && defined(_MSC_VER)))

// no content

#else

#include "Array.h"
#include "Geometry.h"
#include "Hh.h"
#include "RangeOp.h"
#include "Stat.h"
#include "Vec.h"

#if 0
#include "Args.h"
#include "FileIO.h"
#include "GMesh.h"
#include "Matrix.h"
#include "Set.h"
#include "StringOp.h"
#include "Timer.h"
#endif

#if 0
#include "Pqueue.h"
#include "Queue.h"
#endif

#if 0 && defined(_MSC_VER)
#pragma hdrstop("precompiled_libHh.pch")
#endif

#endif
