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

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/Hh.h"
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"
#include "libHh/Vec.h"

#if 0
#include "libHh/Args.h"
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/Matrix.h"
#include "libHh/Set.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#endif

#if 0
#include "libHh/Pqueue.h"
#include "libHh/Queue.h"
#endif

#if 0 && defined(_MSC_VER)
#pragma hdrstop("precompiled_libHh.pch")
#endif

#endif
