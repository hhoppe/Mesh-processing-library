// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
// #pragma once

// The contents are used for several purposes:
// (1) With the _MSC_VER compiler,  to build precompiled_libHh.pch during the compilation of precompiled_libHh.cpp
// (2) With the __clang__ compiler, to build precompiled_libHh.h.pch during the compilation of precompiled_libHh.h
// (3) With the __GNUC__ compiler,  to build precompiled_libHh.h.gch during the compilation of precompiled_libHh.h
// (4) To build the dependences in Makefile.dep using rule $(makedep)

// Thus we can omit the contents if defined(PRECOMPILED_LIBHH_CPP) from precompiled_libHh.cpp and !defined(_MSC_VER)
//  and !defined(MAKEFILE_DEP).

#if defined(PRECOMPILED_LIBHH_CPP) && !defined(_MSC_VER) && !defined(MAKEFILE_DEP)

// no content

#else

#include "Hh.h"
#include "Array.h"
#include "Vec.h"
#include "RangeOp.h"
#include "Stat.h"
#include "Geometry.h"

#if 0
#include "FileIO.h"
#include "Args.h"
#include "Matrix.h"
#include "GMesh.h"
#include "StringOp.h"
#include "Timer.h"
#include "Set.h"
#endif

#if 0
#include "Pqueue.h"
#include "Queue.h"
#endif

#if 0 && defined(_MSC_VER)
#pragma hdrstop("precompiled_libHh.pch")
#endif

#endif  // defined(PRECOMPILED_LIBHH_CPP) && !defined(_MSC_VER) && !defined(MAKEFILE_DEP)
