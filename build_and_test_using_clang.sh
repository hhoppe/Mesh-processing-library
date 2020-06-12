#!/bin/bash

progs="Recon Meshfit Subdivfit Polyfit MeshDistance MeshSimplify reverselines Filterprog FilterPM \
  StitchPM MinCycles Filtermesh Filtera3d Filterframe Filterimage G3dOGL VideoViewer"
tests="$(cd test && ls *.cpp | sed 's/.cpp//')"

if [[ ${WINDIR+x} ]]; then      # on Windows
  llvm_bin="c:/progra~1/LLVM/bin"
  cpp="$llvm_bin/clang++" ar="$llvm_bin/llvm-ar" ranlib="$llvm_bin/llvm-ranlib"
  cppflags="-I.. -I../libHWin -D_CRT_SECURE_NO_WARNINGS -O3 -DNDEBUG"
  libs="libHh libHWin"
  ldflags="../libHWin/libHWin.a ../libHh/libHh.a -lglu32 -lopengl32 -lwinmm -lgdi32 -lcomdlg32 -luser32"
else                            # on Unix
  cpp=clang++ ar=ar ranlib=ranlib
  cppflags="-I.. -I../libHWX -O3 -DNDEBUG"
  libs="libHh libHWX"
  ldflags="../libHWin/libHWin.a ../libHh/libHh.a -lGLU -lGL -lX11"
fi

for lib in $libs; do (cd $lib && echo Building $lib && $cpp $cppflags -c *.cpp && $ar rc $lib.a *.o && $ranlib $lib.a); done

for prog in $progs; do (cd $prog && echo Building $prog && $cpp -o ../bin/clang/$prog.exe $cppflags *.cpp $ldflags); done
prog=Filtervideo; (cd $prog && echo Building $prog && $cpp -o ../bin/clang/$prog.exe $cppflags *.cpp ../VideoViewer/GradientDomainLoop.cpp $ldflags)
prog=G3dVec; (cd $prog && echo Building $prog && $cpp -o ../bin/clang/$prog.exe $cppflags *.cpp ../G3dOGL/{G3d,G3ddraw,G3devent,G3dio}.cpp $ldflags)

for test in $tests; do (cd test && echo Testing $test && $cpp -o $test.exe $cppflags $test.cpp $ldflags && ../bin/hcheck $test); done
