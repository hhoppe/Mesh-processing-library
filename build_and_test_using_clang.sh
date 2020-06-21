#!/bin/bash
echo "This is an alternative to running 'make'.  It does not compile files in parallel."

progs="Recon Meshfit Subdivfit Polyfit MeshDistance MeshSimplify reverselines Filterprog FilterPM \
  StitchPM MinCycles Filtermesh Filtera3d Filterframe Filterimage G3dOGL VideoViewer"
tests="$(cd test && ls *.cpp | sed 's/.cpp//')"

if [[ ${WINDIR+x} ]]; then      # on Windows
  llvm_bin="c:/progra~1/LLVM/bin"
  cpp="$llvm_bin/clang++" ar="$llvm_bin/llvm-ar" ranlib="$llvm_bin/llvm-ranlib"
  cppflags="-I.. -I../libHWin -std=c++14 -O3 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHH_NO_IMAGE_IO"
  libs="libHh libHWin"
  ldflags="../libHWin/libHWin.a ../libHh/libHh.a -lglu32 -lopengl32 -lwinmm -lgdi32 -lcomdlg32 -luser32"
  exe=".exe"
else                            # on Unix
  cpp=clang++ ar=ar ranlib=ranlib
  cppflags="-I.. -I../libHWX -std=c++14 -O3 -DNDEBUG -pthread"
  libs="libHh libHWX"
  ldflags="../libHWX/libHWX.a ../libHh/libHh.a -lGLU -lGL -lX11 -ljpeg -lpng -lz"
  exe=""
fi
if [[ -d /Applications ]]; then  # on Mac
  cppflags="$cppflags -I/opt/X11/include"
  ldflags="-L/opt/X11/lib $ldflags"
fi

for lib in $libs; do (cd $lib && echo Building $lib && $cpp $cppflags -c *.cpp && $ar rc $lib.a *.o && $ranlib $lib.a); done

for prog in $progs; do (cd $prog && echo Building $prog && $cpp -o ../bin/clang/$prog$exe $cppflags *.cpp $ldflags); done
prog=Filtervideo; (cd $prog && echo Building $prog && $cpp -o ../bin/clang/$prog$exe $cppflags *.cpp ../VideoViewer/GradientDomainLoop.cpp $ldflags)
prog=G3dVec; (cd $prog && echo Building $prog && $cpp -o ../bin/clang/$prog$exe $cppflags *.cpp ../G3dOGL/{G3d,G3ddraw,G3devent,G3dio}.cpp $ldflags)

for test in $tests; do (cd test && echo Testing $test && $cpp -o $test$exe $cppflags $test.cpp $ldflags && ../bin/hcheck $test); done
