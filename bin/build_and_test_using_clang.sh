#!/bin/bash

cd "$(dirname "$0")/.."  # Run from the repository root.
echo "This is an alternative to running 'make'.  It does not compile files in parallel."

shopt -s nullglob  # So that "*.c" expands to nothing in directories lacking C source files.

progs="Recon Meshfit Subdivfit Polyfit MeshDistance MeshSimplify Filterprog FilterPM \
  StitchPM MinCycles MeshReorder SphereParam SphereSample Filtermesh Filtera3d Filterframe Filterimage \
  G3dOGL VideoViewer"  # Filtervideo and G3dVec are handled separately below.
tests="$(cd test && ls *.cpp | sed 's/.cpp//')"

if [[ ${WINDIR+x} ]]; then      # on Windows
  llvm_bin="c:/progra~1/LLVM/bin"
  cpp="$llvm_bin/clang++" ar="$llvm_bin/llvm-ar" ranlib="$llvm_bin/llvm-ranlib"
  cppflags="-I.. -I../libHwWindows -std=c++23 -O3 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHH_NO_IMAGE_LIBS"
  libs="libHh libHwWindows"
  ldflags="../libHwWindows/libHwWindows.a ../libHh/libHh.a -lopengl32 -lwinmm -lgdi32 -lcomdlg32 -luser32"
  exe=".exe"
else                            # on Unix
  cpp=clang++ ar=ar ranlib=ranlib
  cppflags="-I.. -I../libHwX -std=c++23 -O3 -DNDEBUG -pthread"
  libs="libHh libHwX"
  ldflags="../libHwX/libHwX.a ../libHh/libHh.a -lGL -lX11 -ljpeg -lpng -lz"
  exe=""
fi
if [[ -d /Applications ]]; then  # on Mac
  cppflags="$cppflags -I/opt/X11/include"
  ldflags="-L/opt/X11/lib $ldflags"
fi

for lib in $libs; do (cd $lib && echo Building $lib && $cpp $cppflags -c *.cpp && $ar rc $lib.a *.o && $ranlib $lib.a); done

mkdir -p ./bin/clang
for prog in $progs; do (cd progs && echo Building $prog && $cpp -o ../bin/clang/$prog$exe $cppflags -x c++ $prog/*.cpp $prog/*.c -x none $ldflags); done
prog=Filtervideo; (cd progs && echo Building $prog && $cpp -o ../bin/clang/$prog$exe $cppflags $prog/*.cpp VideoViewer/GradientDomainLoop.cpp $ldflags)
prog=G3dVec; (cd progs && echo Building $prog && $cpp -o ../bin/clang/$prog$exe $cppflags $prog/*.cpp G3dOGL/{G3d,G3ddraw,G3devent,G3dio}.cpp $ldflags)

for test in $tests; do (cd test && echo Testing $test && $cpp -o $test$exe $cppflags $test.cpp $ldflags && ../bin/hcheck $test); done
