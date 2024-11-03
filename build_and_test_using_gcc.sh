#!/bin/bash
echo "This is an alternative to running 'make'.  It does not compile files in parallel."

progs="Recon Meshfit Subdivfit Polyfit MeshDistance MeshSimplify reverselines Filterprog FilterPM \
  StitchPM MinCycles MeshReorder SphereParam SphereSample Filtermesh Filtera3d Filterframe Filterimage \
  G3dOGL VideoViewer"  # Filtervideo and G3dVec are handled separately below.
tests="$(cd test && ls *.cpp | sed 's/.cpp//')"

if [[ ${WINDIR+x} ]]; then      # on Windows
  gcc_bin="/cygdrive/c/progra~1/mingw-w64/x86_64-5.2.0-posix-seh-rt_v4-rev0/mingw64/bin"
  cpp="$gcc_bin/g++" ar="$gcc_bin/ar" ranlib="$gcc_bin/ranlib"
  cppflags="-I.. -I../libHwWindows -std=c++17 -O3 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHH_NO_IMAGE_IO"
  libs="libHh libHwWindows"
  ldflags="../libHwWindows/libHwWindows.a ../libHh/libHh.a -lopengl32 -lwinmm -lgdi32 -lcomdlg32 -luser32"
  exe=".exe"
else                            # on Unix
  cpp=g++ ar=ar ranlib=ranlib
  cppflags="-I.. -I../libHwX -std=c++17 -O3 -DNDEBUG -pthread"
  libs="libHh libHwX"
  ldflags="../libHwX/libHwX.a ../libHh/libHh.a -lGL -lX11 -ljpeg -lpng -lz"
  exe=""
fi
if [[ -d /Applications ]]; then  # on Mac
  # Note that on Mac, gcc/g++ gets mapped to clang.
  cppflags="$cppflags -I/opt/X11/include"
  ldflags="-L/opt/X11/lib $ldflags"
fi

for lib in $libs; do (cd $lib && echo Building $lib && $cpp $cppflags -c *.cpp && $ar rc $lib.a *.o && $ranlib $lib.a); done

mkdir -p ./bin/gcc
for prog in $progs; do (cd $prog && echo Building $prog && $cpp -o ../bin/gcc/$prog$exe $cppflags *.cpp $ldflags); done
prog=Filtervideo; (cd $prog && echo Building $prog && $cpp -o ../bin/gcc/$prog$exe $cppflags *.cpp ../VideoViewer/GradientDomainLoop.cpp $ldflags)
prog=G3dVec; (cd $prog && echo Building $prog && $cpp -o ../bin/gcc/$prog$exe $cppflags *.cpp ../G3dOGL/{G3d,G3ddraw,G3devent,G3dio}.cpp $ldflags)

for test in $tests; do (cd test && echo Testing $test && $cpp -o $test$exe $cppflags $test.cpp $ldflags && ../bin/hcheck $test); done
