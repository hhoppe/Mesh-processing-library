# Building, testing, and running the demos

This page gives the requirements of the [Mesh Processing Library](../README.md), and explains how to build it
(using Microsoft Visual Studio, GNU `make`, or Docker), run its [unit tests](#unit-tests), and run its [demos](#demos).


## Requirements / dependencies

The code compiles with Microsoft Visual Studio, using the solution (`*.sln`)
and project (`*.vcxproj`) files.

On Unix (Linux, macOS, WSL, Cygwin),
the code compiles using the `clang` and `gcc` compilers and GNU `make`.

The code requires C++23.
Continuous integration verifies Microsoft Visual Studio 2026,
`gcc` 15, `clang` 20 and 21, and Apple `clang` 21 (Xcode 26),
on Linux (x86-64 and ARM64), macOS (ARM64), and Windows;
Visual Studio 2022 is also supported.<!--
Older compilers lack needed features:
`gcc` 13 lacks explicit object parameters (`this auto&& self`),
`clang` 18 lacks class template argument deduction for alias templates,
and `clang` 19 and 20 fail with the older `libstdc++` 14 (though `clang` 20 works with `libstdc++` 15). -->
GNU `make` 3.81 (as shipped with macOS) suffices.
The `make` builds compile with `-march=native`, so their executables are tuned to (and may require) the CPU of the
building machine; the Visual Studio builds require AVX2 (`/arch:AVX2`).

Reading/writing of images and videos is enabled using several options.
Video I/O spawns the command <a href="https://ffmpeg.org/">`ffmpeg`</a> in a piped subprocess
whenever it is present in the `PATH`, and otherwise uses Windows Media Foundation (WMF) if available.
Image I/O uses Windows Imaging Component (WIC) in Visual Studio builds, `libpng`/`libjpeg` on Linux
and Cygwin, and `ffmpeg` elsewhere (e.g., macOS) or for formats the former lack.
A file specified as a URL is read by spawning the command `wget`.

On Ubuntu 26.04, the needed packages are installed using
`sudo apt install make clang libgl-dev libx11-dev libjpeg-dev libpng-dev zlib1g-dev ffmpeg`
(or `g++` instead of `clang`, to build using `make CC=gcc`).

On macOS, it is necessary to install
<a href="https://www.xquartz.org/">`XQuartz`</a> for `X11` support and
<a href="https://evermeet.cx/ffmpeg/">`ffmpeg`</a> for image/video I/O,
e.g., using `brew install --cask xquartz && brew install ffmpeg`.


## Code compilation

### Build using Microsoft Visual Studio

Open the `mesh_processing.sln` file and build the solution
(typically as a `"ReleaseMD - x64"` build).
Executables are placed in `bin/msbuild` or `bin/msbuild_debug`,
depending on the build configuration.


### Build using GNU `make`

The variable `CONFIG` selects one of five build configurations.
Each is defined in a file `make/Makefile_config_*` and places its executables in the directory `bin/<CONFIG>`:

| `CONFIG` | Platform | Compiler | C++ library | Default |
| --- | --- | --- | --- | --- |
| `unix` | Linux, WSL, macOS | `clang` (or `gcc` using `CC=gcc`) | `libstdc++` (`libc++` on macOS) | release |
| `win` | Windows | Microsoft `cl` | MSVC STL | debug |
| `clang` | Windows | `clang` | MSVC STL | release |
| `mingw` | Windows | MinGW-w64 `gcc` | `libstdc++` | release |
| `cygwin` | Windows (Cygwin) | `gcc` (or `clang` using `CC=clang`) | `libstdc++` | release |

On Unix platforms (Linux, macOS, WSL), `CONFIG=unix` is the unique and default setting.
On Windows, `CONFIG` defaults to `win`, and `make` must be run within a Cygwin or MSYS2 shell,
to provide GNU `make`, `bash`, `perl`, `diff`, and `g++` (used only to determine the header dependencies).

For example:

```shell
make -j                          # Build all programs and run all unit tests.
make -j test                     # Build just the libraries and run all unit tests.
make -j Filtermesh               # Build a single program.
make -j demos                    # Build all programs and run all demos.
make CC=gcc -j                   # On Unix, use the gcc compiler (the default is clang).
make release=0 -j                # Create a debug build (or release=1 for a release build).
make CXX_STD=c++26 -j            # Compile for a different C++ standard (the default is c++23).
make CONFIG=mingw -j libHh       # On Windows, build just the main library using the mingw gcc compiler.
make CONFIG=clang -j Filtermesh  # On Windows, build Filtermesh (into bin/clang) using the clang compiler.
make CONFIG=cygwin -j demos      # Under Cygwin, build all programs and run all demos using the gcc compiler.
make CONFIG=all -j               # Run each configuration of the current platform in turn.
make CONFIG=all -j deepclean     # Clean up all files in all configurations.
```

The settings that affect compilation (`CONFIG`, the compiler, the C++ standard, and debug or release) are recorded,
so that changing any of them rebuilds the affected files.
The `win` configuration creates `*.obj` and `*.lib` files whereas the other four share `*.o` and `*.a` files,
so alternating between `win` and one other configuration requires no rebuilding.

The targets `mostlyclean`, `clean`, and `deepclean` remove progressively more:
the intermediate files, then also the executables, and then also the header-dependency files.

The compiler tool paths are discovered automatically;
to override them, set the variables named in `make/Makefile_base_vc` and `make/Makefile_config_*`
(e.g., `vs_instance`, `MINGW_ROOT`, or `LLVM_ROOT`) in a file `Makefile_local_defs` at the repository root.


### Build using Docker

The file `make/Dockerfile` defines a Linux environment (Ubuntu with `clang`, GNU `make`, and the libraries above)
in which all programs are built and the unit tests are run, without installing anything else:
<br/>`docker build -f make/Dockerfile -t mesh-processing .`

To then start a shell in which the programs are in the `PATH`:
<br/>`docker run -it --rm mesh-processing`

To create and check the demo results (`xvfb-run` provides an X display, and requires `--init`):
<br/>`docker run --init --rm mesh-processing xvfb-run -a make -C demos create check`


## Unit tests

The directory `test` contains the unit tests (`*_test.cpp`), which exercise the classes of `libHh`.
They are run using `make` (the Visual Studio solution does not include them):

```shell
make -j test                # Build the libraries and run all unit tests.
make -C test Array_test.ou  # Run a single unit test.
```

For each test `X_test`, the script `bin/hcheck` runs the program, saves its output in `test/X_test.ou`,
and compares that output with the expected output in `test/X_test.ref`.
On a mismatch, the differences are shown and saved in `test/X_test.diff`, and `make` reports a failure
(as it does on every later run until the test passes).


## Demos

After the code is compiled, the demos can be run as follows.

On Windows, create, view, and clean up all the results using the batch scripts:
```shell
demos\all_demos_create_results.bat
demos\all_demos_view_results.bat
demos\all_demos_clean.bat
```

On Unix-based systems (Linux, macOS, WSL, Cygwin), either run the `bash` scripts:
```shell
demos/all_demos_create_results.sh
demos/all_demos_view_results.sh
demos/all_demos_clean.sh
```

or alternatively (and faster), invoke `make` to create all results in parallel and then view them sequentially:

<pre>
make <em>[CONFIG=<var>config</var>]</em> -j demos
</pre>

Note that pressing the <kbd>Esc</kbd> key closes any open program window.

See also the [many usage examples of individual programs](../progs/README.md).
