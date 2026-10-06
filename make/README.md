# Building, testing, and running the demos

This page gives the requirements of the [Mesh Processing Library](../README.md), and explains how to build it
(using Microsoft Visual Studio, GNU `make`, or Docker), run its unit tests, and run its [demos](#demos).


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

On macOS, it is necessary to install
<a href="https://www.xquartz.org/">`XQuartz`</a> for `X11` support and
<a href="https://evermeet.cx/ffmpeg/">`ffmpeg`</a> for image/video I/O.


## Code compilation

### Build using Microsoft Visual Studio

Open the `mesh_processing.sln` file and build the solution
(typically as a `"ReleaseMD - x64"` build).
Executables are placed in `bin/msbuild` or `bin/msbuild_debug`,
depending on the build configuration.


### Build using GNU `make`

The `CONFIG` environment variable determines
which `make/Makefile_config_*` definition file is loaded.
On Windows, `CONFIG` can be chosen among `{win, mingw, clang, cygwin}`,
defaulting to `win` if undefined.
On Unix platforms (Linux, macOS, WSL), `CONFIG=unix` is the unique and default setting.

For example, to build using the Microsoft `cl` compiler (a debug build, placing `*.exe` into directory `bin/win`):
<br/>`make -j8`<br/>

To build all programs (into either `bin/unix` or `bin/win`) and run all unit tests:
<br/>`make -j`

To build just the libraries and run all unit tests:
<br/>`make -j test`

To build on Unix, forcing the use of the `gcc` compiler (default is `clang`):
<br/>`make CC=gcc -j`

To build just the main library using the `mingw gcc` compiler on Windows:
<br/>`make CONFIG=mingw -j libHh`

To build the `Filtermesh` program (into `bin/clang`) using the `clang` compiler on Windows:
<br/>`make CONFIG=clang -j Filtermesh`

To build all programs (into `bin/cygwin`) and run all demos using the `gcc` compiler under Cygwin:
<br/>`make CONFIG=cygwin -j demos`

To clean up all files in all configurations:
<br/>`make CONFIG=all -j deepclean`

Note that additional options such as debug/release and
compiler parameters are set in the various `make/Makefile_*` files.
For instance, the line
`"release ?= 0"` in `make/Makefile_config_win` specifies a debug (non-release) build.
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

Setting the environment variable `DEMOS_HIDDEN=1` instead runs the viewing demos as a non-interactive
check: no window is ever mapped, and each viewer terminates itself after a few seconds.

<pre>
DEMOS_HIDDEN=1 make <em>[CONFIG=<var>config</var>]</em> -C demos view
</pre>

This only detects crashes and assertion failures; it compares no rendered images.
A display is still required, because only the mapping of the window is suppressed.
