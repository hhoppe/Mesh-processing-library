# Mesh-processing-library

A C++23 computational geometry and mesh processing library
(github.com/hhoppe/Mesh-processing-library).

## Build

Two parallel build systems cover the same sources:

- GNU make, with the machinery under `make/`.
- MSBuild, with roughly 24 `.vcxproj` files in a `.sln`, sharing `hhmain.props`
  and `hhmain_first.props`.

The make build has five configurations, selected with `CONFIG=`:

| CONFIG   | Compiler and standard library                |
| -------- | -------------------------------------------- |
| `unix`   | WSL/Linux Clang + libstdc++                  |
| `win`    | MSVC                                         |
| `clang`  | Windows Clang + MSVC STL                     |
| `mingw`  | Windows GCC + libstdc++                      |
| `cygwin` | Cygwin GCC + libstdc++ (`CC=clang` optional) |

`make -j12` builds all programs into `bin/<CONFIG>/` and runs the unit tests. Name a target
to build less, e.g. `make -j12 Filtermesh` or `make CONFIG=mingw -j12 libHh`.

- The default `CONFIG` is `win` on Windows and `unix` elsewhere.
- `CONFIG=win` defaults to a debug build (`release=0`); the others default to release.
- Under `unix`, the compiler is Clang by default; `CC=gcc` switches to GCC.
- `make CONFIG=all` (or `make makeall`) runs every configuration in turn.
- `PEDANTIC=1` enables the full warning set; adding `ignore_compile_warnings=0` also makes warnings
  errors (`-Werror`, or `-WX` under MSVC).
- Sanitizers work under `CONFIG=unix` only (mingw ships no sanitizer runtime):
  `make CONFIG=unix release=0 PEDANTIC=1 sanitize=address,undefined -j12 test`, or
  `sanitize=thread`.
- The Windows configurations run make under Cygwin. From WSL, launch them with
  `/mnt/c/cygwin64/bin/bash.exe -lc 'cd /hh/git/mesh_processing && make CONFIG=win -j12' </dev/null`.
- Toolchain paths can be overridden in `Makefile_local_defs` at the repository root.
- The `.exe` files directly in `bin/` come from the MSBuild `ReleaseMD - x64` build.

Warnings are signal, not noise. Treat every new warning as a defect to fix, in every
configuration, not just the one you happened to build.

Toolchain discovery in the makefiles uses `$(wildcard)` rather than `$(shell)`, because
the recursive make invocations make shell subprocesses expensive. Keep it that way.

## Layout

- `libHh` is the core library (containers, geometry, meshes, images, video, audio);
  `libHh/README.txt` gives an overview of its core classes.
- `libHwWindows` (Win32) and `libHwX` (X11) implement windowing; a build links one of them.
- Each program (`Filtermesh`, `MeshSimplify`, `G3dOGL`, ...) lives in its own directory and
  links against these libraries. `G3dVec` compiles sources from `G3dOGL`, and `Filtervideo`
  compiles `VideoViewer/GradientDomainLoop.cpp`, hence their ordering in the top-level
  `Makefile`.
- `test/` holds the unit tests. `make demos` builds all programs and runs `demos/`, which reads
  its inputs from `demos/data/` and writes all generated files into `demos/results/`.

## Test

- Run all unit tests with `make -j12 test`, or a single one with
  `make -C test Array_test.ou`.
- For each test, `bin/hcheck` runs `X_test` (or `X_test.script` when present), filters the
  output through `bin/hcheck_aux` (which masks dates, paths, and `.exe`) into `X_test.ou`,
  and diffs it against `X_test.ref`, leaving `X_test.diff` on a mismatch.
- `.ref` files are ground truth. Each test has a single `.ref`, which must match across
  `-O0` through `-O3` and every configuration.
- Floating-point discrepancies from vectorization or sanitizer differences are expected
  and are handled with `round()` wrappers rather than by loosening the comparison.
- Test files use `SHOW()` and `assertx()`, with explicit template instantiations at the
  bottom of the file.

A change is not done until the tests pass and the affected configurations build.

## Validation tiers

Choose the configurations by what a change touches, always with `PEDANTIC=1`:

| Change touches | Validate with |
| -------------- | ------------- |
| Programs, `libHwX`, `libHwWindows`, demo scripts | `unix` and `win`, building only the affected directories; plus the MSBuild build and `make -C demos create check` when rendering or demo outputs may change |
| `libHh` (including the core headers) | also `unix CC=gcc`, and all unit tests under `unix` and `win` |
| `make/`, portability code, or before a checkpoint tag | every configuration, all tests, and the demos |

- `unix` (`.o`) and `win` (`.obj`) keep separate object files, so alternating between them
  stays incremental. `unix`, `mingw`, `cygwin`, and `clang` all share `.o` files (and the
  `unix` and `clang` builds also share the libHh precompiled header), so switching among those
  rebuilds everything.
- Treat any line containing "error" or "warning" in a build log as a failure, and confirm
  that the executables were relinked (or are newer than the changed sources). A running
  program locks its `.exe` on Windows, which makes the link fail.

## Code conventions

- The `k_` prefix is reserved for constants, never for functions. Use the static
  accessor pattern instead: `Pixel::white()`, not `k_Pixel_white`.
- Prefer one-liner operation blocks where the body macros (`NEW_GG`, `NEW_G`, and
  similar) make that readable.
- End macro definitions with `HH_EAT_SEMICOLON` so that MSVC `/W4` stays quiet at
  call sites.
- Prefer a loud compile-time or runtime failure over a silent wrong answer.

## Correctness standards

These are strict, and they are the point of the project.

- **Never claim a change is codegen-neutral without measuring it.** Produce per-function
  assembly diffs at both `-O2` and `-O2 -DNDEBUG`. Use `g++ -S`, or
  `llvm-objdump -d -S --x86-asm-syntax=intel` for interleaved source and assembly.
- Never report static instruction counts as a performance result. If the question is
  speed, benchmark and report measured time or cycles.
- When adding `noexcept`, measure the codegen impact rather than annotating broadly on
  the assumption that it helps.
- Validate across configurations (see "Validation tiers") before considering a change
  complete. A change that builds cleanly under `CONFIG=unix` may fail under MSVC or mingw.

## Static analysis

- `clang-tidy`, configured by the repository `.clang-tidy`. Its header comment has the
  command lines for running it on one file or across the whole tree.
- Run `clang-tidy` both under WSL (with `-I./libHwX`) and under Windows with the Windows LLVM
  build (with `-I./libHwWindows`). Each covers code that the other does not: `libHwWindows`, the
  `_WIN32` code paths, and the MSVC STL on one side, and `libHwX` and libstdc++ on the other.
- Clang Static Analyzer, with `ipa-always-inline-size=5` in `ExtraArgs`.
- Do not suppress `-Wdangling-reference` or `-Wnrvo` broadly. Silence a confirmed false
  positive locally, as `[[HH_NO_DANGLING]]` (`libHh/Hh.h`) and the pragma in
  `MeshSimplify/MeshSimplify.cpp` do.

## Working in this codebase

- Discuss the approach before writing code. For anything touching the core headers
  (`Mesh.h`, `RangeOp.h`, `Array.h`, `Grid.h`, `Vec.h`, `Spatial.h`), use plan mode and
  get agreement on the design first.
- Keep changes small enough to verify. A large refactor that builds is less valuable
  than a small one whose assembly you can diff.
- For history questions, `git log --oneline -S '<term>' --all` finds when a code string
  appeared or vanished, and `git grep -nE '<pattern>' <commit> -- <path>` inspects a
  specific revision.
