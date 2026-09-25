:: This batch file is called from the other demos.
:: There is no need to run it directly.

:: No setlocal because we want to set variables in the parent script.
:: setlocal

:: The demos read their inputs from data/ and write all generated files into results/.
if not exist results mkdir results

:: Test that the demos package is self-contained by overriding PATH to just the bare system directories.
:: set path=c:/cygwin/bin:c:/windows/system32
:: set path=c:/windows/system32

:: If running from a Makefile, prefer the selected CONFIG.
if defined CONFIG set path=../bin/%CONFIG%;%path%

:: Otherwise, add all possible build directories as fallback if not specified below.
if not defined CONFIG set path=../bin/msbuild;../bin/msbuild_debug;../bin/win;../bin/mingw;../bin/clang;../bin/cygwin;../bin/unix;%path%

:: Explicitly set desired build directory here.
:: set path=../bin/msbuild;%path%
:: set path=../bin/msbuild_debug;%path%
:: set path=../bin/win;%path%
:: set path=../bin/mingw;%path%
:: set path=../bin/clang;%path%
:: set path=../bin/cygwin;%path%
:: set path=../bin/unix;%path%

:: The scripts in ../bin (e.g., mesh_to_pm.bat) take precedence over the executables.
set path=../bin;%path%

:: Here one can adjust the default window size and position for most demos.
set G3DARGS=-geom 750x600
set G3DARGS=-geom 900x700 -bigfont
set G3DARGS=-geom 1100x850+100+50 -bigfont
set G3DARGS=-geom 1100x850+150+50 -bigfont

:: For models with texture, use smaller window, depending on graphics memory.
set TEXGEOMETRY=-geom 1000x800+100+50
set TEXGEOMETRY=-geom 1000x800+150+50

:: Setting DEMOS_HIDDEN=1 runs the view demos as a non-interactive check: no window is ever mapped, and
:: each viewer terminates itself after a few seconds using the "\c" escape of -hwkey.  This only detects
:: crashes and assertion failures; it compares no rendered pixels.  The Windows desktop is still required;
:: only the display of the window is suppressed.
set HIDDEN_ARGS=
if defined DEMOS_HIDDEN set HIDDEN_ARGS=-hidden -hwdelay 1 -hwkey \9\c

:: Extra arguments for the viewers; the geometry of %G3DARGS% is overridden by any later -geom.
set G3DARGS=%G3DARGS% %HIDDEN_ARGS%
set VIDEOVIEWER_ARGS=%HIDDEN_ARGS%
