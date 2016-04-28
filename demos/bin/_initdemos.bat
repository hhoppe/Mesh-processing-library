:: This batch file is called from the other demos.
:: There is no need to run it directly.

:: No setlocal because we want to set variables in the parent script.
:: setlocal

:: Test that the demos package is self-contained by overriding PATH to just the bare system directories.
:: set path=c:/cygwin/bin:c:/windows/system32
:: set path=c:/windows/system32

:: Add all possible build directories as fallback if not specified below.
set path=../bin;../bin/debug;../bin/Win32;../bin/Win32/debug;../bin/win;../bin/w32;../bin/mingw;../bin/clang../bin/cygwin;../bin/mingw32;../bin/unix;%path%

:: Explicitly set desired build directory here.
:: set path=%path%
:: set path=../bin/debug;%path%
:: set path=../bin/Win32;%path%
:: set path=../bin/Win32/debug;%path%
:: set path=../bin/win;%path%
:: set path=../bin/w32;%path%
:: set path=../bin/mingw;%path%
:: set path=../bin/clang;%path%
:: set path=../bin/cygwin;%path%
:: set path=../bin/mingw32;%path%
:: set path=../bin/unix;%path%

:: Here one can adjust the default window size and position for most demos.
set G3DARGS=-geom 750x600
set G3DARGS=-geom 900x700 -bigfont
set G3DARGS=-geom 1100x850+50+50 -bigfont
set G3DARGS=-geom 1100x850+100+50 -bigfont

:: For models with texture, use smaller window, depending on graphics memory.
set TEXGEOMETRY=-geom 1000x800+50+50
set TEXGEOMETRY=-geom 1000x800+100+50
