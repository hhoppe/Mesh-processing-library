# This file is sourced from the other demos.
# There is no need to run it directly.

# Test that the demos package is self-contained by overriding PATH to just the bare system directories.
# PATH=c:/cygwin/bin:c:/windows/system32
# PATH=c:/windows/system32

# If running from a Makefile, prefer the selected CONFIG.
if [[ ${CONFIG+x} ]]; then
  PATH=../${CONFIG}/bin:$PATH
else # Otherwise, explicitly set the desired build directory here.
  # Add all possible build directories as fallback if not specified below.
  PATH=../bin:../bin/debug:../bin/Win32:../bin/Win32/debug:../bin/win:../bin/w32:../bin/mingw:../bin/clang../bin/cygwin:../bin/mingw32:../bin/unix:$PATH
  # PATH=$PATH                    # hmake (msbuild)
  # PATH=../bin/debug:$PATH
  # PATH=../bin/Win32:$PATH
  # PATH=../bin/Win32/debug:$PATH    # hmake debug win32 static
  # PATH=../bin/win:$PATH
  # PATH=../bin/w32:$PATH
  # PATH=../bin/mingw:$PATH
  # PATH=../bin/clang:$PATH
  # PATH=../bin/cygwin:$PATH
  # PATH=../bin/mingw32:$PATH
  # PATH=../bin/unix:$PATH
  :  # empty statement in case everything above is commented
fi

# Allow running scripts in the current directory.
PATH=.:$PATH

# Here one can adjust the default window size and position for most demos.
export G3DARGS="-geom 750x600"
export G3DARGS="-geom 900x700 -bigfont"
export G3DARGS="-geom 1100x850+50+50 -bigfont"
export G3DARGS="-geom 1100x850+100+50 -bigfont"
if [[ "${BASH_VERSINFO[5]}" == *-apple-* ]]; then
export G3DARGS="-geom 1000x750+0+0 -bigfont"
fi

# For models with texture, use smaller window, depending on graphics memory.
export TEXGEOMETRY="-geom 1000x800+50+50"
export TEXGEOMETRY="-geom 1000x800+100+50"
if [[ "${BASH_VERSINFO[5]}" == *-apple-* ]]; then
export TEXGEOMETRY="-geom 1000x800+0+0"
fi
