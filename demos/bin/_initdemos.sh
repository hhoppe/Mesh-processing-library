# This file is sourced from the other demos.
# There is no need to run it directly.

# Fail the script as soon as any command fails, so that sanitizer errors and crashes reach make.
# `pipefail` is needed because several demos pipe one tool into another.
set -e -o pipefail

# A viewer closed by the user kills upstream writers with SIGPIPE (exit status 141); accept that as success.
# Usage: `writer | viewer || allow_sigpipe`.  Any other failure status is preserved.
allow_sigpipe() { local status=$?; ((status == 141)) || return $status; }

# Report the failing command's line and status, since errexit otherwise exits silently.
trap 'echo "${BASH_SOURCE[0]}: line $LINENO: exit status $?" >&2' ERR

# Include stack-trace frames that lack debug information (e.g., graphics drivers) in any crash report.
export STACKTRACE_VERBOSE=1

# The demos read their inputs from data/ and write all generated files into results/.
mkdir -p results

# Test that the demos package is self-contained by overriding PATH to just the bare system directories.
# PATH=c:/cygwin/bin:c:/windows/system32
# PATH=c:/windows/system32

# If running from a Makefile, prefer the selected CONFIG.
if [[ -n $CONFIG ]]; then
  PATH=../bin/$CONFIG:$PATH
else # Otherwise, explicitly set the desired build directory here.
  # Add all possible build directories as fallback if not specified below.
  PATH=../bin:../bin/debug:../bin/win:../bin/mingw:../bin/clang:../bin/cygwin:../bin/unix:$PATH
  # PATH=$PATH                    # hmake (msbuild)
  # PATH=../bin/debug:$PATH
  # PATH=../bin/win:$PATH
  # PATH=../bin/mingw:$PATH
  # PATH=../bin/clang:$PATH
  # PATH=../bin/cygwin:$PATH
  # PATH=../bin/unix:$PATH
  :  # empty statement in case everything above is commented
fi

# Allow running scripts in the current directory.
PATH=.:$PATH

# Here one can adjust the default window size and position for most demos.
export G3DARGS="-geom 750x600"
export G3DARGS="-geom 900x700 -bigfont"
export G3DARGS="-geom 1100x850+100+50 -bigfont"
export G3DARGS="-geom 1100x850+150+50 -bigfont"
if [[ ${BASH_VERSINFO[5]} == *-apple-* ]]; then
export G3DARGS="-geom 1000x750+0+0 -bigfont"
fi

# For models with texture, use smaller window, depending on graphics memory.
export TEXGEOMETRY="-geom 1000x800+100+50"
export TEXGEOMETRY="-geom 1000x800+150+50"
if [[ ${BASH_VERSINFO[5]} == *-apple-* ]]; then
export TEXGEOMETRY="-geom 1000x800+0+0"
fi

# Setting DEMOS_HIDDEN=1 runs the view demos as a non-interactive check: no window is ever mapped, and
# each viewer terminates itself after a few seconds using the "\c" escape of -hwkey.  This only detects
# crashes, assertion failures, and sanitizer reports; it compares no rendered pixels.  A display is still
# required (X11 or the Windows desktop); only the mapping of the window is suppressed.
hidden_args=''
if [[ -n $DEMOS_HIDDEN ]]; then
  hidden_args='-hidden -hwdelay 1 -hwkey \2\c'
fi

# Extra arguments for the viewers; the geometry of $G3DARGS is overridden by any later -geom.
export G3DARGS="$G3DARGS $hidden_args"
export VIDEOVIEWER_ARGS="$hidden_args"
