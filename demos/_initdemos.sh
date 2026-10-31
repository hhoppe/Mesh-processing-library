# This file is sourced from the other demos.
# There is no need to run it directly.

# Locate the demos and the programs relative to this file, so that the demos of another repository may also source it.
demos_dir=$(dirname "${BASH_SOURCE[0]}")
bin_dir=$demos_dir/../bin

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
mkdir -p "$demos_dir/results"

# Test that the demos package is self-contained by overriding PATH to just the bare system directories.
# PATH=c:/cygwin/bin:c:/windows/system32
# PATH=c:/windows/system32

# If running from a Makefile, prefer the selected CONFIG.
if [[ -n $CONFIG ]]; then
  PATH=$bin_dir/$CONFIG:$PATH
  # WSL bash does not resolve Filtermesh to Filtermesh.exe, so a Windows CONFIG would silently fall back to
  # whatever other build is on PATH.
  if [[ $(type -P Filtermesh) != $bin_dir/$CONFIG/* ]]; then
    echo "_initdemos.sh: CONFIG=$CONFIG, but Filtermesh resolves to '$(type -P Filtermesh)'." >&2
    exit 1
  fi
else  # Otherwise, use the build directories of this platform.
  if [[ $OSTYPE == cygwin ]]; then
    PATH=$bin_dir/msbuild:$bin_dir/msbuild_debug:$bin_dir/win:$bin_dir/mingw:$bin_dir/clang:$bin_dir/cygwin:$PATH
  else
    PATH=$bin_dir/unix:$PATH
  fi
fi

# The scripts in ../bin (e.g., mesh_to_pm) take precedence over the executables.
PATH=$bin_dir:$PATH

# Allow running scripts in the current directory.
PATH=.:$PATH

# The Cygwin builds of the viewers draw through X11, so start an X server if none is running.
if [[ $(type -P G3dOGL) == */bin/cygwin/* ]]; then ensure_x11_server; fi

# Define the default window geometry of the viewers, doubled on a screen of at least 2560x1600 pixels (if the viewer
# can report it).  It is not given to g3d_cmp, which arranges its own windows.
read -r _ screen_width screen_height _ < <(G3dOGL -hwquery </dev/null 2>/dev/null) || true
win_size=1000x750
if ((${screen_width:-0} >= 2560 && ${screen_height:-0} >= 1600)); then win_size=2000x1500; fi
win_pos=+150+0
if [[ ${BASH_VERSINFO[5]} == *-apple-* ]]; then win_pos=+0+0; fi
export G3D_GEOM="-geom $win_size$win_pos"

# Setting DEMOS_HIDDEN=1 runs the view demos as a non-interactive check: no window is ever mapped, and
# each viewer terminates itself after a few seconds using the "\c" escape of -hwkey.  This only detects
# crashes, assertion failures, and sanitizer reports; it compares no rendered pixels.  A display is still
# required (X11 or the Windows desktop); only the mapping of the window is suppressed.
hidden_args=''
if [[ -n $DEMOS_HIDDEN ]]; then
  hidden_args='-hidden -hwdelay 1 -hwkey \2\c'
fi

# Other arguments for all viewers, including each window of g3d_cmp.
export G3D_ARGS="-bigfont $hidden_args"
export VIDEOVIEWER_ARGS="$hidden_args"

# Make g3d_cmp wait for its windows to close (and report their failures) rather than run them in the background.
export G3D_CMP_FOREGROUND=1
