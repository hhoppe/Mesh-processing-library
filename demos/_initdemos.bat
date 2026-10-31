:: This batch file is called from the other demos.
:: There is no need to run it directly.

:: No setlocal because we want to set variables in the parent script.
:: setlocal

:: Locate the demos and the programs relative to this file, so that the demos of another repository may also call it.
set "demos_dir=%~dp0"
set "demos_dir=%demos_dir:\=/%"
set "bin_dir=%demos_dir%../bin"

:: The demos read their inputs from data/ and write all generated files into results/.
if not exist "%~dp0results" mkdir "%~dp0results"

:: Test that the demos package is self-contained by overriding PATH to just the bare system directories.
:: set path=c:/cygwin/bin:c:/windows/system32
:: set path=c:/windows/system32

:: If running from a Makefile, prefer the selected CONFIG.
if defined CONFIG set path=%bin_dir%/%CONFIG%;%path%

:: Otherwise, use the build directories of this platform.
if not defined CONFIG set path=%bin_dir%/mingw;%bin_dir%/clang;%bin_dir%/cygwin;%path%
if not defined CONFIG set path=%bin_dir%/msbuild;%bin_dir%/msbuild_debug;%bin_dir%/win;%path%

:: The scripts in ../bin (e.g., mesh_to_pm.bat) take precedence over the executables.
set path=%bin_dir%;%path%

:: Define the default window geometry of the viewers, doubled on a screen of at least 2560x1600 pixels (if the viewer
:: can report it).  It is not given to g3d_cmp, which arranges its own windows.
set G3D_GEOM=-geom 1000x750+150+0
for /f "tokens=2,3" %%a in ('G3dOGL -hwquery 2^>nul ^<nul') do (
  if %%a geq 2560 if %%b geq 1600 set G3D_GEOM=-geom 2000x1500+150+0
)

:: Setting DEMOS_HIDDEN=1 runs the view demos as a non-interactive check: no window is ever mapped, and
:: each viewer terminates itself after a few seconds using the "\c" escape of -hwkey.  This only detects
:: crashes, assertion failures, and sanitizer reports; it compares no rendered pixels.  A display is still
:: required (X11 or the Windows desktop); only the mapping of the window is suppressed.
set HIDDEN_ARGS=
if defined DEMOS_HIDDEN set HIDDEN_ARGS=-hidden -hwdelay 1 -hwkey \2\c

:: Other arguments for all viewers, including each window of g3d_cmp.
set G3D_ARGS=-bigfont %HIDDEN_ARGS%
set VIDEOVIEWER_ARGS=%HIDDEN_ARGS%

:: Make g3d_cmp wait for its windows to close (and report their failures) rather than run them in the background.
set G3D_CMP_FOREGROUND=1
