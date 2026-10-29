@echo off
setlocal EnableExtensions EnableDelayedExpansion

::: Usage: g3d_cmp [-g3d G3dOGL_command] [-ncol #cols] [per_win_opts...] filename(s)... [global_opts...]
:::
::: Compare geometric objects side-by-side in a 2D grid of G3dOGL windows, linked by a sequence of directional pipes
::: in raster order to synchronize viewpoint changes. A window is created for each object using
::: `G3dOGL_command filename win_opts... global_opts...`, where:
:::   -g3d G3dOGL_command  viewer command (default "G3dOGL").
:::   -ncol #cols          number of grid columns (default: the one maximizing the window size).
:::   -winf "opts"         extra options for the first window; similarly -winm (middle windows), -winl (last window),
:::                        -wino (all but the first window), and -win<i> (window i, indexed from 1).
:::   global_opts          options given to all windows, from the first unrecognized argument that begins with '-'.
::: The windows run in the background, unless the environment variable G3D_CMP_FOREGROUND is set (as in the demos),
::: in which case the command waits for the windows to close and returns the status of the last window.
::: This is the Windows counterpart of the bash script g3d_cmp; its arguments cannot contain exclamation marks or
::: carets.
:::
::: Examples:
::: g3d_cmp mesh.m mesh.modified.m -key DmDe
::: g3d_cmp -win1 "-key Dm" -win2 "-key De" mesh1.m mesh2.m
::: g3d_cmp -g3d "G3dOGL -pm_mode" demos/data/bunny.split_meridian.pm demos/data/bunny.outer_hull.pm
::: g3d_cmp -g3d "G3dOGL -ply_mode" pr.ply ssd.ply

:: The script uses no "goto" or "call :label", because cmd.exe may fail to find a label in a file with LF line
:: endings. Hence the argument loop is a bounded "for /l", which reads the current arguments using "call" (since
:: "%1" inside a block would expand only once) and ends when they run out.

:: Number of pixels between adjacent windows; windows overlap on a shared border line when gap=0.
set gap=0

set g3d=G3dOGL
set winf=-key "&Oii"
set winm=-nosetforeground -key "&Oii" -async -killeof -input
set winl=-nosetforeground -async -killeof -key ii -input

set "self=%~f0"
set status=
if "%~1"=="" set status=0
if "%~1"=="--help" set status=0

set n=0
if not defined status for /l %%i in (1,1,1000) do if not defined end (
  set "r1=" & set "r2=" & set "a1=" & set "a2=" & set "take=" & set "dest="
  call set r1=%%1
  call set r2=%%2
  call set "a1=%%~1"
  call set "a2=%%~2"
  if not defined r1 (
    set end=1
  ) else if defined global (
    set "gopts=!gopts! !r1!" & set take=1
  ) else if not "!a1:~0,1!"=="-" (
    set "ob!n!=!a1!" & set /a n+=1 & set take=1
  ) else if defined r2 (
    if "!a1!"=="-g3d" (set "g3d=!a2!" & set take=2)
    if "!a1!"=="-ncol" (set "ncol=!a2!" & set take=2)
    if "!a1!"=="-winf" set dest=winf
    if "!a1!"=="-winm" set dest=winm
    if "!a1!"=="-winl" set dest=winl
    if "!a1!"=="-wino" set "dest=winm winl"
    if "!a1:~0,4!"=="-win" if not defined dest (
      set "idx=!a1:~4!" & set "num="
      set /a "num=idx" 2>nul
      if "!num!"=="!idx!" if !num! geq 1 set dest=winx!num!
    )
    if defined dest set take=2
  )
  if defined r1 if not defined take (set global=1 & set "gopts=!r1!" & set take=1)
  rem Append the words of the per-window options, quoting each one to protect characters such as "&".
  if defined dest (
    set "rest=!a2!"
    for /l %%k in (1,1,100) do if defined rest for /f "tokens=1*" %%u in ("!rest!") do (
      for %%d in (!dest!) do set "%%d=!%%d! "%%u""
      set "rest=%%v"
    )
  )
  if defined take shift
  if "!take!"=="2" shift
)

if not defined status if %n%==0 (
  >&2 echo g3d_cmp: requires at least one object
  set status=1
)
if defined status (
  if !status!==0 (
    for /f "delims=" %%s in ('findstr /b ":::" "%self%"') do (set "line=%%s" & echo(!line:~4!)
  ) else (
    for /f "delims=" %%s in ('findstr /b ":::" "%self%"') do (set "line=%%s" & echo(!line:~4!)>&2
  )
  exit /b !status!
)

:: Get the screen size, the work area (excluding the taskbar), the visible window frame (left, top, right, bottom)
::  around the client area, and the offset from a -geom position to the visible frame.
for /f "tokens=1" %%u in ("%g3d%") do set g3d0=%%u
set oy=
for /f "tokens=2,3,5-8,10-13,15,16" %%a in ('%g3d0% -hwquery ^<nul') do (
  set sw=%%a& set sh=%%b& set wx=%%c& set wy=%%d& set ww=%%e& set wh=%%f
  set fl=%%g& set ft=%%h& set fr=%%i& set fb=%%j& set ox=%%k& set oy=%%l
)
if not defined oy (
  >&2 echo g3d_cmp: '%g3d0% -hwquery' failed
  exit /b 1
)

:: Choose the number of columns that maximizes the (square) window size ws, unless given by -ncol.
:: Overlap adjacent windows by the border width (fb, as on the other sides), so that they share a border.
set /a "dx=fl+fr, dy=ft+fb, sep=gap-fb, ws=0, c0=1, c1=n"
if defined ncol set /a "c0=ncol, c1=ncol"
for /l %%c in (%c0%,1,%c1%) do (
  set /a "r=(n+%%c-1)/%%c, s=(ww-(%%c-1)*sep)/%%c-dx, t=(wh-(r-1)*sep)/r-dy"
  if !t! lss !s! set s=!t!
  if %n%==1 set /a "t=(wh-dy)*85/100"
  if !t! lss !s! set s=!t!
  if !s! gtr !ws! set /a "ws=s, nc=%%c, nr=r"
)

:: Center the grid of windows within the work area.
set /a "x0=wx+(ww-nc*(ws+dx)-(nc-1)*sep)/2-ox, y0=wy+(wh-nr*(ws+dy)-(nr-1)*sep)/2-oy, last=n-1"

:: Build the pipeline of windows, in which each window sends its output to the window of the next object.
if %n%==1 set "gopts=!gopts! -key O"
for /l %%k in (0,1,%last%) do (
  set "opts=!winm!"
  if %%k==0 (set "opts=!winf!") else if %%k==%last% set "opts=!winl!"
  set /a "kk=%%k, x=x0+kk%%nc*(ws+dx+sep), y=y0+kk/nc*(ws+dy+sep), kp=kk+1"
  for %%p in (!kp!) do set "extra=!winx%%p!"
  set "stage=!g3d! "!ob%%k!" -geom !ws!x!ws!+!x!+!y! !opts! !extra! !gopts!"
  if %%k==0 (set "pipeline=!stage! <nul") else (set "pipeline=!pipeline! | !stage!")
)

:: The pipeline is expanded with %...% (not !...!) so that cmd.exe parses its "|" characters. The background
::  instance expands the inherited variable itself, so that its quoted arguments pass through "start" unchanged.
:: In the background case, wait 2 seconds (using ping, since "timeout" rejects redirected input) to let the windows
::  load their models before the next command prompt appears.
if not defined G3D_CMP_FOREGROUND (
  start "g3d_cmp" /b cmd /d /c %%pipeline%%
  ping -n 3 127.0.0.1 >nul
  exit /b 0
)
%pipeline%
