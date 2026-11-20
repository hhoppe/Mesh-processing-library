@echo off
setlocal

:: The Windows command-prompt version of the bash script recon_process, whose comments describe the parameters
:: (the environment variables RECON_*); unlike it, this version does not detect misspelled variable names and does
:: not report the execution time.  Relative paths are relative to the current directory.  For example:
::   set RECON_ROOT=mechpart& set RECON_SAMPLINGD=.035& set RECON_GRID=60& set RECON_FILLHOLES=0& set RECON_ANGLE=50
::   recon_process.bat

:: Use the programs of this repository, as the demos do.
call "%~dp0../_initdemos.bat"

if not defined RECON_ROOT (echo Error: RECON_ROOT not set. 1>&2 & exit /b 1)
if not defined RECON_SAMPLINGD (echo Error: RECON_SAMPLINGD not set. 1>&2 & exit /b 1)
if not defined RECON_PTS set "RECON_PTS=%RECON_ROOT%.pts"
if not defined RECON_RESULTS set RECON_RESULTS=results
if not defined RECON_RMCOMP set RECON_RMCOMP=100
if not defined RECON_FILLHOLES set RECON_FILLHOLES=40
if not defined RECON_CREP set RECON_CREP=1e-5
if not defined RECON_SUBDIVFIT_CREP set "RECON_SUBDIVFIT_CREP=%RECON_CREP%"
if not defined RECON_ANGLE set RECON_ANGLE=40.0
if not defined RECON_RECORD set RECON_RECORD=0

if not exist "%RECON_PTS%" (echo Error: %RECON_PTS% not found. 1>&2 & exit /b 1)
set "pts=%RECON_PTS%"
set "out=%RECON_RESULTS%/%RECON_ROOT%"
if not exist "%RECON_RESULTS%" mkdir "%RECON_RESULTS%"

:: The command prompt has only integer arithmetic, so compute the Subdivfit -csharp (a fifth of its -crep) using
:: PowerShell, with a format independent of the locale and a lowercase exponent (e.g., 2e-06).
set csharp=
set "csharp_expr=([double]'%RECON_SUBDIVFIT_CREP%' / 5).ToString([cultureinfo]::InvariantCulture).ToLower()"
for /f "usebackq" %%c in (`powershell -NoProfile -Command "%csharp_expr%"`) do set csharp=%%c
if not defined csharp (echo Error: cannot compute the -csharp of Subdivfit. 1>&2 & exit /b 1)

set "recon_args=-samplingd %RECON_SAMPLINGD%"
if defined RECON_GRID set "recon_args=%recon_args% -gridsize %RECON_GRID%"
set "meshfit_args=-crep %RECON_CREP%"
set "subdivfit_args=-crep %RECON_SUBDIVFIT_CREP% -csharp %csharp%"
if not "%RECON_RECORD%"=="0" set "recon_args=%recon_args% -rootname "%out%" -what ugpoclm"
if not "%RECON_RECORD%"=="0" set "meshfit_args=%meshfit_args% -record "%out%.opt.record""
if not "%RECON_RECORD%"=="0" set "subdivfit_args=%subdivfit_args% -record "%out%.sub0.record""

echo.
echo == %RECON_ROOT%: Phase 1 - Recon
echo.
Recon "%pts%" %recon_args% %RECON_RECON_PARAMS% >"%out%.m" || exit /b 1

set "fill_args=-rmcomp %RECON_RMCOMP% -fillholes %RECON_FILLHOLES%"
echo.
echo == %RECON_ROOT%: Filling small reconstruction holes (Filtermesh %fill_args%)
echo.
Filtermesh "%out%.m" -genus %fill_args% -fairtriangulate -fixvertices -genus >"%out%.recon.m" || exit /b 1

echo.
echo == %RECON_ROOT%: Phase 2 - Meshfit
echo.
Meshfit -mfile "%out%.recon.m" -file "%pts%" %meshfit_args% %RECON_MESHFIT_PARAMS% -reconstruct ^
  >"%out%.opt.m" || exit /b 1

echo.
echo == %RECON_ROOT%: Phase 3 - Subdivfit
echo.
Filtermesh "%out%.opt.m" -angle %RECON_ANGLE% -mark -genus ^
  | Subdivfit -mfile - -file "%pts%" %subdivfit_args% %RECON_SUBDIVFIT_PARAMS% -reconstruct ^
  >"%out%.sub0.m" || exit /b 1

echo.
echo == %RECON_ROOT%: Computing the final subdivided surface (Subdivfit -nsub 2 -outn)
echo.
Subdivfit -mfile "%out%.sub0.m" -nsub 2 -outn >"%out%.sub2limit.m" || exit /b 1

echo.
echo == %RECON_ROOT%: done
echo.

:: Suggest a command to compare the points with the successive meshes, using the state file next to the points if any.
for %%f in ("%pts%") do set "s3d=%%~dpnf.s3d"
set "s3d=%s3d:\=/%"
set st_args=
if exist "%s3d%" set "st_args= -st %s3d%"
set "meshes=%out%.recon.m %out%.opt.m %out%.sub0.m %out%.sub2limit.m"
echo.
echo == To view the results:
echo.
echo g3d_cmp -win1 "-key o--J" -winl "-key DmDe" %pts% %meshes%%st_args% -key DmDeDb
