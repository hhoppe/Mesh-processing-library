@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

set r=distcap

echo.
echo Running phase 1 - Recon.
echo.
Recon data/%r%.pts -rootname results/%r% -samplingd 0.02 -what ugpoclm >results/%r%.m

echo.
echo Filling any small reconstruction holes.
echo.
Filtermesh results/%r%.m -genus -rmcomp 100 -fillholes 40 -triangulate -fixvertices -genus >results/%r%.recon.m

echo.
echo Running phase 2 - Meshfit.
echo.
Meshfit -mfile results/%r%.recon.m -file data/%r%.pts -record results/%r%.opt.record -reconstruct >results/%r%.opt.m

echo.
echo Running phase 3 - Subdivfit.
echo.
Filtermesh results/%r%.opt.m -angle 52 -mark -genus ^
  | Subdivfit -mfile - -file data/%r%.pts -record results/%r%.sub0.record -reconstruct >results/%r%.sub0.m

echo.
echo Computing the final subdivided surface.
echo.
Subdivfit -mfile results/%r%.sub0.m -nsub 2 -outn >results/%r%.sub2limit.m

echo.
echo Use view_reconstruction_animated.bat to view the results.
echo.
