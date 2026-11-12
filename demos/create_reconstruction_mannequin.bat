@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

set r=mannequin

echo.
echo Running phase 1 - Recon.
echo.
Recon data/%r%.pts -samplingd 5.5 | Filtermesh -genus -fillholes 40 -triangulate -genus >results/%r%.recon.m

echo.
echo Running phase 2 - Meshfit.
echo.
Meshfit -mfile results/%r%.recon.m -file data/%r%.pts -reconstruct >results/%r%.opt.m

echo.
echo Running phase 3 - Subdivfit.
echo.
Filtermesh results/%r%.opt.m -angle 40 -mark -genus ^
  | Subdivfit -mfile - -file data/%r%.pts -csharp 8e-6 -reconstruct >results/%r%.sub0.m

echo.
echo Computing final subdivided surface.
echo.
Subdivfit -mfile results/%r%.sub0.m -nsub 2 -outn >results/%r%.sub2limit.m

echo.
echo Use view_reconstruction_mannequin.bat to view the results.
echo.
