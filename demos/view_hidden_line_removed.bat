@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo Press 'Dh' to toggle hidden-line-removal on/off.
echo .
echo Press 'DP' to save current line-drawing to postscript file (data/spheretext.hlr.ps).
echo .

FilterPM data/spheretext.pm -nfaces 5000 -outmesh | G3dVec -key DhDb---J -st data/spheretext.s3d %G3DARGS% -psfile data/spheretext.hlr.ps
