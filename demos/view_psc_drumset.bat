@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo Use left button to drag the slider up/down.
echo.

set PSC_LOD_LEVEL=0.1

:: Display lists are off ("DC") because the vertex spheres are tessellated according to their size on the screen.
G3dOGL -psc_mode data/drumset_trunc.psc -st data/drumset.s3d -key DC -lightambient .4 %G3D_GEOM% %G3D_ARGS%
