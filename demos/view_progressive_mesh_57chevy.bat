@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo Press 'De' to toggle edges on/off.
echo Or, press 'Ds' to toggle faces on/off.
echo .
echo Use left button to drag slider (logarithmic in #vertices).
echo .

set PM_LOD_LEVEL=0.1

G3dOGL -pm_mode results/57chevy.pm -st data/57chevy.s3d -key , -lightambient .4 %G3D_GEOM% %G3D_ARGS%
