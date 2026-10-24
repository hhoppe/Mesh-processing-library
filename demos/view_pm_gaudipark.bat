@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo Press 'De' to toggle edges on/off.
echo .
echo Use left button to drag slider (logarithmic in #vertices).
echo .

set PM_LOD_LEVEL=0.03

G3dOGL -pm_mode results/gaudipark.pm -st data/imageup.s3d -key ,De -lightambient 1 -lightsource 0 %G3DARGS%
