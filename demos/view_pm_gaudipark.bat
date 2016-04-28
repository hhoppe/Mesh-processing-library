@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo Press 'De' to toggle edges on/off.
echo .
echo Use right button to drag slider exponentially.
echo .

set PM_LOD_LEVEL=0.03

G3dOGL -pm_mode data/gaudipark.pm -st data/imageup.s3d -key ,De -lightambient 1 -lightsource 0 %G3DARGS%
