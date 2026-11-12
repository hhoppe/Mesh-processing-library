@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo Use the left button to drag the slider (logarithmic in #vertices).
echo.

set PM_LOD_LEVEL=0.03

call g3d_cmp -g3d pm_view -win1 "-key De" results/gaudipark.pm results/gaudipark.pm -st data/imageup.s3d ^
             -lightambient 1 -lightsource 0 %G3D_ARGS%
