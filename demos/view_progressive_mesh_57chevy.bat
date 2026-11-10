@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo Use the left button to drag the slider (logarithmic in #vertices).
echo .

set PM_LOD_LEVEL=0.1

call g3d_cmp -g3d pm_view -win1 "-key De" results/57chevy.pm results/57chevy.pm -st data/57chevy.s3d ^
             -lightambient .4 %G3D_ARGS%
