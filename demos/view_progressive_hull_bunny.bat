@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo Press 'De' to toggle edges on/off.
echo .
echo Use right button to drag slider up/down.
echo .

set PM_LOD_LEVEL=0.001

G3dOGL -pm_mode results/bunny.outer_hull.pm -st data/bunny.s3d -key ,De %G3D_GEOM% %G3D_ARGS%
