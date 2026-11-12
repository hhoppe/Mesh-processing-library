@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo Press 'De' to toggle edges on/off.
echo.

set PM_LOD_LEVEL=0.001

call g3d_cmp -g3d pm_view results/bunny.outer_hull.pm results/bunny.inner_hull.pm -st data/bunny.s3d -key De %G3D_ARGS%
