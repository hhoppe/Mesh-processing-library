@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo Here we show a real-time visualization of the SphereParam optimization of the bunny mesh using
echo the progressive mesh produced by create_spherical_param_bunny.bat.
echo Press ^<ESC^> to close the window and continue.
echo.
:: SphereParam creates its own viewer window, so skip it when hidden (it would only repeat the create step).
if not defined DEMOS_HIDDEN SphereParam results/bunny.for_sphereparam.pm -visualize -wait_on_visualizer -nooutput >nul


:: Show a progressive mesh in four windows: normal-mapped, with its texture coordinates shown as a checkerboard,
:: with its mesh edges, and its normal map itself (on a square, which keeps its own viewpoint).
:: The windows are linked in raster order, and the last one waits for a first frame before opening (-waitframe in
:: g3d_cmp), so the third window turns off its output ('O') only after a delay.  (The script view_*.sh instead
:: simulates the keys '!O', which write a single frame and then turn off the output, but g3d_cmp.bat cannot pass the
:: character '!'.)  The fourth window then restores its own viewpoint (',').
set PM_LOD_LEVEL=0.0145

echo.
echo Here is a progressive mesh with an octaflat-parameterized normal map.
echo.
set normalmap=results/bunny.spheresample.octaflat.unrotated.normalmap.png
call g3d_cmp -g3d pm_view ^
             -win1 "-texturemap %normalmap% -texturenormal 1 -key Dt -lighta .25 -lights .8 -st data/bunny.s3d" ^
             -win2 "-texturemap data/gradchecker256.png -key DeDt -lighta 1 -lights 0 -st data/bunny.s3d" ^
             -win3 "-st data/bunny.s3d -key DU -edgecolor #00b000 -hwdelay .2 -hwkey O" ^
             -win4 "-texturemap %normalmap% -texturenormal 0 -key Dt -st data/imagesquare.s3d -hwdelay .4 -hwkey ," ^
             results/bunny.octaflat.pm results/bunny.octaflat.pm results/bunny.octaflat.pm data/imagesquare.pm ^
             %G3D_ARGS%

echo.
echo Here is a progressive mesh with a longitude-latitude-parameterized normal map.
echo.
set normalmap=results/bunny.spheresample.lonlat.unrotated.normalmap.png
call g3d_cmp -g3d pm_view ^
             -win1 "-texturemap %normalmap% -texturenormal 1 -key Dt -lighta .25 -lights .8 -st data/bunny.s3d" ^
             -win2 "-texturemap data/gradchecker256.png -key DeDt -lighta 1 -lights 0 -st data/bunny.s3d" ^
             -win3 "-st data/bunny.s3d -key DU -edgecolor #00b000 -hwdelay .2 -hwkey O" ^
             -win4 "-texturemap %normalmap% -texturenormal 0 -key Dt -st data/imagesquare.s3d -hwdelay .4 -hwkey ," ^
             results/bunny.lonlat.pm results/bunny.lonlat.pm results/bunny.lonlat.pm data/imagesquare.pm ^
             %G3D_ARGS%
