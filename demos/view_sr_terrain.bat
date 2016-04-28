@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo The left window shows instantaneous refinement, whereas the right window shows smooth geomorphs.
echo .
echo Press 'J' in left window to toggle motion off/on.
echo Press 'Dt' in either window to toggle mesh edges on/off.
echo Press 'Dr' in either window to toggle top view.
echo .
echo Press ',' in left window to restart to a good viewpoint.
echo .

set G3D_REV_AUTO=1

set COMMON= -eyeob data/unit_frustum.a3d -sr_mode data/gcanyon_sq200.pm -st data/gcanyon_fly_v98.s3d -texturemap data/gcanyon_color.1024.png -key DeDtDG -sr_screen_thresh .02292 -sr_gtime 64 -lightambient .5

G3dOGL %COMMON% -geom 800x820+100+10 -key "&O" -key ,o----J | G3dOGL %COMMON% -geom 800x820+920+10 -async -killeof -input -key Dg
