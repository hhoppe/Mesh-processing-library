@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

:: The input stream of frames was designed for a display refresh rate of 60 Hz, hence "-input_fps 60".

:: Depending on the speed of the graphics card.
set NFACES=40000

set "ASSERTW_IGNORE=fl,fr._matid ignored"

:: When hidden, run longer than the other viewers (4 seconds), so that the terrain is rendered along the frames.
set HIDDEN_HOLD=
if defined DEMOS_HIDDEN set HIDDEN_HOLD=-hwkey \7\c

G3dOGL -eyeob data/unit_frustum.a3d -sr_mode data/gcanyon_4k2k.l1.srm -texturemap data/gcanyon_color_4k2k.png ^
       -sr_screen_thresh 0.003846 -lightambient .7 -st data/gcanyon_4k2k_fly2.s3d ^
       -sr_regulatenf %NFACES% -sr_fracvtrav .3333 -sr_gain .3333 -key De0------lDt ^
       -hither .0001 -input_fps 60 %G3D_GEOM% %G3D_ARGS% %HIDDEN_HOLD% -input <data/gcanyon_4k2k_fly4.edited.frames
