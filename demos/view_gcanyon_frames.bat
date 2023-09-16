@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

:: Depending on speed of graphics card:
set NFACES=8000
set NFACES=20000
set NFACES=40000

:: Depending on memory of graphics card:
set LOADTEXTURE=
set LOADTEXTURE=-key Dt

G3dOGL -eyeob data/unit_frustum.a3d -sr_mode data/gcanyon_4k2k.l1.srm -texturemap data/gcanyon_color_4k2k.png -sr_screen_thresh 0.003846 -lightambient .7 -st data/gcanyon_4k2k_fly2.s3d -sr_regulatenf %NFACES% -sr_fracvtrav .3333 -sr_gain .3333 -key De0------l -hither .0001 %LOADTEXTURE% %G3DARGS% %TEXGEOMETRY% -input <data/gcanyon_4k2k_fly4.edited.frames
