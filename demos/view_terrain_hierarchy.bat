@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo Move slider up/down to visualize hierarchical simplification.
echo .

set PM_LOD_LEVEL=0.27

G3dOGL -pm_mode data/gcanyon_sq129_b44.l2.pm -st data/gcanyon_sq129_b44_video.s3d -key ,De -lightambient .4 %G3DARGS%
