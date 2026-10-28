@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo Press 'De' to toggle edges on/off.
echo .
echo Use right button to drag slider up/down.
echo .

set PM_LOD_LEVEL=0.01

G3dOGL -pm_mode results/standingblob.ohull.pm -st data/standingblob.s3d -key ,De %G3DARGS%
