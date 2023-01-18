@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo Press 'De' to toggle edges on/off.
echo Or, press 'Ds' to toggle faces on/off.
echo .
echo Use right button to drag slider exponentially.
echo .

set PM_LOD_LEVEL=0.1

G3dOGL -pm_mode data/cessna.pm -st data/cessna.s3d -key , -lightambient .4 %G3DARGS%
