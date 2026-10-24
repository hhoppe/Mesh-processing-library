@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo Press 'De' to toggle edges on/off.
echo Or, press 'Ds' to toggle faces on/off.
echo .
echo Use left button to drag slider (logarithmic in #vertices).
echo .

set PM_LOD_LEVEL=0.1

G3dOGL -pm_mode results/club.pm -st data/club.s3d -key , -lightambient .4 %G3DARGS%
