@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo Note that the mesh genus has been reduced from 50 to 4.
echo Many tiny topological handles have been closed.
echo .
echo To visualize these, press 'Ds' to toggle surface rendering off/on.
echo .

G3dOGL data/office.nf200000.mincycles.m -st data/office.s3d -key DeDEJ---- -thickboundary 0 -lightambient .9 -lightsource .4
