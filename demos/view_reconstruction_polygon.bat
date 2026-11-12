@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo This demo shows the input points (black), the initial curve reconstructed (brown),
echo  and the final fitted curve (black).
echo.
echo Press 'N' and 'P' to manually advance to next and previous objects.
echo.

G3dOGL data/curve1.pts results/curve1.recon.a3d results/curve1.opt.a3d -st data/curve1.s3d -key ojo ^
       -backcolor hhblue -thickpoint 8 -thicka3d 3 %G3D_GEOM% %G3D_ARGS%
