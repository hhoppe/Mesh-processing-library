@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo This demo shows the input points (white), the initial curve reconstructed (brown),
echo  and the final fitted curve (white).
echo .
echo Press 'N' and 'P' to manually advance to next and previous objects.
echo .

G3dOGL data/curve1.pts data/curve1.recon.a3d data/curve1.opt.a3d -key ojo -backcolor white %G3DARGS%
