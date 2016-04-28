@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo Converting gaudipark image to a colored mesh.

Filterimage data/gaudipark.png -scaletox 200 -tomesh >data/gaudipark.orig.m

echo Given mesh data/gaudipark.orig.m (colors sampled from an image), creating progressive mesh data/gaudipark.pm
echo (This will take a few minutes.)

:: call bin/meshtopm.bat data/gaudipark.orig.m -minqem -norfac 0. -colfac 1. -neptfac 1e5 >data/gaudipark.pm
call bin/meshtopm.bat data/gaudipark.orig.m >data/gaudipark.pm

echo .
echo Use view_pm_gaudipark.bat to view the resulting progressive mesh.
echo .
