@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo Converting gaudipark image to a colored mesh.

Filterimage data/gaudipark.png -scaletox 200 -tomesh >results/gaudipark.orig.m

echo Given mesh results/gaudipark.orig.m (colors sampled from an image), creating progressive mesh results/gaudipark.pm
echo (This will take a few minutes.)

:: call meshtopm.bat results/gaudipark.orig.m -fit_colors 1 >results/gaudipark.pm
call meshtopm.bat results/gaudipark.orig.m -minqem -norfac 0. -colfac 1. -neptfac 1e5 >results/gaudipark.pm

echo .
echo Use view_pm_gaudipark.bat to view the resulting progressive mesh.
echo .
