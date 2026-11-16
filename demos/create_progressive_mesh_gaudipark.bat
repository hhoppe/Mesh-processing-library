@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo We convert the gaudipark image to a colored mesh.
echo.
Filterimage data/gaudipark.png -scaletox 200 -best_diagonal -tomesh >results/gaudipark.orig.m

echo.
echo Given mesh results/gaudipark.orig.m (colors sampled from an image), we create the progressive mesh
echo results/gaudipark.pm.
echo.
MeshToPm results/gaudipark.orig.m -norfac 0 -colfac 1 -neptfac 1e5 >results/gaudipark.pm

echo.
echo Use view_progressive_mesh_gaudipark.bat to view the resulting progressive mesh.
echo.
