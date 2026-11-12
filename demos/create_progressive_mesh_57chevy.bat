@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo From the original mesh data/57chevy.orig.m, we create the progressive mesh results/57chevy.pm using memoryless
echo QEM simplification with volume preservation and normal-field preservation.
echo.
call mesh_to_pm.bat data/57chevy.orig.m -minqem >results/57chevy.pm

echo.
echo Use view_progressive_mesh_57chevy.bat to view the resulting progressive mesh.
echo.
