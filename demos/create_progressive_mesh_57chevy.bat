@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo From original mesh data/57chevy.orig.m, creating progressive mesh results/57chevy.pm using memoryless
echo QEM simplification with volume preservation and normal-field preservation.

call mesh_to_pm.bat data/57chevy.orig.m -minqem >results/57chevy.pm

echo .
echo Use view_progressive_mesh_57chevy.bat to view the resulting progressive mesh.
echo .
