@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo From the original mesh data/57chevy.orig.m, we create the progressive mesh results/57chevy.pm using memoryless
echo QEM simplification with half-edge collapses and normal-field preservation.
echo.
:: (Optional) Verify that the progressive mesh reproduces the original mesh.
set roundtrip_check=-roundtrip_check
MeshToPm data/57chevy.orig.m %roundtrip_check% >results/57chevy.pm

echo.
echo Use view_progressive_mesh_57chevy.bat to view the resulting progressive mesh.
echo.
