@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo From the original mesh data/cessna.orig.m, we create the progressive mesh results/cessna.pm.
echo.
call mesh_to_pm.bat data/cessna.orig.m >results/cessna.pm

echo.
echo Use view_progressive_mesh_cessna.bat to view the resulting progressive mesh.
echo.
