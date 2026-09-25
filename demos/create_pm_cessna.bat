@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo From original mesh data/cessna.orig.m, creating progressive mesh results/cessna.pm
echo (This will take a few minutes.)

call mesh_to_pm.bat data/cessna.orig.m >results/cessna.pm

echo .
echo Use view_pm_cessna.bat to view the resulting progressive mesh.
echo .
