@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo From original mesh data/club.orig.m, creating progressive mesh results/club.pm using memoryless
echo QEM simplification with volume preservation and normal-field preservation.
echo (This will take a few minutes.)

call bin/meshtopm.bat data/club.orig.m >results/club.pm

echo .
echo Use view_pm_club.bat to view the resulting progressive mesh.
echo .
