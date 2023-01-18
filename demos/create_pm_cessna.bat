@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo From original mesh data/cessna.orig.m, creating progressive mesh data/cessna.pm
echo (This will take a few minutes.)

call bin/meshtopm.bat data/cessna.orig.m >data/cessna.pm

echo .
echo Use view_pm_cessna.bat to view the resulting progressive mesh.
echo .
