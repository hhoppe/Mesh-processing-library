@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo From original mesh data/club.orig.m, creating progressive mesh data/club.pm
echo (This will take a few minutes.)

call bin/meshtopm.bat data/club.orig.m >data/club.pm

echo .
echo Use view_pm_club.bat to view the resulting progressive mesh.
echo .
