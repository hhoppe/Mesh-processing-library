@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo (This will take several minutes.)
echo .

FilterPM data/office.pm -nfaces 80000 -outmesh >data/office.nf80000.orig.m
call bin/meshtopm.bat data/office.nf80000.orig.m -vsgeom >data/office.nf80000.sr.pm

echo .
echo Use view_sr_office.bat to view the resulting selectively refinable mesh.
echo .
