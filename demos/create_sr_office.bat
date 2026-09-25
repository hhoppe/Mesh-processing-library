@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo (This will take several minutes.)
echo .

FilterPM data/office.pm -nfaces 80000 -outmesh >results/office.nf80000.orig.m
call mesh_to_pm.bat results/office.nf80000.orig.m -vsgeom >results/office.nf80000.sr.pm

echo .
echo Use view_sr_office.bat to view the resulting selectively refinable mesh.
echo .
