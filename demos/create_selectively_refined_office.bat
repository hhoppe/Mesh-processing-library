@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo We extract an 80,000-triangle mesh from the progressive mesh.
echo.
FilterPM data/office.pm -nfaces 80000 -outmesh >results/office.nf80000.orig.m

echo.
echo We construct a progressive mesh with -vsgeom (half-edge collapses) to allow selective refinement.
echo.
call mesh_to_pm.bat results/office.nf80000.orig.m -vsgeom >results/office.nf80000.sr.pm

echo.
echo Use view_selectively_refined_office.bat to view the resulting selectively refinable mesh.
echo.
