@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo From original mesh data/bunny.orig.m, creating the progressive outer hull results/bunny.ohull.pm, a
echo progressive mesh whose every level of detail encloses the original mesh, by solving a linear program at
echo each edge collapse.

call mesh_to_pm.bat data/bunny.orig.m -hull -innerhull 0 -af 3 -minvolume -relerror -attrib_project >results/bunny.ohull.pm

echo .
echo Use view_proghull_bunny.bat to view the resulting progressive hull.
echo .
