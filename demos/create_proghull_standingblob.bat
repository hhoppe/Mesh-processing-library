@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo From progressive mesh data/standingblob.pm, extracting the mesh results/standingblob.nf20000.m with 20000
echo faces, and creating its progressive outer hull results/standingblob.ohull.pm.

FilterPM data/standingblob.pm -nf 20000 -outm | Filtermesh -removekey normal -removekey wid >results/standingblob.nf20000.m
call mesh_to_pm.bat results/standingblob.nf20000.m -hull -innerhull 0 -af 3 -minvolume -relerror -attrib_project >results/standingblob.ohull.pm

echo .
echo Use view_proghull_standingblob.bat to view the resulting progressive hull.
echo .
