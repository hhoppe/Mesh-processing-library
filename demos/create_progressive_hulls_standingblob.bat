@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo From progressive mesh data/standingblob.pm, extracting the mesh results/standingblob.nf20000.m with 20000
echo faces, and creating its progressive outer hull results/standingblob.outer_hull.pm.

FilterPM data/standingblob.pm -nf 20000 -outm | Filtermesh -removekey normal -removekey wid >results/standingblob.nf20000.m
call mesh_to_pm.bat results/standingblob.nf20000.m -hull -innerhull 0 -af 3 -minvolume -relerror -attrib_project >results/standingblob.outer_hull.pm

echo Also creating the progressive inner hull results/standingblob.inner_hull.pm, whose every level of detail lies
echo inside that mesh.

call mesh_to_pm.bat results/standingblob.nf20000.m -hull -innerhull 1 -af 3 -minvolume -relerror -attrib_project >results/standingblob.inner_hull.pm

echo .
echo Use view_progressive_hulls_standingblob.bat to view the resulting progressive hulls.
echo .
