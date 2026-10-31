@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo From original mesh data/bunny.orig.m, creating the progressive outer hull results/bunny.outer_hull.pm, a
echo progressive mesh whose every level of detail encloses the original mesh, by solving a linear program at
echo each edge collapse.

call mesh_to_pm.bat data/bunny.orig.m -hull -innerhull 0 -af 3 -minvolume -relerror -attrib_project >results/bunny.outer_hull.pm

echo Also creating the progressive inner hull results/bunny.inner_hull.pm, whose every level of detail lies
echo inside the original mesh.

call mesh_to_pm.bat data/bunny.orig.m -hull -innerhull 1 -af 3 -minvolume -relerror -attrib_project >results/bunny.inner_hull.pm

echo .
echo Use view_progressive_hulls_bunny.bat to view the resulting progressive hulls.
echo .
