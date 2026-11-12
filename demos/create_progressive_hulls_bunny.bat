@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo From the original mesh data/bunny.orig.m, we create the progressive outer hull results/bunny.outer_hull.pm,
echo a progressive mesh whose every level of detail encloses the original mesh, by solving a linear program at
echo each edge collapse.
echo.
call mesh_to_pm.bat data/bunny.orig.m -hull -innerhull 0 -af 3 -minvolume -relerror -attrib_project ^
                    >results/bunny.outer_hull.pm

echo.
echo We also create the progressive inner hull results/bunny.inner_hull.pm, whose every level of detail lies
echo inside the original mesh.
echo.
call mesh_to_pm.bat data/bunny.orig.m -hull -innerhull 1 -af 3 -minvolume -relerror -attrib_project ^
                    >results/bunny.inner_hull.pm

echo.
echo Use view_progressive_hulls_bunny.bat to view the resulting progressive hulls.
echo.
