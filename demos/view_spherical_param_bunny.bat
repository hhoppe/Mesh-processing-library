@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

:: Filtermesh results/bunny.sphparam.m -renamekey v sph P | G3dOGL - -st data/unitsphere_ang.s3d -key DeoJ %G3DARGS%


:: Because a pipe command is run in a subshell, the syntax "call script.bat" is unnecessary (and wrong) here.
"bin/meshtopm.bat" data/bunny.orig.m -minqem -vsgeom -dihallow | SphereParam - -visualize -wait_on_visualizer -nooutput >nul


VideoViewer results/bunny.spheresample.octaflat.unrotated.normalmap.png


echo .
echo Press "Dt" to toggle use of normal map.
echo .

G3dOGL results/bunny.spheresample.remesh.m -st data/bunny.s3d -texturemap results/bunny.spheresample.octaflat.unrotated.normalmap.png -texturenormal 1 -key DmDe -hwkey "(DtDe)" -hwdelay 1.0 %G3DARGS%


VideoViewer results/bunny.lonlat.unrotated.normalmap.png


echo .
echo Here is a progressive mesh, normal-mapped using the longitude-latitude parameterization.
echo Press "De" to toggle visibility of mesh edges.
echo Press "Dt" to toggle use of normal map.
echo .

set PM_LOD_LEVEL=0.0145
G3dOGL -pm_mode results/bunny.split_meridian.pm -st data/bunny.s3d -texturemap results/bunny.lonlat.unrotated.normalmap.png -texturenormal 1 -key De -lighta .25 -lights .8 -hwkey "(DtDe)" -hwdelay 1.0 %G3DARGS%
