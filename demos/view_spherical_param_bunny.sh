#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

# Filtermesh data/bunny.sphparam.m -renamekey v sph P | G3D_TWOLIGHTS=1 G3dOGL - -st data/unitsphere_ang.s3d -key DeoJ $G3DARGS

bin/meshtopm.sh data/bunny.orig.m -minqem -vsgeom -dihallow | SphereParam - -visualize -nooutput


VideoViewer data/bunny.spheresample.octaflat.unrotated.normalmap.png


echo '.'
echo 'Press "Dt" to toggle use of normal map.'
echo '.'

G3dOGL data/bunny.spheresample.remesh.m -st data/bunny.s3d -texturemap data/bunny.spheresample.octaflat.unrotated.normalmap.png -texturenormal 1 -key DmDe -hwkey '(DtDe)' -hwdelay 1.0 $G3DARGS


VideoViewer data/bunny.lonlat.unrotated.normalmap.png


echo '.'
echo 'Here is a progressive mesh, normal-mapped using the longitude-latitude parameterization.'
echo 'Press "De" to toggle visibility of mesh edges.'
echo 'Press "Dt" to toggle use of normal map.'
echo '.'

PM_LOD_LEVEL="0.005" G3dOGL -pm_mode data/bunny.split_meridian.pm -st data/bunny.s3d -texturemap data/bunny.lonlat.unrotated.normalmap.png -texturenormal 1 -key De -lighta .25 -lights .8 -hwkey '(DtDe)' -hwdelay 1.0 $G3DARGS
