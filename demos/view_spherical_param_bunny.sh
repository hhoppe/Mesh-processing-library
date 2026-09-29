#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# Filtermesh results/bunny.sphparam.m -renamekey v sph P | G3D_TWOLIGHTS=1 G3dOGL - -st data/unitsphere_ang.s3d -key DeoJ $G3DARGS

# SphereParam creates its own viewer window, so omit it when hidden; the parameterization still runs.
visualize_args='-visualize -wait_on_visualizer'
if [[ -n $DEMOS_HIDDEN ]]; then
  visualize_args=''
fi

mesh_to_pm data/bunny.orig.m -minqem -vsgeom -dihallow | SphereParam - $visualize_args -nooutput >/dev/null


VideoViewer results/bunny.spheresample.octaflat.unrotated.normalmap.png $VIDEOVIEWER_ARGS


echo '.'
echo 'Press "Dt" to toggle use of normal map.'
echo '.'

G3dOGL results/bunny.spheresample.remesh.m -st data/bunny.s3d -texturemap results/bunny.spheresample.octaflat.unrotated.normalmap.png -texturenormal 1 -key DmDe -hwkey '(DtDe)' -hwdelay 1.0 $G3DARGS


VideoViewer results/bunny.lonlat.unrotated.normalmap.png $VIDEOVIEWER_ARGS


echo '.'
echo 'Here is a progressive mesh, normal-mapped using the longitude-latitude parameterization.'
echo 'Press "De" to toggle visibility of mesh edges.'
echo 'Press "Dt" to toggle use of normal map.'
echo '.'

PM_LOD_LEVEL="0.0145" G3dOGL -pm_mode results/bunny.split_meridian.pm -st data/bunny.s3d -texturemap results/bunny.lonlat.unrotated.normalmap.png -texturenormal 1 -key De -lighta .25 -lights .8 -hwkey '(DtDe)' -hwdelay 1.0 $G3DARGS
