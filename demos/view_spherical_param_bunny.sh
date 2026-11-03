#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh


# Show the optimization of SphereParam on the progressive mesh created by create_spherical_param_bunny.sh.
# SphereParam creates its own viewer window, so skip it when hidden (it would only repeat the create step).
if [[ -z $DEMOS_HIDDEN ]]; then
  SphereParam results/bunny.for_sphereparam.pm -visualize -wait_on_visualizer -nooutput >/dev/null
fi


VideoViewer results/bunny.spheresample.octaflat.unrotated.normalmap.png $VIDEOVIEWER_ARGS

g3d_cmp -win1 "-key ----J -key DmDe" -win2 "-key DmDt" results/bunny.spheresample.remesh.m{,} -st data/bunny.s3d -texturemap results/bunny.spheresample.octaflat.unrotated.normalmap.png -texturenormal 1 $G3D_ARGS


VideoViewer results/bunny.lonlat.unrotated.normalmap.png $VIDEOVIEWER_ARGS

echo '.'
echo 'Here is a progressive mesh, normal-mapped using the longitude-latitude parameterization.'
echo 'Press "De" to toggle visibility of mesh edges.'
echo 'Press "Dt" to toggle use of normal map.'
echo '.'

PM_LOD_LEVEL="0.0145" G3dOGL -pm_mode results/bunny.split_meridian.pm -st data/bunny.s3d -texturemap results/bunny.lonlat.unrotated.normalmap.png -texturenormal 1 -key De -lighta .25 -lights .8 -hwkey '(DtDe)' -hwdelay 1.0 $G3D_GEOM $G3D_ARGS
