#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'Here we show a real-time visualization of the SphereParam optimization of the bunny mesh using'
echo 'the progressive mesh produced by create_spherical_param_bunny.sh.'
echo 'Press <ESC> to close the window and continue.'
echo
# SphereParam creates its own viewer window, so skip it when hidden (it would only repeat the create step).
if [[ -z $DEMOS_HIDDEN ]]; then
  SphereParam results/bunny.for_sphereparam.pm -visualize -wait_on_visualizer -nooutput >/dev/null
fi


# Show the progressive mesh results/bunny.$1.pm in four windows: normal-mapped, with its texture coordinates shown as
# a checkerboard, with its mesh edges, and its normal map itself (on a square, which keeps its own viewpoint).
# The windows are linked in raster order, and the last one waits for a first frame before opening (-waitframe in
# g3d_cmp), so the third window writes a single frame ('!') and then turns off its output ('O').
show_normal_mapped() {
  local normalmap=results/bunny.spheresample.$1.unrotated.normalmap.png
  PM_LOD_LEVEL=0.0145 g3d_cmp -g3d pm_view \
              -win1 "-texturemap $normalmap -texturenormal 1 -key Dt -lighta .25 -lights .8 -st data/bunny.s3d" \
              -win2 "-texturemap data/gradchecker256.png -key DeDt -lighta 1 -lights 0 -st data/bunny.s3d" \
              -win3 "-st data/bunny.s3d -key DU!O -edgecolor #00b000" \
              -win4 "-texturemap $normalmap -texturenormal 0 -key Dt -st data/imagesquare.s3d" \
              results/bunny.$1.pm{,,} data/imagesquare.pm $G3D_ARGS
}

echo
echo 'Here is a progressive mesh with an octaflat-parameterized normal map.'
echo
show_normal_mapped octaflat

echo
echo 'Here is a progressive mesh with a longitude-latitude-parameterized normal map.'
echo
show_normal_mapped lonlat
