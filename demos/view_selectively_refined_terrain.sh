#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'The left window shows instantaneous refinement, whereas the right window shows smooth geomorphs.'
echo '.'
echo 'Press "J" in left window to toggle motion off/on.'
echo 'Press "Dt" in either window to toggle mesh edges on/off.'
echo 'Press "Dr" in either window to toggle top view.'
echo '.'
echo 'Press "," in left window to restart to a good viewpoint.'
echo '.'

export G3D_REV_AUTO=1

g3d_cmp -win1 "-key ,o----J" -win2 "-key Dg" -g3d "G3dOGL -sr_mode" results/gcanyon_sq200.pm{,} -eyeob data/unit_frustum.a3d -st data/gcanyon_fly_v98.s3d -texturemap data/gcanyon_color.1024.png -key DeDtDG -sr_screen_thresh .02292 -sr_gtime 64 -lightambient .5 $G3D_ARGS
