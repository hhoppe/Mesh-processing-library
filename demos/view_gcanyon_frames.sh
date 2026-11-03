#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# Depending on speed of graphics card.
nfaces=8000
nfaces=20000
nfaces=40000

# Depending on memory of graphics card.
loadtexture="-key Dt"
# loadtexture=""

export ASSERTW_IGNORE='fl,fr._matid ignored'

# When hidden, run longer than the other viewers (4 seconds), so that the terrain is rendered along the frames.
hidden_hold=''
if [[ -n $DEMOS_HIDDEN ]]; then hidden_hold='-hwkey \7\c'; fi

G3dOGL -eyeob data/unit_frustum.a3d -sr_mode data/gcanyon_4k2k.l1.srm -texturemap data/gcanyon_color_4k2k.png -sr_screen_thresh 0.003846 -lightambient .7 -st data/gcanyon_4k2k_fly2.s3d -sr_regulatenf $nfaces -sr_fracvtrav .3333 -sr_gain .3333 -key De0------l -hither .0001 $loadtexture $G3D_GEOM $G3D_ARGS $hidden_hold -input <data/gcanyon_4k2k_fly4.edited.frames
