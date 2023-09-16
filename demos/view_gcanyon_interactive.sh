#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo '.'
echo 'MAIN key:'
echo 'Position cursor in center of window, then press "f" to toggle flying off/on.'
echo '.'
echo 'EMERGENCY key:'
echo ' Press "," to return to the starting viewpoint.'
echo '.'
echo 'EXPLANATION:'
echo ' Press "Dt" (shift-D, little t) to toggle texture and mesh edges.'
echo ' Press "Dr" to toggle top "radar" view.'
echo '.'
echo 'FUN:'
echo ' Press "Dg" to turn on geomorphs, then'
echo '  press "Dc" to get coarsest mesh, then'
echo '  press "Df" to return to current mesh with geomorphs.'
echo '.'
echo 'DETAILED:'
echo ' Press "-" and "=" to adjust flying rate.'
echo ' Press "Dg" to toggle geomorphs on/off (see using mesh view).'
echo '.'
echo 'Note: ignore the assertion warnings below.'
echo '.'

# Depending on speed of graphics card.
nfaces=8000
nfaces=20000
nfaces=40000

# Depending on memory of graphics card.
loadtexture="-key Dt"
# loadtexture=""

G3dOGL -eyeob data/unit_frustum.a3d -sr_mode data/gcanyon_4k2k.l1.srm -texturemap data/gcanyon_color_4k2k.png -sr_screen_thresh 0.003846 -lightambient .7 -st data/gcanyon_4k2k_fly2.s3d -sr_regulatenf $nfaces -sr_fracvtrav .3333 -sr_gain .3333 -key De0------l -hither .0001 $loadtexture $G3DARGS $TEXGEOMETRY
