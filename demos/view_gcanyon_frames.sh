#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# The input stream of frames was designed for a display refresh rate of 60 Hz.  Filterframe doubles it to 120 fps by
# inserting an interpolated frame between each pair, and G3dOGL paces it at "-input_fps 120": a 120 Hz display shows
# every frame, and a 60 Hz display shows every other one, at the same speed.

# Depending on the speed of the graphics card.
nfaces=40000

export ASSERTW_IGNORE='fl,fr._matid ignored'

# When hidden, run longer than the other viewers (4 seconds), so that the terrain is rendered along the frames.
hidden_hold=''
if [[ -n $DEMOS_HIDDEN ]]; then hidden_hold='-hwkey \7\c'; fi

# (A file rather than a pipe, since the viewer may quit before reading all the frames.)
Filterframe -add_mid_frames -frame <data/gcanyon_4k2k_fly4.edited.frames >results/gcanyon_4k2k_fly4.120fps.frames

G3dOGL -eyeob data/unit_frustum.a3d -sr_mode data/gcanyon_4k2k.l1.srm -texturemap data/gcanyon_color_4k2k.png \
       -sr_screen_thresh 0.003846 -lightambient .7 -st data/gcanyon_4k2k_fly2.s3d \
       -sr_regulatenf $nfaces -sr_fracvtrav .3333 -sr_gain .3333 -key De0------lDt \
       -hither .0001 -input_fps 120 $G3D_GEOM $G3D_ARGS $hidden_hold -input <results/gcanyon_4k2k_fly4.120fps.frames
