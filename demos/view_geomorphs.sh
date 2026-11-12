#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'Move slider up and down to see geomorphs.'
echo
echo 'Move slider to bottom, press "PDe" to toggle edges off.  Again move slider up and down.'
echo
echo 'Move slider to top.  Move object backward/forward (middle button or wheel), adjusting LOD using slider.'
echo

export LOD_LEVEL="0.3"
export G3D_DOLLY_LOD=1

G3dOGL results/standingblob.geomorphs -st data/standingblob.s3d -key PDeS -lightambient .5 -thickboundary 1 \
       -sharpedgecolor '#FF000000' $G3D_GEOM $G3D_ARGS
