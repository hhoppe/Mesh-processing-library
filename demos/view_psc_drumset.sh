#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'Use left button to drag slider up/down.'
echo '.'

export PSC_LOD_LEVEL="0.1"

# Display lists are off ("DC") because the vertex spheres are tessellated according to their size on the screen.
G3dOGL -psc_mode data/drumset_trunc.psc data/drumset.box.a3d -st data/drumset.s3d -key DC \
       -lightambient .4 $G3D_GEOM $G3D_ARGS
