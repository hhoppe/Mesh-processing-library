#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'Press "De" to toggle edges on/off.'
echo 'Or, press "Ds" to toggle faces on/off.'
echo '.'
echo 'Use left button to drag slider (logarithmic in #vertices).'
echo '.'

export PM_LOD_LEVEL="0.1"

pm_view results/cessna.pm -st data/cessna.s3d -lightambient .4 $G3D_GEOM $G3D_ARGS
