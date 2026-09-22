#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'Press "De" to toggle edges on/off.'
echo 'Or, press "Ds" to toggle faces on/off.'
echo '.'
echo 'Use right button to drag slider exponentially.'
echo '.'

export PM_LOD_LEVEL="0.1"

G3dOGL -pm_mode results/club.pm -st data/club.s3d -key , -lightambient .4 $G3DARGS
