#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'Press "De" to toggle edges on/off.'
echo '.'
echo 'Use right button to drag slider up/down.'
echo '.'

export PM_LOD_LEVEL="0.01"

G3dOGL -pm_mode results/standingblob.ohull.pm -st data/standingblob.s3d -key ,De $G3DARGS
