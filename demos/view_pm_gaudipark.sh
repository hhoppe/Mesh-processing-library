#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo '.'
echo 'Press "De" to toggle edges on/off.'
echo '.'
echo 'Use right button to drag slider exponentially.'
echo '.'

export PM_LOD_LEVEL="0.03"

G3dOGL -pm_mode data/gaudipark.pm -st data/imageup.s3d -key ,De -lightambient 1 -lightsource 0 $G3DARGS
