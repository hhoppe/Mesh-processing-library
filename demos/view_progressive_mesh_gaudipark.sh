#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'Use the left button to drag the slider (logarithmic in #vertices).'
echo '.'

export PM_LOD_LEVEL="0.03"

g3d_cmp -g3d pm_view -win1 "-key De" results/gaudipark.pm{,} -st data/imageup.s3d \
        -lightambient 1 -lightsource 0 $G3D_ARGS
