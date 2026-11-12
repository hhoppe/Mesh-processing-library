#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'Press "De" to toggle edges on/off.'
echo

export PM_LOD_LEVEL="0.001"

g3d_cmp -g3d pm_view results/bunny.{outer,inner}_hull.pm -st data/bunny.s3d -key De $G3D_ARGS
