#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# Reconstruct the surface in three phases (Recon, Meshfit, and Subdivfit), recording the stages for the animation.
RECON_ROOT=distcap RECON_PTS=data/distcap.pts RECON_RESULTS=results RECON_SAMPLINGD=.02 RECON_ANGLE=52 \
  RECON_RECORD=1 recon/recon_process

echo
echo 'Use view_reconstruction_animated.sh to view the results.'
echo
