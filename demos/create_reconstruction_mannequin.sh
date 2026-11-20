#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# Reconstruct the surface in three phases (Recon, Meshfit, and Subdivfit).
RECON_ROOT=mannequin RECON_PTS=data/mannequin.pts RECON_RESULTS=results RECON_SAMPLINGD=5.5 \
  RECON_SUBDIVFIT_PARAMS='-csharp 8e-6' recon/recon_process

echo
echo 'Use view_reconstruction_mannequin.sh to view the results.'
echo
