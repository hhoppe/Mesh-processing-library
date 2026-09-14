#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo '.'
echo 'Press "P" and "N" to advance to the previous and next models in the reconstruction sequence.'
echo '.'
echo 'Press "J" to stop automatic rotation.'
echo '.'

G3dOGL --version

# G3dOGL data/cactus.pts results/cactus.recon.m results/cactus.opt.m results/cactus.sub0.m results/cactus.sub2limit.m -st data/cactus.s3d -key NNDmDeNDmDeNDmDePPP--oJ -hwdelay 1.5 -hwkey NNNNN -backcolor hhblue $G3DARGS

G3dOGL data/cactus.pts results/cactus.recon.m results/cactus.opt.m results/cactus.sub0.m results/cactus.sub2limit.m -st data/cactus.s3d -key DbNNDmDeNDmDeNDmDePPP--oJ -key NNNNN -backcolor hhblue $G3DARGS
