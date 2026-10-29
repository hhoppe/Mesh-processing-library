#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

G3dOGL --version

# G3dOGL data/cactus.pts results/cactus.recon.m results/cactus.opt.m results/cactus.sub0.m results/cactus.sub2limit.m -st data/cactus.s3d -key DbNNDmDeNDmDeNDmDePPP--oJ -key NNNNN -backcolor hhblue $G3D_GEOM $G3D_ARGS

g3d_cmp -win1 "-key --oJ" -winl "-key DmDeDb" data/cactus.pts results/cactus.recon.m results/cactus.opt.m results/cactus.sub0.m results/cactus.sub2limit.m -st data/cactus.s3d -backcolor hhblue -key DmDeDb $G3D_ARGS
