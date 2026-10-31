#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

G3dOGL --version

g3d_cmp -win1 "-key --oJ" -winl "-key DmDeDb" data/mannequin.pts results/mannequin.recon.m results/mannequin.opt.m results/mannequin.sub0.m results/mannequin.sub2limit.m -st data/mannequin.s3d -backcolor hhblue -key DmDeDb $G3D_ARGS
