#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'To take interactive control, select the first window, press "o" to remove object-centric motion,'
echo '  then drag using any of the mouse buttons.'
echo '.'

g3d_cmp -win1 "-key o--J" -winl "-key DmDe" data/distcap.pts results/distcap.recon.m results/distcap.opt.m \
  results/distcap.sub0.m results/distcap.sub2limit.m -backcolor hhblue -key DbDmDe -st data/distcap.s3d $G3D_ARGS
