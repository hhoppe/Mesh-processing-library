#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'To take interactive control, select the first window, press "o" to remove object-centric motion,'
echo '  then drag using any of the mouse buttons.'
echo '.'

r=distcap
g3d_cmp -win1 "-key o--J" -winl "-key DmDe" data/$r.pts results/$r.recon.m results/$r.opt.m results/$r.sub0.m results/$r.sub2limit.m -st data/$r.s3d -backcolor hhblue -key DmDeDb $G3D_ARGS
