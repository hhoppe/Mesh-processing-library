#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo 'To take interactive control, select the first window, press "o" to remove object-centric motion,'
echo '  then drag using any of the mouse buttons.'
echo '.'

files=(data/distcap.pts results/distcap.recon.m results/distcap.opt.m)
extra_args=()

if [ -f results/distcap.sub0.m ]; then
  files+=(results/distcap.sub0.m results/distcap.sub2limit.m)
  extra_args+=(-winl "-key DmDe")
fi

g3d_cmp -win1 "-key o--J" "${extra_args[@]}" "${files[@]}" -backcolor hhblue -key DbDmDe -st data/distcap.s3d $G3D_ARGS
