#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

if [[ ${BASH_VERSINFO[5]} == *-apple-* ]]; then
  # For some reason the "-offscreen" approach gives all-black on Mac.
  G3dOGL data/mechpart.recon.m -st data/mechpart.s3d -imagename results/mechpart.image.bmp -picture $G3DARGS -geometry 800x800+150+0
else
  G3dOGL data/mechpart.recon.m -st data/mechpart.s3d -offscreen results/mechpart.image.bmp -noinfo 1 $G3DARGS -geometry 800x800+150+0
fi

echo 'File results/mechpart.image.bmp is now created; it can be viewed using view_rendered_mechpart_image.sh'
