#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# On Mac, "-offscreen" once gave an all-black image, so this demo instead rendered into a window and saved it:
#   G3dOGL data/mechpart.recon.m -st data/mechpart.s3d -imagename results/mechpart.image.bmp -picture $G3DARGS \
#     -geometry 800x800+150+0
# The "-offscreen" approach now gives a correct image on macOS 26 with XQuartz and Apple's software OpenGL (as on
# the GitHub runners), though it is untested with the OpenGL of an actual Mac GPU.
G3dOGL data/mechpart.recon.m -st data/mechpart.s3d -offscreen results/mechpart.image.bmp -noinfo 1 $G3DARGS \
  -geometry 800x800+150+0 || exit 1

echo 'File results/mechpart.image.bmp is now created; it can be viewed using view_rendered_mechpart_image.sh'
