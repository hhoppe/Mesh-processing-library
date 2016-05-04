#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

# (G3dOGL data/blob5.orig.m -st data/blob5.s3d -key DmDe &)
# G3dOGL data/blob5.meshopt.simplified.m -st data/blob5.s3d -key DmDe $G3DARGS

# G3dcmp data/blob5.orig.m data/blob5.meshopt.simplified.m -st data/blob5.s3d -key DmDe

: ; \
  G3dOGL -geom 800x820+100+10 -key DmDe -key O -key ,o----J    -st data/blob5.s3d data/blob5.orig.m | \
  G3dOGL -geom 800x820+920+10 -key DmDe -async -killeof -input -st data/blob5.s3d data/blob5.meshopt.simplified.m
