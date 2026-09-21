#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

# (G3dOGL data/blob5.orig.m -st data/blob5.s3d -key DmDe &)
# G3dOGL results/blob5.meshopt.simplified.m -st data/blob5.s3d -key DmDe $G3DARGS

# G3dcmp data/blob5.orig.m results/blob5.meshopt.simplified.m -st data/blob5.s3d -key DmDe

G3dOGL $G3DARGS -geom 800x820+150+10 -key DmDe -key O -key ,o----J -st data/blob5.s3d data/blob5.orig.m |
  G3dOGL $G3DARGS -geom 800x820+970+10 -key DmDe -async -killeof -input -st data/blob5.s3d results/blob5.meshopt.simplified.m
