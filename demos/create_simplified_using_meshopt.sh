#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# This is how the file was converted from Blender via *.ply:
# plytoMesh data/blob5.ply | Filtermesh -genus -gmerge -genus -triangulate -angle 20 -mark -genus >data/blob5.orig.m

echo 'Running Meshfit.'
Filtermesh data/blob5.orig.m -randpts 10000 -vertexpts | Meshfit -mfile data/blob5.orig.m -fi - -crep 1e-5 -simplify >results/blob5.meshopt.simplified.m

echo 'Use view_simplified_using_meshopt.sh to see the results.'
