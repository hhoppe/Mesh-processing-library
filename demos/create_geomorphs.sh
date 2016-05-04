#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

# This is how the file was converted from Blender via *.ply:
# perl bin/plytoMesh.pl data/standingblob.ply | Filtermesh -genus -gmerge -genus -flip -triangulate -angle 20 -mark -genus >data/standingblob.orig.m

FilterPM data/standingblob.pm -geom_nfaces 10 -geom_nfaces 20 -geom_nfaces 40 -geom_nfaces 80 -geom_nfaces 160 -geom_nfaces 320 -geom_nfaces 640 -geom_nfaces 1280 -geom_nfaces 2560 -geom_nfaces 5000 -geom_nfaces 10000 -geom_nfaces 20000 >data/standingblob.geomorphs

echo '.'
echo 'Use view_geomorphs.sh to view the resulting geomorphs.'
echo '.'
