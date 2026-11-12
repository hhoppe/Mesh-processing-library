#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# MeshReorder ~/data/mesh/bunny.orig.m -fifo -analyze -color_corners 1 >v0.m

echo
echo 'From the original mesh data/bunny.orig.m, we create results/bunny.vertexcache.m with optimized face ordering.'
echo
MeshReorder data/bunny.orig.m -fifo -cache_size 16 -analyze -meshify5 -color_corners 1 \
            -analyze >results/bunny.vertexcache.m

echo
echo 'Use view_vertex_cached_bunny.sh to see the optimized mesh traversal order.'
echo
