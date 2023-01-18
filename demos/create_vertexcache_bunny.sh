#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

# MeshReorder ~/data/mesh/bunny.orig.m -fifo -analyze -color_corners 1 >v0.m

echo 'From original mesh data/bunny.orig.m, creating data/bunny.vertexcache.m with optimized face ordering.'
MeshReorder data/bunny.orig.m -fifo -cache_size 16 -analyze -meshify5 -color_corners 1 -analyze >data/bunny.vertexcache.m

echo '.'
echo 'Use view_vertexcache_bunny.sh to see the optimized mesh traversal order.'
echo '.'
