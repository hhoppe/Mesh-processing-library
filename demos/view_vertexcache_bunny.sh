#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo .
echo Drag down with the middle mouse button (or turn the wheel) to have a closer look.
echo .

G3dOGL results/bunny.vertexcache.m -st data/bunny.s3d -key DmDTDC $G3DARGS
