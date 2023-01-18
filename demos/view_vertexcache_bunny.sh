#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo .
echo Drag down with the right mouse button to have a closer look.
echo .

G3dOGL data/bunny.vertexcache.m -st data/bunny.s3d -key DmDTDTDC $G3DARGS
