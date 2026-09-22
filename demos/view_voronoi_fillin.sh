#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

VideoViewer results/texture.output.png $VIDEOVIEWER_ARGS
