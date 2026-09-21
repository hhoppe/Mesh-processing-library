#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

VideoViewer results/mechpart.image.bmp $VIDEOVIEWER_ARGS
