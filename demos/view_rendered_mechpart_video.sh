#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

VideoViewer results/mechpart.video.mp4 -key i $VIDEOVIEWER_ARGS
