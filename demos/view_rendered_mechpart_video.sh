#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

VideoViewer data/mechpart.video.mp4 -key i
