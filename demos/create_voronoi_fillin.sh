#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

Filterimage data/texture.input.png -color 255 0 0 255 -voronoidilate >results/texture.output.png

echo 'Use view_voronoi_fillin.sh to see the result in results/texture.output.png'
