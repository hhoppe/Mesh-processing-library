#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'We convert the gaudipark image to a colored mesh.'
echo
Filterimage data/gaudipark.png -scaletox 200 -tomesh >results/gaudipark.orig.m

echo 'Given mesh results/gaudipark.orig.m (colors sampled from an image), we create the progressive mesh'
echo 'results/gaudipark.pm.'
echo
# mesh_to_pm results/gaudipark.orig.m -fit_colors 1 >results/gaudipark.pm
mesh_to_pm results/gaudipark.orig.m -minqem -norfac 0. -colfac 1. -neptfac 1e5 >results/gaudipark.pm

echo
echo 'Use view_progressive_mesh_gaudipark.sh to view the resulting progressive mesh.'
echo
