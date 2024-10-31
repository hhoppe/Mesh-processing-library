#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo 'Converting gaudipark image to a colored mesh.'

Filterimage data/gaudipark.png -scaletox 200 -tomesh >data/gaudipark.orig.m

echo 'Given mesh data/gaudipark.orig.m (colors sampled from an image), creating progressive mesh data/gaudipark.pm'
echo '(This will take a few minutes.)'

# bin/meshtopm.sh data/gaudipark.orig.m -fit_colors 1 >data/gaudipark.pm
bin/meshtopm.sh data/gaudipark.orig.m -minqem -norfac 0. -colfac 1. -neptfac 1e5 >data/gaudipark.pm

echo '.'
echo 'Use view_pm_gaudipark.sh to view the resulting progressive mesh.'
echo '.'
