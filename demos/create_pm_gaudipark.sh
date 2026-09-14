#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo 'Converting gaudipark image to a colored mesh.'

Filterimage data/gaudipark.png -scaletox 200 -tomesh >results/gaudipark.orig.m

echo 'Given mesh results/gaudipark.orig.m (colors sampled from an image), creating progressive mesh results/gaudipark.pm'
echo '(This will take a few minutes.)'

# bin/meshtopm.sh results/gaudipark.orig.m -fit_colors 1 >results/gaudipark.pm
bin/meshtopm.sh results/gaudipark.orig.m -minqem -norfac 0. -colfac 1. -neptfac 1e5 >results/gaudipark.pm

echo '.'
echo 'Use view_pm_gaudipark.sh to view the resulting progressive mesh.'
echo '.'
