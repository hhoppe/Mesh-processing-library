#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo 'From original mesh data/cessna.orig.m, creating progressive mesh results/cessna.pm'
echo '(This will take a few minutes.)'

mesh_to_pm data/cessna.orig.m >results/cessna.pm

echo '.'
echo 'Use view_pm_cessna.sh to view the resulting progressive mesh.'
echo '.'
