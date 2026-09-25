#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo 'From original mesh data/club.orig.m, creating progressive mesh results/club.pm using memoryless'
echo 'QEM simplification with volume preservation and normal-field preservation.'
echo '(This will take a few minutes.)'

mesh_to_pm data/club.orig.m -minqem >results/club.pm

echo '.'
echo 'Use view_pm_club.sh to view the resulting progressive mesh.'
echo '.'
