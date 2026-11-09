#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo 'From original mesh data/57chevy.orig.m, creating progressive mesh results/57chevy.pm using memoryless'
echo 'QEM simplification with volume preservation and normal-field preservation.'

mesh_to_pm data/57chevy.orig.m -minqem >results/57chevy.pm

echo '.'
echo 'Use view_progressive_mesh_57chevy.sh to view the resulting progressive mesh.'
echo '.'
