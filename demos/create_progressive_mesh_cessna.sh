#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'From the original mesh data/cessna.orig.m, we create the progressive mesh results/cessna.pm.'
echo
mesh_to_pm data/cessna.orig.m >results/cessna.pm

echo
echo 'Use view_progressive_mesh_cessna.sh to view the resulting progressive mesh.'
echo
