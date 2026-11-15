#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'From the original mesh data/57chevy.orig.m, we create the progressive mesh results/57chevy.pm using memoryless'
echo 'QEM simplification with half-edge collapses and normal-field preservation.'
echo
roundtrip_check=-roundtrip_check  # (Optional) Verify that the progressive mesh reproduces the original mesh.
MeshToPm data/57chevy.orig.m $roundtrip_check >results/57chevy.pm

echo
echo 'Use view_progressive_mesh_57chevy.sh to view the resulting progressive mesh.'
echo
