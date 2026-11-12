#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'We extract an 80,000-triangle mesh from the progressive mesh.'
echo
FilterPM data/office.pm -nfaces 80000 -outmesh >results/office.nf80000.orig.m

echo
echo 'We construct a progressive mesh with -vsgeom (half-edge collapses) to allow selective refinement.'
echo
mesh_to_pm results/office.nf80000.orig.m -vsgeom >results/office.nf80000.sr.pm

echo
echo 'Use view_selectively_refined_office.sh to view the resulting selectively refinable mesh.'
echo
