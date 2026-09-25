#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo '.'
echo '(This will take several minutes.)'
echo '.'

FilterPM data/office.pm -nfaces 80000 -outmesh >results/office.nf80000.orig.m
mesh_to_pm results/office.nf80000.orig.m -vsgeom >results/office.nf80000.sr.pm

echo '.'
echo 'Use view_sr_office.sh to view the resulting selectively refinable mesh.'
echo '.'
