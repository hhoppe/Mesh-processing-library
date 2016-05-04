#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo '.'
echo 'Compute the approximation error between original club and club simplified to 1000 faces.'
echo '.'

FilterPM data/club.pm -nfaces 1000 -outmesh >data/club.nf1000.m

echo '.'
echo 'Columns are: geometric error, color error, vertex_normal error'
echo 'Rows are: r=random, v=vertices, 0/1=one_way errors, B=both_way errors'
echo ' L2=root_mean_square_error,  Li=maximum_error'
echo '.'

MeshDistance -mfile data/club.orig.m -mfile data/club.nf1000.m -bothdir 1 -maxerror 1 -verb 2 -distance

rm -f data/club.nf1000.m
