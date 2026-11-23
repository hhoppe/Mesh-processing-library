#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'Measure the distance between original bunny and its progressive outer hull at 10000 faces.'
echo
FilterPM results/bunny.outer_hull.pm -nfaces 10000 -outmesh >results/bunny.outer_hull.nf10000.m

echo
echo 'Columns are: geometric distance, color difference, vertex_normal difference'
echo 'Rows are: r=random, v=vertices, 0/1=one_way, B=both_ways'
echo ' L2=root_mean_square,  Li=maximum'
echo
# The saved output is compared against reference values by check_created_outputs.sh.
MeshDistance -mfile data/bunny.orig.m -mfile results/bunny.outer_hull.nf10000.m -bothdir 1 -maxerror 1 -verb 2 \
             -distance >&results/bunny.outer_hull.approximation_error.txt
cat results/bunny.outer_hull.approximation_error.txt

rm -f results/bunny.outer_hull.nf10000.m
