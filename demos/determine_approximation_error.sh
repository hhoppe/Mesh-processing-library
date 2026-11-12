#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'Compute the approximation error between original bunny and its progressive outer hull at 10000 faces.'
echo
FilterPM results/bunny.outer_hull.pm -nfaces 10000 -outmesh >results/bunny.outer_hull.nf10000.m

echo
echo 'Columns are: geometric error, color error, vertex_normal error'
echo 'Rows are: r=random, v=vertices, 0/1=one_way errors, B=both_way errors'
echo ' L2=root_mean_square_error,  Li=maximum_error'
echo
# The saved output is compared against reference values by check_created_outputs.sh.
MeshDistance -mfile data/bunny.orig.m -mfile results/bunny.outer_hull.nf10000.m -bothdir 1 -maxerror 1 -verb 2 \
             -distance >&results/bunny.outer_hull.approximation_error.txt
cat results/bunny.outer_hull.approximation_error.txt

rm -f results/bunny.outer_hull.nf10000.m
