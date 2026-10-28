#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo 'From original mesh data/bunny.orig.m, creating the progressive outer hull results/bunny.outer_hull.pm, a'
echo 'progressive mesh whose every level of detail encloses the original mesh, by solving a linear program at'
echo 'each edge collapse.'

mesh_to_pm data/bunny.orig.m -hull -innerhull 0 -af 3 -minvolume -relerror -attrib_project >results/bunny.outer_hull.pm

echo '.'
echo 'Use view_progressive_hull_bunny.sh to view the resulting progressive hull.'
echo '.'
