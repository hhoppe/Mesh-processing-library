#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo 'From progressive mesh data/standingblob.pm, extracting the mesh results/standingblob.nf20000.m with 20000'
echo 'faces, and creating its progressive outer hull results/standingblob.ohull.pm.'

FilterPM data/standingblob.pm -nf 20000 -outm | Filtermesh -removekey normal -removekey wid >results/standingblob.nf20000.m
mesh_to_pm results/standingblob.nf20000.m -hull -innerhull 0 -af 3 -minvolume -relerror -attrib_project >results/standingblob.ohull.pm

echo '.'
echo 'Use view_proghull_standingblob.sh to view the resulting progressive hull.'
echo '.'
