#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo 'Running phase 1 - Recon.'
Recon <data/distcap.pts -samplingd 0.02 | Filtermesh -genus -fillholes 30 -triangulate -genus >results/distcap.recon.m

echo 'Running phase 2 - Meshfit.'
Meshfit -mfile results/distcap.recon.m -file data/distcap.pts -crep 1e-5 -reconstruct >results/distcap.opt.m

echo 'Running phase 3 - Subdivfit.'
# Initially tagging edges sharp if dihedral angle exceeds 52 degrees.
Filtermesh results/distcap.opt.m -angle 52 -mark | Subdivfit -mfile - -file data/distcap.pts -crep 1e-5 -csharp .2e-5 -reconstruct >results/distcap.sub0.m

echo 'Computing final subdivided surface.'
Subdivfit -mfile results/distcap.sub0.m -nsub 2 -outn >results/distcap.sub2limit.m

echo 'Use view_reconstruction_distcap.sh to view the results.'
