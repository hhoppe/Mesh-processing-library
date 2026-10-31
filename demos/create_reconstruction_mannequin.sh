#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo 'Running phase 1 - Recon.'
Recon <data/mannequin.pts -samplingd 5.5 | Filtermesh -genus -fillholes 20 -triangulate -genus >results/mannequin.recon.m

echo 'Running phase 2 - Meshfit.'
Meshfit -mfile results/mannequin.recon.m -file data/mannequin.pts -crep 1e-5 -reconstruct >results/mannequin.opt.m

echo 'Running phase 3 - Subdivfit.'
# Initially tagging edges sharp if dihedral angle exceeds 40 degrees.
Filtermesh results/mannequin.opt.m -angle 40 -mark -genus | Subdivfit -mfile - -file data/mannequin.pts -crep 1e-5 -csharp 1e-5 -reconstruct >results/mannequin.sub0.m

echo 'Computing final subdivided surface.'
Subdivfit -mfile results/mannequin.sub0.m -nsub 2 -outn >results/mannequin.sub2limit.m

echo 'Use view_reconstruction_mannequin.sh to view the results.'
