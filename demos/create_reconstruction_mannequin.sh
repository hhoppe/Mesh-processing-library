#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

r=mannequin

echo 'Running phase 1 - Recon.'
Recon data/$r.pts -samplingd 5.5 | Filtermesh -genus -fillholes 40 -triangulate -genus >results/$r.recon.m

echo 'Running phase 2 - Meshfit.'
Meshfit -mfile results/$r.recon.m -file data/$r.pts -reconstruct >results/$r.opt.m

echo 'Running phase 3 - Subdivfit.'
Filtermesh results/$r.opt.m -angle 40 -mark -genus | Subdivfit -mfile - -file data/$r.pts -csharp 8e-6 -reconstruct >results/$r.sub0.m

echo 'Computing final subdivided surface.'
Subdivfit -mfile results/$r.sub0.m -nsub 2 -outn >results/$r.sub2limit.m

echo 'Use view_reconstruction_mannequin.sh to view the results.'
