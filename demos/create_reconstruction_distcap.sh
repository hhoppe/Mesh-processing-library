#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

r=distcap

echo 'Running phase 1 - Recon.'
Recon data/$r.pts -rootname results/$r -samplingd 0.02 -what ugpoclm >results/$r.m

echo 'Filling any small reconstruction holes.'
Filtermesh results/$r.m -genus -fillholes 40 -triangulate -genus >results/$r.recon.m

echo 'Running phase 2 - Meshfit.'
Meshfit -mfile results/$r.recon.m -file data/$r.pts -record results/$r.opt.record -reconstruct >results/$r.opt.m

echo 'Running phase 3 - Subdivfit.'
Filtermesh results/$r.opt.m -angle 52 -mark -genus |
  Subdivfit -mfile - -file data/$r.pts -record results/$r.sub0.record -reconstruct >results/$r.sub0.m

echo 'Computing final subdivided surface.'
Subdivfit -mfile results/$r.sub0.m -nsub 2 -outn >results/$r.sub2limit.m

echo 'Use view_reconstruction_distcap.sh to view the results.'
