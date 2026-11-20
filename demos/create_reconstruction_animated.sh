#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

r=distcap

pts=data/$r.pts
out=results/$r
samplingd=0.02
fillholes=40
angle=52

echo
echo 'Running phase 1 - Recon.'
echo
Recon "$pts" -rootname "$out" -samplingd $samplingd -what ugpoclm >"$out.m"

echo
echo 'Filling any small reconstruction holes.'
echo
Filtermesh "$out.m" -genus -rmcomp 100 -fillholes $fillholes -fairtriangulate -fixvertices -genus >"$out.recon.m"

echo
echo 'Running phase 2 - Meshfit.'
echo
Meshfit -mfile "$out.recon.m" -file "$pts" -record "$out.opt.record" -reconstruct >"$out.opt.m"

echo
echo 'Running phase 3 - Subdivfit.'
echo
Filtermesh "$out.opt.m" -angle $angle -mark -genus |
  Subdivfit -mfile - -file "$pts" -record "$out.sub0.record" -reconstruct >"$out.sub0.m"

echo
echo 'Computing the final subdivided surface.'
echo
Subdivfit -mfile "$out.sub0.m" -nsub 2 -outn >"$out.sub2limit.m"

echo
echo 'Use view_reconstruction_animated.sh to view the results.'
echo
