#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

r=curve1

echo
echo 'Running phase 1 - Recon.'
echo
Recon data/$r.pts -samplingd 0.06 -grid 30 >results/$r.recon.a3d

echo
echo 'Joining unorganized line segments into a single polyline.'
echo
Filtera3d <results/$r.recon.a3d -joinlines >results/$r.joined.a3d

echo
echo 'Optimizing the polyline to fit to points.'
echo
Polyfit -pfile results/$r.joined.a3d -file data/$r.pts -crep 3e-4 -spring 1 -reconstruct >results/$r.opt.a3d

echo
echo 'Use view_reconstruction_polygon.sh to see the results.'
echo
