#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

r=distcap

# The script bin/recon_test overrides these settings using environment variables.
pts=${RECON_PTS:-data/$r.pts}
out=${RECON_OUT:-results/$r}
samplingd=${RECON_SAMPLINGD:-0.02}
fillholes=${RECON_FILLHOLES:-40}
angle=${RECON_ANGLE:-52}

echo 'Running phase 1 - Recon.'
Recon "$pts" -rootname "$out" -samplingd $samplingd $RECON_RECON_PARAMS -what ugpoclm >"$out.m"

echo 'Filling any small reconstruction holes.'
Filtermesh "$out.m" -genus -rmcomp 100 -fillholes $fillholes -triangulate -genus >"$out.recon.m"

echo 'Running phase 2 - Meshfit.'
Meshfit -mfile "$out.recon.m" -file "$pts" -record "$out.opt.record" $RECON_MESHFIT_PARAMS -reconstruct >"$out.opt.m"

echo 'Running phase 3 - Subdivfit.'
Filtermesh "$out.opt.m" -angle $angle -mark -genus |
  Subdivfit -mfile - -file "$pts" -record "$out.sub0.record" $RECON_SUBDIVFIT_PARAMS -reconstruct >"$out.sub0.m"

echo 'Computing final subdivided surface.'
Subdivfit -mfile "$out.sub0.m" -nsub 2 -outn >"$out.sub2limit.m"

echo 'Use view_reconstruction_animated.sh to view the results.'
