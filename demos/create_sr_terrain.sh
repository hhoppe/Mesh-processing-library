#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo 'Creating original terrain mesh from elevation image.'

Filterimage data/gcanyon_elev_crop.bw.png -tobw -elevation -step 6 -scalez 0.000194522 -removekinks -tomesh | Filtermesh -assign_normals >data/gcanyon_sq200.orig.m

echo '.'
echo 'Simplifying terrain mesh.  (This will take several minutes.)'
echo '.'

bin/meshtopm.sh data/gcanyon_sq200.orig.m -vsgeom -terrain >data/gcanyon_sq200.pm

echo 'Use view_sr_terrain.sh to see geomorphs on the view-dependent terrain.'
