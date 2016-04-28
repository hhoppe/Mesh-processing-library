#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

shopt -u failglob

# set -v
# set -x  # for deeper debugging
r=data/gcanyon_sq129_b44

log=data/create_terrain_hierarchy.log
echo >$log
export NO_CONSOLE_PROGRESS=1

echo Constructing 4x4 progressive meshes at level 0
for x in {0..3}; do
  for y in {0..3}; do
    rl=$r.l0.x$x.y$y
    echo Constructing $r.l-1.x$x.y$y.pm and $rl.pm
    rm -f ${rl}* >/dev/null
    (Filterimage $r.elev.png -tobw -elevation -step 1 -scalez 0.000694722 -removekinks \
      -blocks 32 -bx $x -by $y -tomesh | \
      Filtermesh -assign_normals -nocleanup >$rl.orig.m) 2>>$log
    # SRcreate $rl -terrain -no_simp_bnd
    bin/meshtopm.sh $rl.orig.m -vsgeom -terrain -no_simp_bnd >$rl.pm 2>>$log
    rm -f $rl.orig.m
    FilterPM $rl.pm -maxresid .30e-3 -truncate_prior >$r.l-1.x$x.y$y.pm 2>>$log
    FilterPM $rl.pm -maxresid .40e-3 -truncate_prior -maxresidd .30e-3 -truncate_beyond >v.pm 2>>$log && \
      mv v.pm $rl.pm
  done
done
echo Stitching progressive meshes to form >$r.l-1.stitched.pm
StitchPM -rootname $r.l-1 -blockx 4 -blocky 4 -blocks 32 -stitch >$r.l-1.stitched.pm 2>>$log
echo Stitching progressive meshes to form >$r.l0.stitched.pm
StitchPM -rootname $r.l0 -blockx 4 -blocky 4 -blocks 32 -stitch >$r.l0.stitched.pm 2>>$log
rm -f $r.l-1.x?.y?.pm

echo Constructing 2x2 progressive meshes at level 1
for x in {0..1}; do
  for y in {0..1}; do
    rl=$r.l1.x$x.y$y
    (( xo0 = $x * 2 + 0 ))
    (( xo1 = $x * 2 + 1 ))
    (( yo0 = $y * 2 + 0 ))
    (( yo1 = $y * 2 + 1 ))
    cp -p $r.l0.x$xo0.y$yo0.pm $rl.x0.y0.pm
    cp -p $r.l0.x$xo0.y$yo1.pm $rl.x0.y1.pm
    cp -p $r.l0.x$xo1.y$yo0.pm $rl.x1.y0.pm
    cp -p $r.l0.x$xo1.y$yo1.pm $rl.x1.y1.pm
    echo Stitching 2x2 progressive meshes to form $rl.stitched.pm
    StitchPM -rootname $rl -blockx 2 -blocky 2 -blocks 32 -stitch >$rl.stitched.pm 2>>$log
    (( cropxl = $x * 64 ))
    (( cropxh = (1 - $x) * 64 ))
    (( cropyl = $y * 64 ))
    (( cropyh = (1 - $y) * 64 ))
    Filterimage $r.elev.png -tobw -cropsides $cropxl $cropxh $cropyl $cropyh \
      -step 1 -scalez 0.000694722 -removekinks -tofloats $rl.floats 2>>$log
    echo Simplifying stitched progressive mesh $rl.stitched.pm
    bin/PMsimplify $rl.stitched.pm -vsgeom -terrain -wedge_materials 0 -strict_sharp 1 -no_simp_bnd -ter_grid $rl.floats 2>>$log
    nf=`FilterPM $rl.stitched.pm -stat |& grep 'Basemesh' | sed 's/^.*nf=//'`
    echo nf=$nf >>$log
    mv $rl{.stitched.new,.full}.pm
    rm -f $rl.floats $rl.x{0,1}.y{0,1}.pm $rl.stitched.pm
    echo Truncating stitched progressive mesh based on maximum residual error
    FilterPM $rl.full.pm -maxresidd 1.0e-3 -truncate_prior >$rl.pm 2>>$log
    FilterPM $rl.pm -nfaces $nf -truncate_beyond >$r.l1t.x$x.y$y.pm 2>>$log
  done
done
rm -f $r.l0.x?.y?.pm

echo Constructing 1x1 progressive meshes at level 2
echo Stitching 2x2 progressive meshes to form $r.l1t.stitched.pm
StitchPM -rootname $r.l1t -blockx 2 -blocky 2 -blocks 64 -stitch >$r.l1t.stitched.pm 2>>$log
echo Stitching 2x2 progressive meshes to form $r.l1.stitched.pm
StitchPM -rootname $r.l1 -blockx 2 -blocky 2 -blocks 64 -stitch >$r.l1.stitched.pm 2>>$log
rm -f $r.l1.x?.y?{.pm,.full.pm}
rm -f $r.l1t.x?.y?.pm

echo Simplifying stitched progressive mesh to form $r.l2.pm
Filterimage $r.elev.png -tobw -step 1 -scalez 0.000694722 -removekinks -tofloats $r.floats 2>>$log
bin/PMsimplify $r.l1.stitched.pm -vsgeom -terrain -wedge_materials 0 -strict_sharp 1 -ter_grid $r.floats 2>>$log
mv $r.l1.stitched.new.pm $r.l2.pm
rm -f $r.floats

echo The result can be viewed using view_terrain_hierarchy.sh or view_terrain_hierarchy.bat

if false; then
  FilterPM $r.l2.pm -tosrm >$r.l2.srm
fi
