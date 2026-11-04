#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

# The visualization only works for models (e.g., distcap) pretransformed into the unit cube, due to *.record xform.
r=distcap

# Insert end-of-frame elements into a *.record stream, after skipping the initial mesh (up to the first MVertex):
# optionally a pause of $wait seconds, then $pause frames after each fit iteration ("# frame"), decreasing by $speedup
# per iteration since the last edge frame, and one frame after every $every-th edge operation (Ecol, Espl, Eswa, or
# Edge sharpness change).
filter=(perl -s -pe '
  $s ||= /^MVertex/;
  print "wait $wait\n" if $wait && $s && !$w++;
  if (/^# frame/) { $n = int($pause - $j++ * $speedup); print "f 0 0 0\n" x ($n > 1 ? $n : 1); }
  if ($s && /^E/ && $k++ % $every == 0) { print "f 0 0 0\n"; $j = 0; }' --)

stream=results/$r.view.tmp
trap 'rm -f "$stream"' EXIT

# Precompute the input stream to avoid stutters in the viewer.
(
  cat data/$r.pts;                       echo wait 2.0; echo O 0 1 0
  cat results/$r.u.a3d;                  echo wait 1.5; echo O 0 1 0
  cat results/$r.g.a3d;                  echo wait 2.0; echo O 0 1 0
  cat results/$r.p.a3d;                  echo wait 1.5; echo O 0 1 0
  cat results/$r.o.a3d;                  echo wait 1.5
  Filtera3d results/$r.c.a3d -split 30;  echo wait 1.5; echo O 0 1 0
  cat results/$r.l.a3d;                  echo wait 1.5; echo O 0 1 0
  cat results/$r.m;                      echo wait 2.0; echo O 0 1 0
  "${filter[@]}" -wait=2.0 -pause=20 -speedup=0.3 -every=14 results/$r.opt.record; echo wait 1.5; echo O 0 1 0
  "${filter[@]}" -wait=2.0 -pause=30 -speedup=0.0 -every=1 results/$r.sub0.record; echo wait 1.5; echo O 0 1 0
  cat results/$r.sub2limit.m;            echo keys DmDe
) >"$stream"

G3dOGL -st data/$r.s3d -backcolor hhblue -key oJ---- -key DmDeDbDC -terse -sync $G3D_GEOM $G3D_ARGS <"$stream"
