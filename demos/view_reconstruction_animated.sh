#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

r=distcap

# The script bin/recon_test overrides these settings using environment variables.
pts=${RECON_PTS:-data/$r.pts}
out=${RECON_OUT:-results/$r}
s3d=${RECON_S3D:-data/$r.s3d}
if [[ -n $RECON_S3D ]]; then unset G3D_NO_S3DNAME; fi  # Let s3dname find a state file given by name.

# Insert end-of-frame elements into a *.record stream, after skipping the initial mesh (up to the first MVertex):
# optionally a pause of $wait seconds, then $pause frames after each fit iteration ("# frame"), decreasing by $speedup
# per iteration since the last edge frame, and one frame after every $every-th edge operation (Ecol, Espl, Eswa, or
# Edge sharpness change).
filter=(perl -s -pe '
  $s ||= /^MVertex/;
  print "wait $wait\n" if $wait && $s && !$w++;
  if (/^# frame/) { $n = int($pause - $j++ * $speedup); print "f 0 0 0\n" x ($n > 1 ? $n : 1); }
  if ($s && /^E/ && $k++ % $every == 0) { print "f 0 0 0\n"; $j = 0; }' --)

split=$out.split.tmp
opt=$out.opt.tmp
sub0=$out.sub0.tmp
trap 'rm -f "$split" "$opt" "$sub0"' EXIT

# Generate the slow parts in the background, so that the viewer starts at once and only finished files feed its pipe,
# which avoids stutters.  The background processes start after the .pts file, whose size exceeds the pipe capacity,
# so that they do not compete with the viewer startup.
(
  cat "$pts";                        echo wait 2.0; echo O 0 1 0
  Filtera3d "$out.c.a3d" -split 30 >"$split" & pid_split=$!
  cat "$out.u.a3d";                  echo wait 1.5; echo O 0 1 0
  "${filter[@]}" -wait=2.0 -pause=20 -speedup=0.3 -every=14 "$out.opt.record" >"$opt" & pid_opt=$!
  Filtera3d "$out.g.a3d" -tobinary;  echo wait 2.0; echo O 0 1 0
  "${filter[@]}" -wait=2.0 -pause=30 -speedup=0.0 -every=1 "$out.sub0.record" >"$sub0" & pid_sub0=$!
  Filtera3d "$out.p.a3d" -tobinary;  echo wait 1.5; echo O 0 1 0
  cat "$out.o.a3d";                  echo wait 1.5
  wait $pid_split; cat "$split";     echo wait 1.5; echo O 0 1 0
  Filtera3d "$out.l.a3d" -tobinary;  echo wait 1.5; echo O 0 1 0
  cat "$out.m";                      echo wait 2.0; echo O 0 1 0
  wait $pid_opt; cat "$opt";         echo wait 1.5; echo O 0 1 0
  wait $pid_sub0; cat "$sub0";       echo wait 1.5; echo O 0 1 0
  cat "$out.sub2limit.m";            echo keys DmDe
) | G3dOGL -st "$s3d" -backcolor hhblue -key oJ---- -key DmDeDbDC -terse -sync $G3D_GEOM $G3D_ARGS \
  || allow_sigpipe
