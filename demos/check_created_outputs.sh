#!/bin/bash

# Check that the files written into results/ by the create scripts look reasonable, using the reference values
# in results_reference.txt.  For an image or video, each per-channel mean and standard deviation must be within 2%
# (so a blank rendering fails); for the MeshDistance output *.approximation_error.txt, the geometric and normal
# rms errors (dL2 nL2) must be within 20% and the maximum errors (dLi nLi) within 50%, because a maximum depends on
# a single worst point and so varies more across compilers; for any other file, the size must be within 20%, except
# 35% for cactus.sub2limit.m (see below).
# Usage: check_created_outputs.sh [--update]
#  --update: rewrite the reference values from the current files instead of checking them.

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

update=0
if [[ ${1-} == --update ]]; then update=1; fi

# Print the values measured for the file results/$1: per-channel "mean sd" for an image or video, the errors
# "dL2 nL2 dLi nLi" from the both-directions ("B") rows of MeshDistance output, else the size.
measure() {
  case $1 in
    # In each "#  B(" row, print the value of each dL2=, nL2=, dLi=, and nLi= field (without "%" or a trailing CR).
    *.approximation_error.txt) awk '/^#  B\(/ {
        for (i = 1; i <= NF; i++) if ($i ~ /^[dn]L[2i]=/) {
          v = $i; sub(/^[dn]L[2i]=%?/, "", v); sub(/\r$/, "", v); printf "%s%s", sep, v; sep = " " } }' "results/$1" ;;
    # The -stat statistics go to stderr, so pipe only stderr (discarding stdout); for each "ComponentN: ... av=X sd=Y"
    #  line, print X and Y (the last two fields without their "av=" and "sd=" prefixes).
    *.png | *.bmp) Filterimage "results/$1" -stat 2>&1 >/dev/null | awk '/^Component/ {
        printf "%s%.2f %.2f", sep, substr($(NF - 1), 4), substr($NF, 4); sep = " " }' ;;
    *.mp4) Filtervideo "results/$1" -stat 2>&1 >/dev/null | awk '/^Component/ {
        printf "%s%.2f %.2f", sep, substr($(NF - 1), 4), substr($NF, 4); sep = " " }' ;;
    *) echo $(($(wc -c <"results/$1"))) ;;
  esac
}

status=0
new_list=()
while IFS= read -r line; do
  read -r name refs <<<"$line"
  if [[ $name == \#* || -z $refs ]]; then
    new_list+=("$line")
    continue
  fi
  if [[ ! -f results/$name ]]; then
    echo "*** results/$name is missing." >&2
    status=1
    new_list+=("$line")
    continue
  fi
  if ! values=$(measure "$name"); then
    echo "*** results/$name could not be read." >&2
    status=1
    new_list+=("$line")
    continue
  fi
  if ((update)); then
    new_list+=("$name $values")
    continue
  fi
  tolerance=0.20
  if [[ $name == *.png || $name == *.bmp || $name == *.mp4 ]]; then tolerance=0.02; fi
  if [[ $name == *.approximation_error.txt ]]; then tolerance='0.20 0.20 0.50 0.50'; fi
  # The size of this file is proportional to the face count of the mesh fitted by Subdivfit, whose stochastic
  # optimization reaches different face counts across configurations (e.g., 196 to 229 faces, up to +22% in size).
  # Under the MSVC STL it even varies from run to run, because std::hash of a pointer depends on its absolute
  # address, which ASLR randomizes, so the iteration order of Set<Vertex> and similar containers changes.
  if [[ $name == cactus.sub2limit.m ]]; then tolerance=0.35; fi
  # The tolerance is a list with one entry per value, whose last entry also applies to any remaining values.
  if ! awk -v v="$values" -v r="$refs" -v t="$tolerance" 'BEGIN {
      n = split(v, va); if (n != split(r, ra)) exit 1
      nt = split(t, ta)
      for (i = 1; i <= n; i++) {
        tol = ta[i <= nt ? i : nt]
        if (va[i] < ra[i] * (1 - tol) || va[i] > ra[i] * (1 + tol)) exit 1 } }'; then
    echo "*** results/$name: measured '$values'; reference '$refs'." >&2
    status=1
  fi
done <results_reference.txt

if ((update)); then
  printf '%s\n' "${new_list[@]}" >results_reference.txt
  echo 'Updated the reference values in results_reference.txt.'
elif ((status == 0)); then
  echo 'All created outputs match their reference values.'
fi
exit $status
