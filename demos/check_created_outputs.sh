#!/bin/bash

# Check that the files written by the create scripts look reasonable, using the reference values in
# generated_files.txt.  For an image or video, each per-channel mean and standard deviation must be within 2%
# (so a blank rendering fails); for any other file, the size must be within 20%.  Files listed without
# reference values are not checked.
# Usage: check_created_outputs.sh [--update]
#  --update: rewrite the reference values from the current files instead of checking them.

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

update=0
if [[ ${1-} == --update ]]; then update=1; fi

# Print the values measured for data file $1: per-channel "mean sd" for an image or video, else the size.
measure() {
  case $1 in
    *.png | *.bmp) Filterimage "data/$1" -stat 2>&1 >/dev/null | awk '/^Component/ {
        printf "%s%.2f %.2f", sep, substr($(NF - 1), 4), substr($NF, 4); sep = " " }' ;;
    *.mp4) Filtervideo "data/$1" -stat 2>&1 >/dev/null | awk '/^Component/ {
        printf "%s%.2f %.2f", sep, substr($(NF - 1), 4), substr($NF, 4); sep = " " }' ;;
    *) echo $(($(wc -c <"data/$1"))) ;;
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
  if [[ ! -f data/$name ]]; then
    echo "*** data/$name is missing." >&2
    status=1
    new_list+=("$line")
    continue
  fi
  if ! values=$(measure "$name"); then
    echo "*** data/$name could not be read." >&2
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
  if ! awk -v v="$values" -v r="$refs" -v t="$tolerance" 'BEGIN {
      n = split(v, va); if (n != split(r, ra)) exit 1
      for (i = 1; i <= n; i++) if (va[i] < ra[i] * (1 - t) || va[i] > ra[i] * (1 + t)) exit 1 }'; then
    echo "*** data/$name: measured '$values'; reference '$refs'." >&2
    status=1
  fi
done <generated_files.txt

if ((update)); then
  printf '%s\n' "${new_list[@]}" >generated_files.txt
  echo 'Updated the reference values in generated_files.txt.'
elif ((status == 0)); then
  echo 'All created outputs match their reference values.'
fi
exit $status
