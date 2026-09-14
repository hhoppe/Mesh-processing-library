#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"

# Remove the files listed in generated_files.txt, skipping its comment lines.
while read -r name _; do
  if [[ -n $name && $name != \#* ]]; then rm -f "data/$name"; fi
done <generated_files.txt
