#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'We extract a mesh of 200000 triangles from the progressive mesh of the office model, then'
echo ' remove its topological noise by simplifying it from genus 50 to genus 4.'
echo
FilterPM data/office.pm -nf 200000 -outmesh |
  MinCycles - -frac_cycle_length 1.2 -max_cycle_length .10 >results/office.nf200000.mincycles.m

echo
echo 'Use view_topologically_simplified.sh to see the results.'
echo
