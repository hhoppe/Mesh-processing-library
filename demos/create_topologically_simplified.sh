#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo 'Extracting a mesh of 200000 triangles from the progressive mesh of the office model, then'
echo ' removing its topological noise by simplifying it from genus 50 to genus 4.'

FilterPM data/office.pm -nf 200000 -outmesh | \
  MinCycles - -fraccyclelength 1.2 -maxcyclelen .10 -closecycles >data/office.nf200000.mincycles.m

echo 'Use view_topologically_simplified.sh to see the results.'
