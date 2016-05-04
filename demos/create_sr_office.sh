#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo '.'
echo '(This will take several minutes.)'
echo '.'

FilterPM data/office.pm -nfaces 80000 -outmesh >data/office.nf80000.orig.m
bin/meshtopm.sh data/office.nf80000.orig.m -vsgeom >data/office.nf80000.sr.pm

echo '.'
echo 'Use view_sr_office.sh to view the resulting selectively refinable mesh.'
echo '.'
