#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo 'From original mesh data/cessna.orig.m, creating progressive mesh data/cessna.pm'
echo '(This will take a few minutes.)'

bin/meshtopm.sh data/cessna.orig.m >data/cessna.pm

echo '.'
echo 'Use view_pm_cessna.sh to view the resulting progressive mesh.'
echo '.'
