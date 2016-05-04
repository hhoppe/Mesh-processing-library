#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo 'From original mesh data/club.orig.m, creating progressive mesh data/club.pm'
echo '(This will take a few minutes.)'

bin/meshtopm.sh data/club.orig.m >data/club.pm

echo '.'
echo 'Use view_pm_club.sh to view the resulting progressive mesh.'
echo '.'
