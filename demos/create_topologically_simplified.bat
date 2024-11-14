@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo Extracting a mesh of 200000 triangles from the progressive mesh of the office model, then
echo  removing its topological noise by simplifying it from genus 50 to genus 4.

FilterPM data/office.pm -nf 200000 -outmesh | MinCycles - -frac_cycle_length 1.2 -max_cycle_length .10 >data/office.nf200000.mincycles.m

echo Use view_topologically_simplified.bat to see the results.
