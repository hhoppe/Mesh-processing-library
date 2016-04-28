@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

:: This is how the file was converted from Blender via *.ply:
::   perl bin/plytoMesh.pl data/blob5.ply | Filtermesh -genus -gmerge -genus -triangulate -angle 20 -mark -genus >data/blob5.orig.m

echo Running Meshfit.
Filtermesh data/blob5.orig.m -randpts 10000 -vertexpts | Meshfit -mfile data/blob5.orig.m -fi - -crep 1e-5 -simplify >data/blob5.meshopt.simplified.m

echo Use view_simplified_using_meshopt.bat to see the results.
