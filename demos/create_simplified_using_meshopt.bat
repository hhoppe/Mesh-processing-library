@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

:: This is how the file was converted from Blender via *.ply:
::   perl ../bin/ply_to_mesh ~/data/blender/blob5.ply | Filtermesh -genus -removekey normal -gmerge -genus -triangulate -angle 20 -mark -genus >data/blob5.orig.m

echo Running Meshfit.
Filtermesh data/blob5.orig.m -randpts 10000 -vertexpts | Meshfit -mfile data/blob5.orig.m -fi - -simplify >results/blob5.meshopt.simplified.m

echo Use view_simplified_using_meshopt.bat to see the results.
