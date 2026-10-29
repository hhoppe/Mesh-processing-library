@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

call g3d_cmp -win1 "-key ,o----J" data/blob5.orig.m results/blob5.meshopt.simplified.m -st data/blob5.s3d -key DmDe %G3D_ARGS%
