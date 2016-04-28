@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo Running phase 1 - Recon. 
Recon <data/distcap.pts -samplingd 0.02 | Filtermesh -genus -rmcomp 100 -fillholes 30 -triangulate -genus >data/distcap.recon.m

echo Running phase 2 - Meshfit. 
Meshfit -mfile data/distcap.recon.m -file data/distcap.pts -crep 1e-5 -reconstruct >data/distcap.opt.m

if .==. (
  echo Skipping phase 3 because it takes several minutes to compute.
  del data\distcap.sub0.m
  del data\distcap.sub2limit.m
) else (
  echo Running phase 3 - Subdivfit. 
  :: Initially tagging edges sharp if dihedral angle exceeds 52 degrees.
  Filtermesh data/distcap.opt.m -angle 52 -mark | Subdivfit -mfile - -file data/distcap.pts -crep 1e-5 -csharp .2e-5 -reconstruct >data/distcap.sub0.m

  echo Computing final subdivided surface.
  Subdivfit -mfile data/distcap.sub0.m -nsub 2 -outn >data/distcap.sub2limit.m
)

echo Use view_recon_distcap.bat to view the results.
