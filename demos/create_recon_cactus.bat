@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo Running phase 1 - Recon. 
Recon <data/cactus.pts -samplingd 0.04 >results/cactus.recon.m

echo Running phase 2 - Meshfit. 
Meshfit -mfile results/cactus.recon.m -file data/cactus.pts -crep 1e-5 -reconstruct >results/cactus.opt.m

echo Running phase 3 - Subdivfit. 
:: Initially tagging edges sharp if dihedral angle exceeds 55 degrees.
Filtermesh results/cactus.opt.m -angle 55 -mark | Subdivfit -mfile - -file data/cactus.pts -crep 1e-5 -csharp .2e-5 -reconstruct >results/cactus.sub0.m

echo Computing final subdivided surface.
Subdivfit -mfile results/cactus.sub0.m -nsub 2 -outn >results/cactus.sub2limit.m

echo Use view_recon_cactus.bat to view the results.
