@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo .
echo Press 'P' and 'N' to advance to the previous and next models in the reconstruction sequence.
echo .
echo Press 'J' to stop automatic rotation.
echo .

G3dOGL --version

:: G3dOGL data/cactus.pts data/cactus.recon.m data/cactus.opt.m data/cactus.sub0.m data/cactus.sub2limit.m -st data/cactus.s3d -key NNDmDeNDmDeNDmDePPP--oJ -hwdelay 1.5 -hwkey NNNNN -backcolor hhblue %G3DARGS%

G3dOGL data/cactus.pts data/cactus.recon.m data/cactus.opt.m data/cactus.sub0.m data/cactus.sub2limit.m -st data/cactus.s3d -key DbNNDmDeNDmDeNDmDePPP--oJ -key NNNNN -backcolor hhblue %G3DARGS%
