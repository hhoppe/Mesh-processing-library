@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo To take interactive control, select the first window, press "o" to remove object-centric motion,
echo   then drag using any of the mouse buttons.
echo .

call g3d_cmp -win1 "-key o--J" -winl "-key DmDe" data/mannequin.pts results/mannequin.recon.m results/mannequin.opt.m results/mannequin.sub0.m results/mannequin.sub2limit.m -st data/mannequin.s3d -backcolor hhblue -key DmDeDb %G3D_ARGS%
