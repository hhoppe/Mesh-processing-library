@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

G3dOGL --version

call g3d_cmp -win1 "-key --oJ" -winl "-key DmDeDb" data/mannequin.pts results/mannequin.recon.m results/mannequin.opt.m results/mannequin.sub0.m results/mannequin.sub2limit.m -st data/mannequin.s3d -backcolor hhblue -key DmDeDb %G3D_ARGS%
