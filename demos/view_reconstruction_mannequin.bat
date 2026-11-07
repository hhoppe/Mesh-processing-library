@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

set r=mannequin

echo .
echo To take interactive control, drag in the first window using any of the mouse buttons.
echo .

call g3d_cmp -win1 "-key o--J" -winl "-key DmDe" data/%r%.pts results/%r%.recon.m results/%r%.opt.m results/%r%.sub0.m results/%r%.sub2limit.m -st data/%r%.s3d -backcolor hhblue -key DmDeDb %G3D_ARGS%
