@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo .
echo To take interactive control, select the upper-left window, press "o" to remove object-centric motion,
echo   then drag using any of the mouse buttons.
echo .

:: G3dOGL data/distcap.pts results/distcap.recon.m results/distcap.opt.m results/distcap.sub0.m results/distcap.sub2limit.m -st data/distcap.s3d -key DbNNDmDeNDmDeNDmDePPP--oJ -hwdelay 1.5 -hwkey NNNNN -backcolor hhblue %G3DARGS%

set common=-st data/distcap.s3d -key Dbii -backcolor hhblue %G3DARGS%

if exist results/distcap.sub0.m (
G3dOGL %common% data/distcap.pts         -geom 500x500+150+4   -key "o--J&O" | ^
G3dOGL %common% results/distcap.recon.m     -geom 500x500+670+4   -async -input -killeof -key ODmDe | ^
G3dOGL %common% results/distcap.opt.m       -geom 500x500+1190+4  -async -input -killeof -key ODmDe | ^
G3dOGL %common% results/distcap.sub0.m      -geom 500x500+150+550 -async -input -killeof -key ODmDe | ^
G3dOGL %common% results/distcap.sub2limit.m -geom 500x500+670+550 -async -input -killeof
) else (
Recon <data/distcap.pts -samplingd 0.02 -what c | Filtera3d -split 30 | ^
G3dOGL %common% data/distcap.pts         -geom 500x500+150+4   -key "o--J&ODC" -input -terse | ^
G3dOGL %common% results/distcap.recon.m     -geom 500x500+670+4   -async -input -killeof -key ODmDe | ^
G3dOGL %common% results/distcap.opt.m       -geom 500x500+1190+4  -async -input -killeof -key DmDe
)
