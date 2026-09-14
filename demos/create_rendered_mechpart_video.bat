@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

G3dOGL data/mechpart.recon.m -st data/mechpart.s3d -key iiJ %G3DARGS% -geometry 800x600+150+0 -hidden -video 360 results/mechpart.video.mp4

echo File results/mechpart.video.mp4 is now created; it can be viewed using view_rendered_mechpart_video.bat
