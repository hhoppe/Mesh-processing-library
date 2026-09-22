@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

VideoViewer results/mechpart.video.mp4 -key i %VIDEOVIEWER_ARGS%
