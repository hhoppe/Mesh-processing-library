@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

VideoViewer results/mechpart.image.bmp %VIDEOVIEWER_ARGS%
