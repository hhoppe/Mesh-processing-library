@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

VideoViewer results/texture.output.png %VIDEOVIEWER_ARGS%
