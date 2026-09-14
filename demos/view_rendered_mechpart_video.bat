@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

VideoViewer results/mechpart.video.mp4 -key i
