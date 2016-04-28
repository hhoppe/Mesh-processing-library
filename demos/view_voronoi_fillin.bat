@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

VideoViewer data/texture.output.png
