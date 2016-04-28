@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

Filterimage data/texture.input.png -color 255 0 0 255 -voronoidilate >data/texture.output.png

echo Use view_voronoi_fillin.bat to see the result in data/texture.output.png
