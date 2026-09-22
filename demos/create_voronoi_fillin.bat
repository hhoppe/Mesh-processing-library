@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

Filterimage data/texture.input.png -color 255 0 0 255 -voronoidilate >results/texture.output.png

echo Use view_voronoi_fillin.bat to see the result in results/texture.output.png
