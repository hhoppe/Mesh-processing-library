@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo '.'
echo 'Drag down with the right mouse button to have a closer look.'
echo '.'

G3dOGL data/bunny.vertexcache.m -st data/bunny.s3d -key DmDTDC %G3DARGS%
