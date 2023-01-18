@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

:: MeshReorder ~/data/mesh/bunny.orig.m -fifo -analyze -color_corners 1 >v0.m

echo From original mesh data/bunny.orig.m, creating data/bunny.vertexcache.m with optimized face ordering.
MeshReorder data/bunny.orig.m -fifo -cache_size 16 -analyze -meshify5 -color_corners 1 -analyze >data/bunny.vertexcache.m

echo .
echo Use view_vertexcache_bunny.bat to see the optimized mesh traversal order.
echo .
