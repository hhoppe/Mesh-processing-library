@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo Running phase 1 - Recon. 
Recon <data/curve1.pts -samplingd 0.06 -grid 30 >data/curve1.recon.a3d

echo Joining unorganized line segments into a single polyline.
Filtera3d <data/curve1.recon.a3d -joinlines >data/curve1.joined.a3d

echo Optimizing the polyline to fit to points.
Polyfit -pfile data/curve1.joined.a3d -file data/curve1.pts -crep 3e-4 -spring 1 -reconstruct >data/curve1.opt.a3d

echo Done.

echo .
echo Use view_recon_polygon.bat to see the results.
echo .
