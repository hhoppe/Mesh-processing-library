@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

set r=curve1

echo Running phase 1 - Recon.
Recon data/%r%.pts -samplingd 0.06 -grid 30 >results/%r%.recon.a3d

echo Joining unorganized line segments into a single polyline.
Filtera3d <results/%r%.recon.a3d -joinlines >results/%r%.joined.a3d

echo Optimizing the polyline to fit to points.
Polyfit -pfile results/%r%.joined.a3d -file data/%r%.pts -crep 3e-4 -spring 1 -reconstruct >results/%r%.opt.a3d

echo Done.

echo .
echo Use view_reconstruction_polygon.bat to see the results.
echo .
