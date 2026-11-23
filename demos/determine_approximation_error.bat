@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo.
echo Measure the distance between original bunny and its progressive outer hull at 10000 faces.
echo.
FilterPM results/bunny.outer_hull.pm -nfaces 10000 -outmesh >results/bunny.outer_hull.nf10000.m

echo.
echo Columns are: geometric distance, color difference, vertex_normal difference
echo Rows are: r=random, v=vertices, 0/1=one_way, B=both_ways
echo  L2=root_mean_square,  Li=maximum
echo.
:: The saved output is compared against reference values by check_created_outputs.sh.
MeshDistance -mfile data/bunny.orig.m -mfile results/bunny.outer_hull.nf10000.m -bothdir 1 -maxerror 1 -verb 2 ^
             -distance >results/bunny.outer_hull.approximation_error.txt 2>&1
:: (The command "type" does not accept "/" as a path separator.)
type results\bunny.outer_hull.approximation_error.txt

del results\bunny.outer_hull.nf10000.m 2>nul
