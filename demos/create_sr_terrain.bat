@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

echo Creating original terrain mesh from elevation image.

Filterimage data/gcanyon_elev_crop.bw.png -tobw -elevation -step 6 -scalez 0.000194522 -removekinks -tomesh | Filtermesh -assign_normals >results/gcanyon_sq200.orig.m

echo .
echo Simplifying terrain mesh.  (This will take several minutes.)
echo .

call mesh_to_pm.bat results/gcanyon_sq200.orig.m -vsgeom -terrain >results/gcanyon_sq200.pm

echo Use view_sr_terrain.bat to see geomorphs on the view-dependent terrain.
