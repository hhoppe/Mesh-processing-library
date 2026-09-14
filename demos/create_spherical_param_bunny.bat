@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat


echo Compute a mapping from the flat octahedron to the sphere that is optimized for inverse stretch.
:: Because a pipe command is run in a subshell, the syntax "call script.bat" is unnecessary (and wrong) here.
SphereSample -domain octaflat -scheme domain -egrid 128 -keys domaincorner,imageuv -mesh_sphere | Filtermesh -gmerge -removekey Ovi -cornermerge -renamekey v domaincorner global -renamekey vc imageuv uv -assign_normals -removekey sharp | "bin/meshtopm.bat" -minqem -qemvolume 0 -dihallow -affectpq 3 -vsgeom -keepglobalv 1 -norfac 0 -trishapeafac 1e1 -strict_sharp 2 >results/v_sharp.pm

SphereParam results/v_sharp.pm -base coord -flatten_to_x0 -fix_base -optimize_inverse -respect_sharp_edges -keep_uv 1 >results/v.m

FilterPM results/v_sharp.pm -finest -truncate_prior | SphereParam -mesh_for_base results/v.m -fix_base -optimize_inverse -keep_uv 1 | Filtermesh -genus -removekey normal -removekey wid -cornermerge >results/octaflat_eg128.uv.sphparam.m

del results\v_sharp.pm results\v.m 2>nul


echo From the original mesh data/bunny.orig.m, compute a progressive mesh, then the spherical parameterization results/bunny.sphparam.m.
"bin/meshtopm.bat" data/bunny.orig.m -minqem -vsgeom -dihallow | SphereParam - -rot data/bunny.s3d -split_meridian >results/bunny.sphparam.m


echo Resample the bunny spherical parameterization into a remesh and an associated normal map.

SphereSample -domain octaflat -egrid 128 -sample_map results/octaflat_eg128.uv.sphparam.m -param results/bunny.sphparam.m -rot data/bunny.s3d -keys imageuv -remesh | Filtermesh -renamekey v imageuv uv >results/bunny.spheresample.remesh.m

SphereSample -domain octaflat -grid 1024 -domain_file results/octaflat_eg128.uv.sphparam.m -param results/bunny.sphparam.m -signal N -write_texture results/bunny.spheresample.octaflat.unrotated.normalmap.png


echo Create a longitude-latitude normal map to use on the original mesh.

SphereSample -grid 1024 -param results/bunny.sphparam.m -signal N -write_lonlat_texture results/bunny.lonlat.unrotated.normalmap.png

echo Create a progressive mesh by minimizing an "Appearance-preserving simplification" (APS) metric.

call bin/meshtopm.bat results/bunny.sphparam.m -minaps -nominii1 -strict 2 >results/bunny.split_meridian.pm


echo .
echo Use view_spherical_param_bunny.bat to see the results.
echo .
