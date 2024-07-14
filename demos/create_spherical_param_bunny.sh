#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh


echo 'From the original mesh data/bunny.orig.m, compute a progressive mesh, then the spherical parameterization data/bunny.sphparam.m.'
bin/meshtopm.sh data/bunny.orig.m -minqem -vsgeom -dihallow | SphereParam - -rot data/bunny.s3d -split_meridian >data/bunny.sphparam.m


echo 'Resample this spherical parameterization into a remesh and an associated normal map.'

SphereSample -egrid 128 -domain octaflat -scheme best -param data/bunny.sphparam.m -rot data/bunny.s3d -keys imageuv -remesh | Filtermesh -renamekey v imageuv uv >data/bunny.spheresample.remesh.m

SphereSample -egrid 1024 -omit_faces -domain octaflat -scheme best -param data/bunny.sphparam.m -rot data/identity.s3d -signal N -write_texture data/bunny.spheresample.octaflat.unrotated.normalmap.png


echo 'Create a longitude-latitude normal map to use on the original mesh.'

SphereSample -grid 1024 -param data/bunny.sphparam.m -rot data/identity.s3d -signal N -write_lonlat_texture data/bunny.lonlat.unrotated.normalmap.png

echo 'Create a progressive mesh by minimizing an "Appearance-preserving simplification" (APS) metric.'

bin/meshtopm.sh data/bunny.sphparam.m -minaps -nominii1 -strict 2 >data/bunny.split_meridian.pm


echo '.'
echo 'Use view_spherical_param_bunny.sh to see the results.'
echo '.'
