#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

echo
echo 'Compute a mapping from the flat octahedron to the sphere that is optimized for inverse stretch.'
echo
SphereSample -domain octaflat -scheme domain -egrid 128 -keys domaincorner,imageuv -mesh_sphere |
  Filtermesh -gmerge -removekey Ovi -cornermerge -renamekey v domaincorner global -renamekey vc imageuv uv \
             -assign_normals -removekey sharp |
  mesh_to_pm -minqem -qemvolume 0 -dihallow -affectpq 3 -vsgeom -keepglobalv 1 -norfac 0 -trishapeafac 1e1 \
             -strict_sharp 2 >results/octahedron_octaflat_sharp_tmp.pm

SphereParam results/octahedron_octaflat_sharp_tmp.pm -base coord -flatten_to_x0 -fix_base -optimize_inverse \
            -respect_sharp_edges -keep_uv 1 >results/octahedron_octaflat_sphparam_tmp.m

FilterPM results/octahedron_octaflat_sharp_tmp.pm -finest -truncate_prior |
  SphereParam -mesh_for_base results/octahedron_octaflat_sphparam_tmp.m -fix_base -optimize_inverse -keep_uv 1 |
  Filtermesh -genus -removekey normal -removekey wid -cornermerge >results/octaflat_eg128.uv.sphparam.m

rm -f results/octahedron_octaflat_sharp_tmp.pm results/octahedron_octaflat_sphparam_tmp.m


echo
echo 'From the original mesh data/bunny.orig.m, compute a progressive mesh for SphereParam optimization.'
echo
mesh_to_pm data/bunny.orig.m -minqem -vsgeom -dihallow >results/bunny.for_sphereparam.pm

echo
echo 'Spherically parameterize and store octaflat uv coordinates in results/bunny.sphparam.octaflat.m.'
echo
SphereParam results/bunny.for_sphereparam.pm -rot data/bunny.s3d \
            -uv_map results/octaflat_eg128.uv.sphparam.m >results/bunny.sphparam.octaflat.m

echo
echo 'Spherically parameterize and store longitude-latitude uv coordinates in results/bunny.sphparam.lonlat.m.'
echo
SphereParam results/bunny.for_sphereparam.pm -rot data/bunny.s3d -split_meridian >results/bunny.sphparam.lonlat.m


echo
echo 'Create an octaflat normal map.'
echo
SphereSample -domain octaflat -grid 1024 -domain_file results/octaflat_eg128.uv.sphparam.m \
             -param results/bunny.sphparam.octaflat.m -signal normal \
             -write_texture results/bunny.spheresample.octaflat.unrotated.normalmap.png

echo
echo 'Create a longitude-latitude normal map.'
echo
SphereSample -grid 1024 -param results/bunny.sphparam.lonlat.m -signal normal \
             -write_lonlat_texture results/bunny.spheresample.lonlat.unrotated.normalmap.png


echo
echo 'Create an octaflat progressive mesh by minimizing an "Appearance-preserving simplification" (APS) metric.'
echo
mesh_to_pm results/bunny.sphparam.octaflat.m -minaps -nominii1 -strict 2 >results/bunny.octaflat.pm

echo
echo 'Similarly create a longitude-latitude progressive mesh.'
echo
mesh_to_pm results/bunny.sphparam.lonlat.m -minaps -nominii1 -strict 2 >results/bunny.lonlat.pm


echo
echo 'Use view_spherical_param_bunny.sh to see the results.'
echo
