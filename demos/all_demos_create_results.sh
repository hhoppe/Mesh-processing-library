#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"

PATH=.:$PATH # allow running bash scripts in local directory

set -v

create_reconstruction_cactus.sh
create_reconstruction_distcap.sh
create_reconstruction_polygon.sh

create_simplified_using_meshopt.sh

create_progressive_mesh_gaudipark.sh
create_progressive_mesh_cessna.sh
create_progressive_mesh_club.sh
determine_approximation_error.sh

create_geomorphs.sh
create_progressive_hull_bunny.sh
create_progressive_hull_standingblob.sh

create_selectively_refined_office.sh
create_selectively_refined_terrain.sh

create_terrain_hierarchy.sh

create_topologically_simplified.sh

create_vertex_cached_bunny.sh

create_spherical_param_bunny.sh

create_rendered_mechpart_image.sh
create_rendered_mechpart_video.sh
create_voronoi_fillin.sh


