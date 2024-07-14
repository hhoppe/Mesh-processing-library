#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"

PATH=.:$PATH # allow running bash scripts in local directory

set -v

create_recon_cactus.sh
create_recon_distcap.sh
create_recon_polygon.sh

create_simplified_using_meshopt.sh

create_pm_gaudipark.sh
create_pm_cessna.sh
create_pm_club.sh
determine_approximation_error.sh

create_geomorphs.sh

create_sr_office.sh
create_sr_terrain.sh

create_terrain_hierarchy.sh

create_topologically_simplified.sh

create_vertexcache_bunny.sh

create_spherical_param_bunny.sh

create_rendered_mechpart_image.sh
create_rendered_mechpart_video.sh
create_voronoi_fillin.sh


