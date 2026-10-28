#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"

PATH=.:$PATH # allow running bash scripts in local directory

set -v

view_reconstruction_cactus.sh
view_reconstruction_distcap.sh
view_reconstruction_polygon.sh

view_simplified_using_meshopt.sh

view_progressive_mesh_gaudipark.sh
view_progressive_mesh_cessna.sh
view_progressive_mesh_club.sh
view_geomorphs.sh
view_progressive_hull_bunny.sh
view_progressive_hull_standingblob.sh

view_selectively_refined_office.sh
view_selectively_refined_terrain.sh

view_terrain_hierarchy.sh
view_gcanyon_frames.sh

view_psc_drumset.sh

view_topologically_simplified.sh

view_vertex_cached_bunny.sh

view_spherical_param_bunny.sh

view_rendered_mechpart_image.sh
view_rendered_mechpart_video.sh
view_voronoi_fillin.sh

view_hidden_line_removed.sh


