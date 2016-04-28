#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"

PATH=.:$PATH # allow running bash scripts in local directory

set -v

view_recon_cactus.sh
view_recon_distcap.sh
view_recon_polygon.sh

view_simplified_using_meshopt.sh

view_pm_gaudipark.sh
view_pm_club.sh
view_geomorphs.sh

view_sr_office.sh
view_sr_terrain.sh

view_terrain_hierarchy.sh
view_gcanyon_frames.sh

view_psc_drumset.sh


view_topologically_simplified.sh

view_rendered_mechpart_image.sh
view_rendered_mechpart_video.sh
view_voronoi_fillin.sh

view_hidden_line_removed.sh


