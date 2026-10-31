:: @echo off
setlocal

cd "%~p0"

if .==. (
  call create_reconstruction_mannequin.bat
  call create_reconstruction_distcap.bat
  call create_reconstruction_polygon.bat

  call create_simplified_using_meshopt.bat

  call create_progressive_mesh_gaudipark.bat
  call create_progressive_mesh_cessna.bat
  call create_progressive_mesh_club.bat
  call determine_approximation_error.bat

  call create_geomorphs.bat
  call create_progressive_hull_bunny.bat
  call create_progressive_hull_standingblob.bat

  call create_selectively_refined_office.bat
  call create_selectively_refined_terrain.bat

  call create_terrain_hierarchy.bat

  call create_topologically_simplified.bat

  call create_vertex_cached_bunny.bat

  call create_spherical_param_bunny.bat

  call create_rendered_mechpart_image.bat
  call create_rendered_mechpart_video.bat
  call create_voronoi_fillin.bat


)
