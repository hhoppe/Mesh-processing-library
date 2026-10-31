:: @echo off
setlocal

cd "%~p0"

if .==. (
  call view_reconstruction_mannequin.bat
  call view_reconstruction_distcap.bat
  call view_reconstruction_polygon.bat

  call view_simplified_using_meshopt.bat

  call view_progressive_mesh_gaudipark.bat
  call view_progressive_mesh_cessna.bat
  call view_progressive_mesh_club.bat
  call view_geomorphs.bat
  call view_progressive_hull_bunny.bat
  call view_progressive_hull_standingblob.bat

  call view_selectively_refined_office.bat
  call view_selectively_refined_terrain.bat

  call view_terrain_hierarchy.bat
  call view_gcanyon_frames.bat

  call view_psc_drumset.bat

  call view_topologically_simplified.bat

  call view_vertex_cached_bunny.bat

  call view_spherical_param_bunny.bat

  call view_rendered_mechpart_image.bat
  call view_rendered_mechpart_video.bat
  call view_voronoi_fillin.bat

  call view_hidden_line_removed.bat


)
