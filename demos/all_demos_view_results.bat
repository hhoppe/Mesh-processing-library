:: @echo off
setlocal

cd "%~p0"

if .==. (
  call view_recon_cactus.bat
  call view_recon_distcap.bat
  call view_recon_polygon.bat

  call view_simplified_using_meshopt.bat

  call view_pm_gaudipark.bat
  call view_pm_cessna.bat
  call view_pm_club.bat
  call view_geomorphs.bat

  call view_sr_office.bat
  call view_sr_terrain.bat

  call view_terrain_hierarchy.bat
  call view_gcanyon_frames.bat

  call view_psc_drumset.bat

  call view_topologically_simplified.bat

  call view_vertexcache_bunny.bat

  call view_rendered_mechpart_image.bat
  call view_rendered_mechpart_video.bat
  call view_voronoi_fillin.bat

  call view_hidden_line_removed.bat


)
