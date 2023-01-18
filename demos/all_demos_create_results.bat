:: @echo off
setlocal

cd "%~p0"

if .==. (
  call create_recon_cactus.bat
  call create_recon_distcap.bat
  call create_recon_polygon.bat

  call create_simplified_using_meshopt.bat

  call create_pm_gaudipark.bat
  call create_pm_cessna.bat
  call create_pm_club.bat
  call determine_approximation_error.bat

  call create_geomorphs.bat

  call create_sr_office.bat
  call create_sr_terrain.bat
  
  call create_terrain_hierarchy.bat

  call create_topologically_simplified.bat

  call create_vertexcache_bunny.bat

  call create_rendered_mechpart_image.bat
  call create_rendered_mechpart_video.bat
  call create_voronoi_fillin.bat


)
