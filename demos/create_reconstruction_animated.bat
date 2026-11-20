@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

:: Reconstruct the surface in three phases (Recon, Meshfit, and Subdivfit), recording the stages for the animation.
set RECON_ROOT=distcap
set RECON_PTS=data/distcap.pts
set RECON_RESULTS=results
set RECON_SAMPLINGD=.02
set RECON_ANGLE=52
set RECON_RECORD=1
call recon/recon_process.bat || exit /b 1

echo.
echo Use view_reconstruction_animated.bat to view the results.
echo.
