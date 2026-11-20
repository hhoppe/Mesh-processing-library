@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

:: Reconstruct the surface in three phases (Recon, Meshfit, and Subdivfit).
set RECON_ROOT=mannequin
set RECON_PTS=data/mannequin.pts
set RECON_RESULTS=results
set RECON_SAMPLINGD=5.5
set "RECON_SUBDIVFIT_PARAMS=-csharp 8e-6"
call recon/recon_process.bat || exit /b 1

echo.
echo Use view_reconstruction_mannequin.bat to view the results.
echo.
