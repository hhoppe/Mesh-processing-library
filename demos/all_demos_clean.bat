@echo off
setlocal

cd "%~p0"

:: Remove the files listed in generated_files.txt, skipping its comment lines.
for /f "eol=# tokens=1" %%f in (generated_files.txt) do del "data\%%f" 2>nul
