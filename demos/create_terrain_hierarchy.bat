@echo off
setlocal

cd "%~p0"
call bin/_initdemos.bat

echo Note: this next step requires that bash be installed on the system.

bash -c ./create_terrain_hierarchy.sh

echo .
echo Use view_terrain_hierarchy.bat to view the resulting selectively refinable mesh.
echo .
