@echo off
setlocal

:: Examples:
::  meshtopm.bat <file.m >file.pm
::  meshtopm.bat file.m >file.pm
::  cat file.m | meshtopm.bat -numpts 100000 >file.numpts100000.pm

:: (The earlier timestamp used `wmic`, which is removed from current Windows.)
set tmproot=%temp:\=/%/%~nx0_%random%

MeshSimplify %* -prog %tmproot%.prog -simplify >%tmproot%.base.m || goto :cleanup

reverselines %tmproot%.prog >%tmproot%.rprog || goto :cleanup

:: Delete the .prog file early because it can be large.
del %tmproot:/=\%.prog 2>nul

Filterprog -fbase %tmproot%.base.m -fprog %tmproot%.rprog -pm

:cleanup
set status=%errorlevel%

:: Always remove the temporary files, as does the trap in meshtopm.sh.
del %tmproot:/=\%.prog %tmproot:/=\%.rprog %tmproot:/=\%.base.m 2>nul

exit /b %status%
