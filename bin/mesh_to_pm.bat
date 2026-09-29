@echo off
setlocal

:: Examples:
::  mesh_to_pm.bat <file.m >file.pm
::  mesh_to_pm.bat file.m >file.pm
::  cat file.m | mesh_to_pm.bat -numpts 100000 >file.numpts100000.pm

:: (The earlier timestamp used `wmic`, which is removed from current Windows.)
set tmproot=%temp:\=/%/%~nx0_%random%

:: The commands are chained rather than joined by "goto" because cmd.exe may fail to find a label in a file with
:: LF line endings.
MeshSimplify %* -prog %tmproot%.prog -simplify >%tmproot%.base.m && ^
Filterprog -fbase %tmproot%.base.m -fprog %tmproot%.prog -pm
set status=%errorlevel%

:: Always remove the temporary files, as does the trap in mesh_to_pm.
del %tmproot:/=\%.prog %tmproot:/=\%.base.m 2>nul

exit /b %status%
