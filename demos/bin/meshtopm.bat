@echo off
setlocal

:: Examples:
::  meshtopm.bat <file.m >file.pm
::  meshtopm.bat file.m >file.pm
::  cat file.m | meshtopm.bat -numpts 100000 >file.numpts100000.pm

:: set tmproot=%temp:\=/%/%~nx0_%date:-=%_%time::=%_%random%
:: set tmproot=%temp:\=/%/%~nx0_%random%
for /f "skip=1" %%x in ('wmic os get localdatetime') do if not defined tmpdate set tmpdate=%%x
set datetime=%tmpdate:~0,8%_%tmpdate:~8,6%
set tmproot=%temp:\=/%/%~nx0_%datetime%_%random%

MeshSimplify %* -prog %tmproot%.prog -simplify >%tmproot%.base.m || exit /b 1

reverselines %tmproot%.prog >%tmproot%.rprog || exit /b 1

del %tmproot:/=\%.prog 2>nul

Filterprog -fbase %tmproot%.base.m -fprog %tmproot%.rprog -pm || exit /b 1

del %tmproot:/=\%.rprog 2>nul
del %tmproot:/=\%.base.m 2>nul
