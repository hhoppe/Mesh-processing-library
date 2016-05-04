rem The *.natvis file is already included in libHh.vcxproj
rem However, the ImageWatch VS plugin does not look for it there; it must find it in the directory below.

rem copy "%~dp0libHh\libHh.natvis" "%USERPROFILE%\Documents\Visual Studio 2013\Visualizers\libHh.natvis"
rem copy "%~dp0libHh\libHh.natvis" "%USERPROFILE%\Documents\Visual Studio 2015\Visualizers\libHh.natvis"
copy "%~dp0libHh.natvis" "%USERPROFILE%\Documents\Visual Studio 2015\Visualizers\libHh.natvis"
