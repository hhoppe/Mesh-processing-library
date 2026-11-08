@echo off
setlocal

cd "%~p0"
call ./_initdemos.bat

where perl >nul 2>nul || (echo This demo requires Perl, which is not on PATH.& exit /b 1)

set r=distcap

:: Insert end-of-frame elements into a *.record stream, after skipping the initial mesh (up to the first MVertex):
:: optionally a pause of $wait seconds, then $pause frames after each fit iteration ("# frame"), decreasing by $speedup
:: per iteration since the last edge frame, and one frame after every $every-th edge operation (Ecol, Espl, Eswa, or
:: Edge sharpness change).
set filter=perl -s -pe "$s ||= /^MVertex/; print qq(wait $wait\n) if $wait && $s && !$w++;
set filter=%filter% if (/^# frame/) { $n = int($pause - $j++ * $speedup); print qq(f 0 0 0\n) x ($n > 1 ? $n : 1); }
set filter=%filter% if ($s && /^E/ && $k++ %% $every == 0) { print qq(f 0 0 0\n); $j = 0; }" --
set filter_opt=%filter% -wait=2.0 -pause=20 -speedup=0.3 -every=14 results/%r%.opt.record
set filter_sub0=%filter% -wait=2.0 -pause=30 -speedup=0.0 -every=1 results/%r%.sub0.record

echo At the end of the animation, press 'P' and 'N' to move backwards and forwards through the objects.

set stream=results\%r%.view.tmp

:: Both echo and the native Perl end their lines with CRLF, which G3dOGL strips; suppress its warning about it.
set IGNORE_DOS_EOL=1

:: Precompute the input stream to avoid stutters in the viewer.  After each object: pause for some duration; get ready
:: to read the next object slot ("o 1 1 0"); and simulate keys, by default hiding the current object (u) and advancing
:: to the next object (<C-n>).  In cmd, ^< and ^> escape the angle brackets of the key names.
(
  type data\%r%.pts &                                 (echo wait 2.0) & (echo o 1 1 0) & (echo keys u^<C-n^>)
  Filtera3d results/%r%.u.a3d -tobinary &             (echo wait 2.5) & (echo o 1 1 0) & (echo keys u^<C-n^>)
  Filtera3d results/%r%.g.a3d -tobinary &             (echo wait 2.0) & (echo o 1 1 0) & (echo keys u^<C-n^>)
  Filtera3d results/%r%.p.a3d -tobinary &             (echo wait 1.5) & (echo o 1 1 0) & (echo keys u^<C-n^>)
  Filtera3d results/%r%.o.a3d -tobinary &             (echo wait 1.5) & (echo o 1 1 0)
  Filtera3d results/%r%.c.a3d -split 30 -tobinary &   (echo wait 1.5) & (echo o 1 1 0) & (echo keys ^<C-n^>u^<C-n^>)
  Filtera3d results/%r%.l.a3d -tobinary &             (echo wait 2.0) & (echo o 1 1 0)
  (echo keys ^<C-p^>^<C-p^>u^<C-n^>^<C-n^>u^<C-n^>)
  type results\%r%.m &                                (echo wait 4.0) & (echo o 1 1 0) & (echo keys u^<C-n^>)
  %filter_opt% &                                      (echo wait 1.5) & (echo o 1 1 0) & (echo keys u^<C-n^>)
  %filter_sub0% &                                     (echo wait 2.5) & (echo o 1 1 0) & (echo keys u^<C-n^>)
  type results\%r%.sub2limit.m &                      (echo keys DmDe)
) >%stream%

G3dOGL -st data/%r%.s3d -backcolor hhblue -key oJ---- -key DmDeDbDC -terse -sync %G3D_GEOM% %G3D_ARGS% <%stream%
del "%stream%"
