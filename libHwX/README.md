# libHwX

The X Window System (X11) implementation of the windowing class `Hw`, which the viewers `G3dOGL`, `G3dVec`,
and `VideoViewer` use to create a window, receive mouse and keyboard events, and render using OpenGL.
Its base class `HwBase` is declared in [`libHwWindows/HwBase.h`](../libHwWindows/HwBase.h).
The `unix` and `cygwin` configurations of `make` link this library.  (On macOS, X11 is provided by XQuartz.)
