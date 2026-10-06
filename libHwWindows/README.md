# libHwWindows

The Win32 implementation of the windowing class `Hw`, which the viewers `G3dOGL`, `G3dVec`, and `VideoViewer`
use to create a window, receive mouse and keyboard events, and render using OpenGL.
Its base class `HwBase` (declared in `HwBase.h` here) is shared with the X11 implementation in
[`libHwX`](../libHwX).
The Visual Studio build and the `win`, `clang`, and `mingw` configurations of `make` link this library.
