# Mesh Processing Library

[//]: # (This may be the most platform independent comment -- after a blank line)
[//]: # (Preview using http://daringfireball.net/projects/markdown/dingus )
[//]: # (See nice docs in http://daringfireball.net/projects/markdown/syntax )
[//]: # ( or https://confluence.atlassian.com/bitbucketserver/markdown-syntax-guide-776639995.html )
[//]: # (GitHub-specific: https://help.github.com/categories/writing-on-github/ )


## Overview

This <a href="https://github.com/hhoppe/Mesh-processing-library">GitHub package</a>
contains a C++ library and several application programs that demonstrate mesh processing technologies
published in research papers at ACM SIGGRAPH in 1992&ndash;1998:

- <em>surface reconstruction</em> (from unorganized, unoriented points)
- <em>mesh optimization</em>
- <em>subdivision surface fitting</em>
- <em>mesh simplification</em>
- <em>progressive meshes</em> (level-of-detail representation)
- <em>geomorphs</em> (smooth transitions across LOD meshes)
- <em>view-dependent mesh refinement</em>
- <em>smooth terrain LOD</em>
- <em>progressive simplicial complexes</em>
<!-- - <em>optimized mesh traversal</em> (for transparent vertex caching) -->

The source code has been updated to modern C++ style and supports cross-platform use.


## Requirements / dependencies

The code can be compiled with Microsoft Visual Studio using the solution (`*.sln`)
and project (`*.vcxproj`) files.

On Unix (Linux, Mac OS, and Cygwin),
the code compiles using the `clang` and `gcc` compilers and GNU `make`.

Reading/writing of images and videos is enabled using several options.
If available, image I/O can use `libpng`/`libjpeg` or Windows Imaging Component (WIC).
Video I/O can use Windows Media Foundation (WMF).
Across all platforms, if the command <a href="https://ffmpeg.org/">`ffmpeg`</a>
is present in the `PATH`,
it is spawned in a piped subprocess for both image and video I/O.

On Mac OS X, it is necessary to install
<a href="https://www.xquartz.org/">`XQuartz`</a> for `X11` support and
<a href="https://evermeet.cx/ffmpeg/">`ffmpeg`</a> for image/video I/O.


## Code compilation

### Build using Microsoft Visual Studio

Open the `mesh_processing.sln` file and build the solution
(typically as a `"ReleaseMD - x64"` build).
Executables are placed in `bin`, `bin/debug`, `bin/Win32`, or `bin/Win32/debug`,
depending on the build configuration.


### Build using GNU `make`

The `CONFIG` environment variable determines
which `make/Makefile_config_*` definition file is loaded.
On Windows, `CONFIG` can be chosen among `{win, w32, cygwin, mingw, mingw32, clang}`,
defaulting to `win` if undefined.
On Unix platforms (Linux and Mac OS), `CONFIG=unix` is the unique and default setting.

For example, to build using the Microsoft `cl` compiler (Debug, placing `*.exe` into directory `bin/win`):
<br/>`make -j8`<br/>

To build all programs (into either `bin/unix` or `bin/win`) and run all unit tests:
<br/>`make -j test`

To build on Unix, forcing the use of the `gcc` compiler (default is `clang`):
<br/>`make CC=gcc -j`

To build just the main library using the `mingw gcc` compiler on Windows:
<br/>`make CONFIG=mingw -j libHh`

To build the `Filtermesh` program (into `bin/clang`) using the `clang` compiler on Windows:
<br/>`make CONFIG=clang -j Filtermesh`

To build all programs (into `bin/cygwin`) and run all demos using the `gcc` compiler under Cygwin:
<br/>`make CONFIG=cygwin -j demos`

To clean up all files in all configurations:
<br/>`make CONFIG=all -j deepclean`

Note that additional options such as debug/release, 32-bit/64-bit, and
compiler tool paths/parameters are set in the various `make/Makefile_*` files.
These need to be adjusted depending on the versions and installation paths of the tools.
For instance, the line
`"rel ?= 0"` in `make/Makefile_config_win` specifies a debug (non-release) build, and
`"$(call prepend_PATH,...)"` in `make/Makefile_base_vc` sets the compiler directory.


## Publications and associated programs/demos

<table id="publications">

 <tr id="pub_recon">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/recon.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/recon/">Surface reconstruction from unorganized points</a>.</div>
   <div class="authors">Hugues Hoppe, Tony DeRose, Tom Duchamp, John McDonald, Werner Stuetzle.</div>
   <div class="pub"><cite>ACM SIGGRAPH 1992 Proceedings</cite>, 71-78.</div>
   <div class="desc"><em>Signed-distance field estimated from a set of unoriented noisy points.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_recon"><code>Recon</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>create_recon_*.{sh,bat}</code>, <code>view_recon_*.{sh,bat}</code></div>
  </td>
 </tr>

 <tr id="pub_meshopt">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/meshopt.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/meshopt/">Mesh optimization</a>.</div>
   <div class="authors">Hugues Hoppe, Tony DeRose, Tom Duchamp, John McDonald, Werner Stuetzle.</div>
   <div class="pub"><cite>ACM SIGGRAPH 1993 Proceedings</cite>, 19-26.</div>
   <div class="desc"><em>Traversing the space of triangle meshes to optimize model fidelity and conciseness.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_Meshfit"><code>Meshfit</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>create_recon_*</code>, <code>view_recon_*</code>, <code>create_simplified_using_meshopt</code>, <code>view_simplified_using_meshopt</code></div>
  </td>
 </tr>

 <tr id="pub_psrecon">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/psrecon.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/psrecon/">Piecewise Smooth Surface Reconstruction</a>.</div>
   <div class="authors">Hugues Hoppe, Tony DeRose, Tom Duchamp, Michael Halstead, Hubert Jin, John McDonald, Jean Schweitzer, Werner Stuetzle.</div>
   <div class="pub"><cite>ACM SIGGRAPH 1994 Proceedings</cite>, 295-302.</div>
   <div class="desc"><em>Subdivision surfaces with sharp features, and their automatic creation by data fitting.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_Subdivfit"><code>Subdivfit</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>create_recon_cactus</code>, <code>view_recon_cactus</code></div>
  </td>
 </tr>

 <tr id="pub_pm">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/pm.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/pm/">Progressive meshes</a>.</div>
   <div class="authors">Hugues Hoppe.</div>
   <div class="pub"><cite>ACM SIGGRAPH 1996 Proceedings</cite>, 99-108.</div>
   <div class="desc"><em>Efficient, lossless, continuous-resolution representation of surface triangulations.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_MeshSimplify"><code>MeshSimplify</code></a>, <a href="#prog_reverselines"><code>reverselines</code></a>, <a href="#prog_Filterprog"><code>Filterprog</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>create_geomorphs</code>, <code>view_geomorphs</code>
  </td>
 </tr>

 <tr id="pub_efficientpm">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/efficientpm.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/efficientpm/">Efficient implementation of progressive meshes</a>.</div>
   <div class="authors">Hugues Hoppe.</div>
   <div class="pub"><cite>Computers &amp; Graphics</cite>, 22(1), 1998, 27-36.</div>
   <div class="desc"><em>Progressive mesh data structures compatible with GPU vertex buffers.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_FilterPM"><code>FilterPM</code></a>, <a href="#prog_G3dOGL"><code>G3dOGL</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>create_pm_club</code>, <code>view_pm_club</code>, <code>determine_approximation_error</code></div>
  </td>
 </tr>

 <!--<tr id="pub_newqem">
     <td class="lcell">
      <img class="thumbnail" src="thumbnails/newqem.red.jpg" alt=""/>
     </td>
     <td class="rcell">
      <div class="title"><a href="https://hhoppe.com/proj/newqem/">New quadric metric for simplifying meshes with appearance attributes</a>.</div>
      <div class="authors">Hugues Hoppe.</div>
      <div class="pub"><cite>IEEE Visualization 1999 Conference</cite>, 59-66.</div>
      <div class="desc"><em>Efficient simplification metric designed around correspondence in 3D space.</em></div>
      <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_MeshSimplify"><code>MeshSimplify</code></a></div>
      <div class="demos"><span class="sdemos">Demos:</span> <code>create_pm_gaudipark</code>, <code>view_pm_gaudipark</code></div>
     </td>
 </tr>-->

 <tr id="pub_vdrpm">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/vdrpm.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/vdrpm/">View-dependent refinement of progressive meshes</a>.</div>
   <div class="authors">Hugues Hoppe.</div>
   <div class="pub"><cite>ACM SIGGRAPH 1997 Proceedings</cite>, 189-198.</div>
   <div class="desc"><em>Lossless multiresolution structure for incremental local refinement/coarsening.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_FilterPM"><code>FilterPM</code></a>, <a href="#prog_G3dOGL"><code>G3dOGL</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>create_sr_office</code>, <code>view_sr_office</code></div>
  </td>
 </tr>

 <tr id="pub_svdlod">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/svdlod.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/svdlod/">Smooth view-dependent level-of-detail control and its application to terrain rendering</a>.</div>
   <div class="authors">Hugues Hoppe.</div>
   <div class="pub"><cite>IEEE Visualization 1998 Conference</cite>, 35-42.</div>
   <div class="desc"><em>Visually smooth adaptation of mesh refinement using cascaded temporal geomorphs.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_StitchPM"><code>StitchPM</code></a>, <a href="#prog_G3dOGL"><code>G3dOGL</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>create_terrain_hierarchy</code>, <code>view_terrain_hierarchy</code>, <code>create_sr_terrain</code>, <code>view_sr_terrain</code>, <code>view_gcanyon_*</code></div>
  </td>
 </tr>

 <tr id="pub_psc">
  <td class="lcell">
   <img class="thumbnail" src="thumbnails/psc.red.jpg" alt=""/>
  </td>
  <td class="rcell">
   <div class="title"><a href="https://hhoppe.com/proj/psc/">Progressive simplicial complexes</a>.</div>
   <div class="authors">Jovan Popovic, Hugues Hoppe.</div>
   <div class="pub"><cite>ACM SIGGRAPH 1997 Proceedings</cite>, 217-224.</div>
   <div class="desc"><em>Progressive encoding of both topology and geometry.</em></div>
   <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_G3dOGL"><code>G3dOGL</code></a></div>
   <div class="demos"><span class="sdemos">Demos:</span> <code>view_psc_drumset</code></div>
  </td>
 </tr>

 <!--<tr id="pub_tvc">
     <td class="lcell">
      <img class="thumbnail" src="thumbnails/tvc.red.jpg" alt=""/>
     </td>
     <td class="rcell">
      <div class="title"><a href="https://hhoppe.com/proj/tvc/">Optimization of mesh locality for transparent vertex caching</a>.</div>
      <div class="authors">Hugues Hoppe.</div>
      <div class="pub"><cite>ACM SIGGRAPH 1999 Proceedings</cite>, 269-276.</div>
      <div class="desc"><em>Face reordering for efficient GPU vertex cache, advocating a FIFO policy.</em></div>
      <div class="bins"><span class="sprogram">Programs:</span> <a href="#prog_MeshReorder"><code>MeshReorder</code></a></div>
      <div class="demos"><span class="sdemos">Demos:</span> <code>create_vertexcache_result</code>, <code>view_vertexcache_result</code></div>
     </td>
 </tr>-->

</table>


## Demos

After the code is compiled, the demos can be run as follows.

In Windows, create, view, and clean up all the results using the `batch` scripts:
```shell
demos/all_demos_create_results.bat
demos/all_demos_view_results.bat
all_demos_clean.bat
```

On Unix-based systems (e.g. Linux, Mac OS, Cygwin), either run the `bash` scripts:
```shell
demos/all_demos_create_results.sh
demos/all_demos_view_results.sh
demos/all_demos_clean.sh
```

or alternatively (and faster), invoke `make` to create all results in parallel and then view them sequentially:

<pre>
make <em>[CONFIG=<var>config</var>]</em> -j demos
</pre>

Note that pressing the <kbd>Esc</kbd> key closes any open program window.


## Filter programs

All programs recognize the argument `--help` (or `-?`) to show their many options.

The programs `Filterimage`, `Filtermesh`, `Filtervideo`,
`FilterPM`, and `Filterframe` are all designed to:
- read media from `std::cin` (or from files or procedures specified as initial arguments),
- perform operations specified by arguments, and
- write media to `std::cout` (unless `-nooutput` is specified).

For example, the **`Filterimage`** command<a id="prog_Filterimage"></a>
```shell
Filterimage demos/data/gaudipark.png -rotate 20 -cropleft 100 -cropright 100 \
  -filter lanczos6 -scaletox 100 -color 0 0 255 255 -boundary border -cropall -20 \
  -setalpha 255 -color 0 0 0 0 -drawrectangle 30% 30% -30% -30% -gdfill \
  -info -to jpg >gaudipark.new.jpg
```
- reads the specified image,
- rotates it 20 degrees counterclockwise (with the default reflection boundary rule),
- crops its left and right sides by 100 pixels,
- scales it uniformly to a horizontal resolution of 100 pixels using a 6&times;6 Lanczos filter,
- adds a 20-pixel blue border on all sides,
- adds an alpha channel and creates an undefined (`alpha=0`) rectangular region in the image center,
- fills this region using gradient-domain smoothing,
- outputs some statistics on pixel colors (to `std::cerr`), and
- writes the result to a file under a different encoding.

As another example, the **`FilterPM`** command<a id="prog_FilterPM"></a>
```shell
FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | \
  Filtermesh -info -signeddistcontour 60 -genus | \
  G3dOGL -key DmDe
```
- reads a <em>progressive mesh</em> stream to construct a mesh with 1000 faces,
- reports statistics on the mesh geometry,
- remeshes the surface as the zero isocontour of its signed-distance function on a 60<sup>3</sup> grid,
- reports the new mesh genus, and
- shows the result in an interactive viewer using the specified view parameters,
- simulating keypresses <kbd>Dm</kbd> to enable flat shading and <kbd>De</kbd> to make mesh edges visible.

The **`Filtermesh`** command<a id="prog_Filtermesh"></a>
```shell
FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | \
  Filtermesh -angle 35 -silsubdiv -silsubdiv -mark | \
  G3dOGL -key DmDeDbJ---- -st demos/data/spheretext.s3d
```
- reads a 2000-face mesh, marks all edges with dihedral angle greater than 35 degrees as sharp,
- applies two steps of adaptive subdivision near these sharp edges, and
- shows the result flat-shaded (<kbd>Dm</kbd>), with edges (<kbd>De</kbd>),
  without backface culling (<kbd>Db</kbd>), spinning (<kbd>J</kbd>) somewhat slowly (<kbd>----</kbd>),
- starting from the view parameters stored in the `spheretext.s3d` file.

The **`Filtervideo`** command<a id="prog_Filtervideo"></a>
```shell
Filtervideo demos/data/palmtrees_small.mp4 -filter keys -scaleu 1.5 >palmtrees_small.scale1.5.mp4
```
- reads the video (entirely into memory),
- uniformly scales the two spatial dimensions by a factor 1.5 using the Keys bicubic filter, and
- saves the new video.

The command
```shell
Filtervideo demos/data/palmtrees_small.mp4 -info -trimbeg 4 -boundary clamped -trimend -20% | \
    -tscale 1.5 -framerate 150% -croprectangle 50% 50% 400 240 -gamma 1.5 -bitrate 10m | \
  VideoViewer demos/data/palmtrees_small.mp4 - -key =an
```
- reads the video (entirely into memory),
- reports statistics on the color channels,
- trims off 4 frames at the beginning,
- adds repeated copies of the last frames (with length 20% of the video),
- temporally scales the content by a factor of 1.5 and adjusts the framerate accordingly,
- spatially crops a centered rectangle with width 400 pixels and height 240 pixels,
- adjusts the color gamma,
- sets the output bitrate to 10 megabits/sec, and
- shows the result (`-` for `std::cin`) together with the original video in an interactive viewer,
- with keypress <kbd>=</kbd> to scale the window by 2, <kbd>a</kbd> to loop all (two) videos,
  and <kbd>n</kbd> to initially select the next video.


## Surface reconstruction

### <a id="prog_recon"></a>Recon

This program reads a list of 3D (x, y, z) points assumed to be sampled near some unknown manifold surface,
and reconstructs an approximating triangle mesh.
For example,
```shell
Recon <demos/data/distcap.pts -samplingd 0.02 | \
  Filtermesh -genus -rmcomp 100 -fillholes 30 -triangulate -genus | tee distcap.recon.m | \
  G3dOGL -st demos/data/distcap.s3d -key DmDe
```
- reads the text file of points,
- reconstructs a triangle mesh assuming a max sample spacing (&delta;+&rho; in paper) of 2% of the bounding volume,
- reports the genus of this initial mesh,
- removes all connected components with fewer than 100 triangle faces,
- fills and triangulates any hole bounded by 30 or fewer mesh edges,
- reports the genus of the modified mesh,
- saves it to a file, and
- displays it interactively starting from a specified viewpoint, with flat-shaded faces (<kbd>Dm</kbd>)
  and mesh edges (<kbd>De</kbd>).
  
To show the progression of the Marching Cubes algorithm,
```shell
Recon <demos/data/distcap.pts -samplingd 0.02 -what c | \
  Filtera3d -split 30 | G3dOGL -key DCDb -st demos/data/distcap_backside.s3d -terse
```
- selects the 'c' (cubes) output stream,
- forces a frame refresh every 30 polygon primitive, and
- shows the result without display-list caching (<kbd>DC</kbd>) and without backface culling (<kbd>Db</kbd>).

To show a similar streaming reconstruction of the surface mesh,
```shell
Recon <demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | \
  Filtera3d -split 30 | \
  G3dOGL demos/data/distcap.pts -key DCDb -st demos/data/distcap_backside.s3d -terse -input -key _Jo
```
- selects the default 'm' (mesh) output stream,
- converts the mesh to a stream of polygons, and
- shows the points and streamed reconstruction with a slow (<kbd>_</kbd>) rotation (<kbd>J</kbd>)
  about the object frame (<kbd>o</kbd>).
 
The same program can also read a list of 2D (y, z) points to reconstruct an approximating curve:
```shell
Recon <demos/data/curve1.pts -samplingd 0.06 -grid 30 | \
  Filtera3d -joinlines | tee curve1.a3d | \
  G3dOGL demos/data/curve1.pts -input -st demos/data/curve1.s3d
```

### <a id="prog_Meshfit"></a>Meshfit

Given an initial mesh and a list of 3D points, this program optimizes both the mesh connectivity and
geometry to improve the fit, i.e. minimizing the squared distances from the points to the surface.
For example,
```shell
Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | \
  tee distcap.opt.m | G3dOGL -st demos/data/distcap.s3d -key DmDe
```
- reads the previously reconstructed mesh and the original list of points,
- applies an optimized sequence of perturbations to improve both the mesh connectivity and geometry,
- using a specified tradeoff between mesh conciseness and fidelity
  (<var>c<sub>rep</sub></var>=1e-4 yields a coarser mesh),
- saves the result to a file, and displays it interactively.

The input points can also be sampled from an existing surface, e.g.:
```shell
Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | \
  Meshfit -mfile demos/data/blob5.orig.m  -file - -crep 1e-6 -simplify | \
  G3dOGL -st demos/data/blob5.s3d -key DmDe
```

To view the real-time fitting optimization,
```shell
Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts \
  -crep 1e-5 -outmesh - -record -reconstruct | \
  G3dOGL -st demos/data/distcap.s3d -key DmDeDC -async -terse
```
- writes both the initial mesh and the stream of mesh modifications, and
- displays the changing mesh asynchronously with display-list caching disabled (<kbd>DC</kbd>).

### <a id="prog_Polyfit"></a>Polyfit

This related program performs a similar optimization of a 1D polyline (either open or closed)
to fit a set of 2D points.  For example,
```shell
Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | \
  G3dOGL demos/data/curve1.pts -input -st demos/data/curve1.s3d
```
- reads the previously reconstructed polyline and the original list of points,
- optimizes vertex positions and simplifies the number of line segments according to some representation cost, and
- displays the result together with the original points.

### <a id="prog_Subdivfit"></a>Subdivfit

In a subdivision surface representation, a coarse base mesh tagged with <em>sharp</em> edges
defines a <em>piecewise smooth</em> surface as the limit of a subdivision process.
Such a representation both improves geometric fidelity and leads to a more concise description.
```shell
Filtermesh distcap.opt.m -angle 52 -mark | \
  Subdivfit -mfile - -file demos/data/distcap.pts -crep 1e-5 -csharp .2e-5 -reconstruct >distcap.sub0.m
```
- reads the previously optimized mesh and tags all edges with dihedral angle greater than 52 degrees as <em>sharp</em>,
- loads this tagged mesh and the original list of points,
- optimizes the mesh connectivity, geometry, and assignment of sharp edges to fit a
  <em>subdivision surface</em> to the points,
- with a representation cost of `1e-5` per vertex and `.2e-5` per sharp edge, and
- saves the resulting optimized base mesh to a file.  (The overall process takes a few minutes.)

To view the result,
```shell
G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" \
  -st demos/data/distcap.s3d -key NDmDe -hwdelay 5 -hwkey N
```
- reads the base mesh together with a second mesh obtained by applying two iterations of subdivision,
- shows the first mesh (<kbd>N</kbd>) with flat-shaded faces and edges (<kbd>DmDe</kbd>),
- waits for 5 seconds, and displays the second mesh (<kbd>N</kbd>) as a smooth surface without edges.

### <a id="prog_MeshDistance"></a>MeshDistance

This program computes measures of differences between two meshes.
It samples a dense set of points from a first mesh and computes the
projections of each point onto the closest point on a second mesh.
```shell
MeshDistance -mfile distcap.recon.m -mfile distcap.opt.m -bothdir 1 -maxerror 1 -distance
```
- `MeshDistance` loads the earlier results of mesh reconstruction and mesh optimization,
- computes correspondences from points sampled on each mesh to the other mesh (in both directions), and
- reports differences in geometric distance, color, and surface normals,
  using both L<sup>2</sup> (rms) and L<sup>&infin;</sup> (max) norms.


## Mesh simplification

Given a mesh, `MeshSimplify` applies a sequence of <em>edge collapse</em> operations
to simplify it to a coarse <em>base mesh</em> while trying to best preserve the appearance of the original model.
It supports many different simplification criteria, as well as face properties,
edges tagged as sharp, and vertex and corner attributes
(<var>n<sub>x</sub></var>,<var>n<sub>y</sub></var>,<var>n<sub>z</sub></var> normals,
<var>r</var>,<var>g</var>,<var>b</var> colors, and <var>u</var>,<var>v</var> texture coordinates).

For example,<a id="prog_MeshSimplify"></a>
```shell
MeshSimplify demos/data/club.orig.m -prog club.prog -simplify >club.base.m
```
- reads the original mesh and randomly samples points over its surface,
- progressively simplifies it by examining point residual distances, while recording changes to a `*.prog` file, and
- writes the resulting base mesh.

The next step is to reverse the sequence of stored edge collapses,
i.e. forming a progressive sequence of <em>vertex splits</em>: <a id="prog_reverselines"></a>
```shell
reverselines club.prog >club.rprog
```

We construct a concise <em>progressive mesh</em> by encoding the base mesh together
with the sequence of vertex splits that exactly recover the original mesh:<a id="prog_Filterprog"></a>
```shell
Filterprog -fbase club.base.m -fprog club.rprog -pm_encode >club.pm
```

The complete process from the original mesh to the progressive mesh is implemented by the script call
```shell
demos/bin/meshtopm.{sh,bat} demos/data/club.orig.m >club.pm
```

Given a progressive mesh, we can interactively traverse its continuous levels of detail:
```shell
G3dOGL -pm_mode club.pm -st demos/data/club.s3d -lightambient .4
```
- by dragging the left vertical slider using the left or right mouse button, and
- toggling mesh edges using the <kbd>De</kbd> key sequence.
 
We can also define geomorphs between discrete levels of detail, e.g.
```shell
FilterPM club.pm -nfaces 2000 -geom_nfaces 3300 -geom_nfaces 5000 -geom_nfaces 8000 | \
  G3dOGL -st demos/data/club.s3d -key SPDeN -lightambient .5 -thickboundary 1 -video 101 - | \
  VideoViewer - -key m
```
- creates a geomorph between 2000 and 3300 faces, another between 3300 and 5000 faces, and one more,
- shows these in a viewer with the level-of-detail slider enabled (<kbd>S</kbd>),
- selects all three geomorph meshes (<kbd>P</kbd>), enables mesh edges (<kbd>De</kbd>),
  selects the first mesh (<kbd>N</kbd>),
- records a video of 101 frames while moving the LOD slider, and
- shows the resulting video with mirror looping enabled (<kbd>m</kbd>).

This example displays a progressive mesh after truncating all detail below 300 faces and above 10000 faces:
```shell
FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | \
  G3dOGL -pm_mode - -st demos/data/standingblob.s3d
```

As an example of simplifying meshes with appearance attributes,
<!-- MeshSimplify - -nfaces 4000 -minqem -norfac 0. -colfac 1. -neptfac 1e5 -simplify | \ -->
```shell
Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | \
  MeshSimplify - -nfaces 4000 -simplify | \
  G3dOGL -st demos/data/imageup.s3d -key De -lightambient 1 -lightsource 0
```
- forms a planar grid mesh whose 200&times;200 vertices have colors sampled from a downsampled image,
- simplifies the mesh to 4000 faces while minimizing color differences,
  <!-- - ignoring surface normals and giving high weight to boundary accuracy, and-->
- shows the result with mesh edges (<kbd>De</kbd>) and only ambient lighting.


## Selective view-dependent mesh refinement

Within `demos/create_sr_office`, the script call
```shell
meshtopm.{sh,bat} demos/data/office.nf80000.orig.m -vsgeom >office.sr.pm
```
creates a progressive mesh in which the simplified vertices are constrained to lie
at their original positions (`-vsgeom`).
This enables selective refinement, demonstrated by
```shell
G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d \
  -key ,DnDeDoDb -lightambient .4 -sr_screen_thresh .002 -frustum_frac 2
```

The mesh is adaptively refined within the view frustum, shown as the inset rectangle (key <kbd>Do</kbd>)
or in the top view (key <kbd>Dr</kbd>).  Drag the mouse buttons to rotate, pan, and dolly the object.


## Terrain level-of-detail control

Within `demos/create_sr_terrain.{sh,bat}`,
```shell
Filterimage demos/data/gcanyon_elev_crop.bw.png -tobw -elevation -step 6 -scalez 0.000194522 \
    -removekinks -tomesh | \
  Filtermesh -assign_normals >gcanyon_sq200.orig.m
demos/bin/meshtopm.{sh,bat} gcanyon_sq200.orig.m -vsgeom -terrain >gcanyon_sq200.pm
```
- converts an elevation image to a smoothed terrain grid mesh, and
- simplifies it to create a selectively refinable mesh.

Then, within `demos/view_sr_terrain.sh`,
```shell
(common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcanyon_fly_v98.s3d \
   -texturemap demos/data/gcanyon_color.1024.png -key DeDtDG -sr_screen_thresh .02292 -sr_gtime 64 \
   -lightambient .5"; \
 export G3D_REV_AUTO=1; \
 G3dOGL $common -geom 800x820+100+10 -key "&O" -key ,o----J | \
   G3dOGL $common -geom 800x820+920+10 -async -killeof -input -key Dg)
```

- opens two synchronized side-by-side windows of the same texture mapped terrain,
- in which the first windows shows the temporal pops resulting from instantaneous mesh operations,
- whereas the second window shows the smooth appearance provided by runtime geomorphs (<kbd>Dg</kbd>).

For large terrain meshes, we form a hierarchical progressive mesh by partitioning the terrain mesh into tiles,
simplifying each tile independently to form a progressive mesh,
stitching the progressive meshes together 2-by-2,
and recursively simplifying and merging at coarser pyramid levels.

An example is presented in `demos/create_terrain_hierarchy`.  It makes use of <a id="prog_StitchPM"></a>
```shell
StitchPM -rootname terrain.level0 -blockx 2 -blocky 2 -blocks 32 -stitch >terrain.level0.stitched.pm
```
to assemble each 2-by-2 set of progressive mesh tiles `terrain.level0.x{0,1}.y{0,1}.pm` at the finest level.

The script `demos/view_gcanyon_interactive` launches an interactive flythrough over a Grand Canyon terrain model,
using a progressive mesh precomputed from an original 4096&times;2048 height field.

Alternatively, `demos/view_gcanyon_frames` shows a real-time flythrough using a pre-recorded flight path,
whereby keystroke commands embedded within the input stream automatically change viewing modes.


## Topology simplification

The program **`MinCycles`** removes topological noise from a mesh
by iteratively pinching off the smallest nonseparating cycle of edges until a
specified criterion (cycle length, number of cycle edges, number of cycles, or mesh genus) is reached.

For example, within `demos/create_topologically_simplified.{sh,bat}`, <a id="prog_MinCycles"></a>
```shell
FilterPM demos/data/office.pm -nf 200000 -outmesh | \
  MinCycles - -fraccyclelength 1.2 -maxcyclelen 0.10 -closecycles | \
  G3dOGL -st demos/data/office.s3d -key DeDEJ---- -thickboundary 0 -lightambient .9
```
- extracts a mesh of 200000 faces from a progressive mesh,
- closes 46 topological handles to reduce the mesh genus from 50 to 4,
- where the final remaining handle would require a nonseparating cycle of length greater than `0.10`.
- speeding up the process by identifying approximately shortest nonseparating cycles
  within a factor 1.2 of optimal, and
- shows the resulting closed edge cycles (tagged as sharp) in blue.


<!--## Optimized mesh traversal

The program `MeshReorder` reorders the triangle faces (and optionally vertices) within a mesh
so as to exploit a GPU vertex cache to minimize memory bandwidth and shading cost.

For example, within `demos/create_vertexcache_result`,
```shell
Filtermesh demos/data/small_stell_dodecahedron4.nsub4.m -renumber -removeinfo | \
  MeshReorder - -fifo -cache_size 16 -analyze -meshify5 -color_corners 1 -analyze \
    >small_stell_dodecahedron4.nsub4.vertexcache.m
G3dOGL small_stell_dodecahedron4.nsub4.vertexcache.m -key DmDe -lightambient .70 -lightsource .35
```
- simulates traversal using a FIFO cache of 16 vertices and reports cache miss rates,
- optimizes the triangle face ordering and re-simulates the cache behavior,
- writes the mesh with corner colors that identify cache misses,
- and finally views the result.
-->


## <a id="prog_G3dOGL"></a>Geometry viewer

The **`G3dOGL`** program shows interactive rasterized renderings of 3D (and 2D) geometry,
represented as

- streams of polygons/polylines/points (`*.a3d` format),
- triangle meshes including geomorphs (`*.m`),
- progressive meshes (`*.pm`),
- encoded selectively refinable meshes (`*.srm`),
- progressive simplicial complexes (`*.psc`), or
- simple `*.ply` files.

Please see the many examples presented earlier.
The viewer can also read `*.frame` elements to position the viewer and the objects in world space.
Elements of `*.a3d`, `*.m`, and `*.frame` streams can all be interleaved in a single input stream.

The viewer can take image snapshots (see `demos/create_rendered_mechpart_image`) and
record videos (see `demos/create_rendered_mechpart_video`).

The mouse/keyboard UI controls include:
<pre>
 Mouse movements:
 left mouse:          rotate
 middle mouse:        pan
 right mouse:         dolly
 shift-left:          pan
 shift-middle mouse:  roll
 shift-right mouse:   zoom
 (mouse movements are with respect to current object; see '0-9' below)

 Important key strokes:
 ? : print complete list of keys
 D?: print list of keys prefixed by 'D'
 De: toggle edges
 Ds: toggle shading of faces
 Db: toggle backface culling
 Dm: toggle Gouraud/flat shading
 DP: save current window as an image file
 DS: toggle show some sliders
 S : toggle show some other sliders
 j : jump to a default viewpoint
 J : automatically rotate object
 D/: edit viewpoint filename
 , : read the viewpoint
 . : save the viewpoint
 0-9: select object (0=eye_frame, 1=first object, 2=second object...)
 u : display/hide current object
 N : select next object
 P : select previous object
 -=: decrease/increase the magnitude of all movements
 f : toggle flying (usually with '0' eye selected)
</pre>

To record a 6-second (360-frame) video of a rotating mesh and then view the resulting video:

```shell
G3dOGL demos/data/standingblob.orig.m -st demos/data/standingblob.s3d -key iioJ \
  -video 360 output_video.mp4
VideoViewer output_video.mp4
```

The related program **`G3dVec`** shows wireframe hidden-line-removed renderings of `*.a3d` streams and `*.m` meshes.
It can write vector-based Postscript figures (see `demos/view_hidden_line_removed`).

In both programs, the keys <kbd>?</kbd> and <kbd>D?</kbd> show a list of available keyboard commands.

## <a id="prog_VideoViewer"></a>Image/video viewer

The **`VideoViewer`** program enables interactive viewing and simple editing of both images and videos.
Again, the key <kbd>?</kbd> shows a list of available keyboard commands.
Press <kbd>pageup</kbd>/<kbd>pagedown</kbd> to quickly browse through the videos and/or images in a directory.
Audio is not currently supported.


## File formats

### Mesh (`*.m`)

See the documentation at the end of `libHh/GMesh.h`

A mesh is a set of vertices and faces.  These in turn also define edges and corners.
Arbitrary string tuples can be associated with vertices, faces, edges, and corners.
Examples of string tuples:
`{normal=(.1 .2 .3) rgb=(1 1 1) matid=5 material="string"}`.
See the several `demos/data/*.m` files for examples of the mesh format.
Note that the indices of vertices and faces start at 1 instead of 0;
in hindsight that was a poor choice.

### Geometry stream (`*.a3d`, `*.pts`)

See the documentation at the end of `libHh/A3dStream.h`

The stream contains polygons, polylines, points, and control codes
(like end-of-frame, end-of-input, change-of-object).
Unlike in a mesh, these primitives do not share vertices.  The stream can be either text or binary.

### Frame stream (`*.frame`, `*.s3d`)

See the documentation at the end of `libHh/FrameIO.h`

This text or binary format encodes a 4&times;3 affine transformation
(plus an object id and a scalar field-of-view zoom).
It is used to record default viewing configurations, and sequences of frames for flythroughs.
It usually represents the linear transform from object space (or eye space) to world space.
The stream can be either text or binary.

### Progressive mesh (`*.pm`)

This is a binary representation that consists of a coarse base mesh and a sequence of vertex split records.

### Edge collapse / vertex split records (`*.prog`, `*.rprog`)

These are temporary text files containing verbose information for a sequence of edge collapse / vertex split records
used by MeshSimplify / reverselines / Filterprog to create a progressive mesh.


## Libraries

The library <a href="https://github.com/hhoppe/Mesh-processing-library/tree/main/libHh">`libHh`</a>
contains the main reusable classes.
All files include `Hh.h` which sets up a common cross-platform environment.

The libraries <a href="https://github.com/hhoppe/Mesh-processing-library/tree/main/libHWin">`libHWin`</a>
and <a href="https://github.com/hhoppe/Mesh-processing-library/tree/main/libHWX">`libHWX`</a>
define different implementations of a simple windowing interface (class `HW`),
under `Win32` and `X Windows`, respectively.
Both implementations support `OpenGL` rendering.


## Code details

The include file <code>libHh/<b>RangeOp</b>.h</code> defines many functions that act on <em>ranges</em>,
which are containers or views for which `begin()` and `end()` are defined.
For example, the function call `hh::fill(ar, 1.f)` assigns the value `1.f` to all
elements in the array named `ar`,
and the function call `hh::mean(matrix)` computes the average value of all entries in the
named `matrix`.

The debugging macro <code><b>SHOW</b>(expr)</code> outputs `expr = ...` on `std::cerr`
and also returns `expr`.
It also accepts multiple arguments in which case it returns `void`.
For example, `SHOW(min(1, 2), "hello", 3*2)` outputs the line `min(1, 2)=1 hello 3*2=6`.
Note the special treatment of literal string values.

Unicode strings are stored using <b>UTF-8</b> encoding into ordinary `std::string` variables.
The functions `hh::widen()` and `hh::narrow()` convert to and from the
`std::wstring` UTF-16 encodings used in `Win32` system calls.

All files use end-of-line encodings based on Unix `'\n'` LF (rather than DOS `'\r\n'` CR+LF).
All streams are opened in binary mode.  This allows text and binary to coexist in the same file.


## License

See the file <a href="https://github.com/hhoppe/Mesh-processing-library/tree/main/LICENSE">`./LICENSE`</a>.

This project has adopted the <a href="https://opensource.microsoft.com/codeofconduct/">Microsoft Open Source Code of Conduct</a>.  For more information see the <a href="https://opensource.microsoft.com/codeofconduct/faq/">Code of Conduct FAQ</a> or contact <a href="mailto:opencode@microsoft.com">opencode@microsoft.com</a> with any additional questions or comments.
