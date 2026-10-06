# libHh

The core library of the Mesh-processing-library: containers, geometry, meshes, images, video, and utilities.
Everything is in namespace `hh`.

## Arrays and views (one-dimensional)

- `Vec<T, n>`: fixed-size array of `n` elements, like `std::array<T, n>` but with constructors and support for
  `n == 0`; `Vec2<T>`, `Vec3<T>`, and `Vec4<T>` abbreviate the common sizes.  `V(a, b, ...)` creates a `Vec` whose
  element type and size are inferred from its arguments, e.g. `V(1.f, 2.f)` is a `Vec2<float>` (and `V<T>(...)`
  specifies the element type).
- `CArrayView<T>`: view of a contiguous range of const `T` elements (a `const T*` and a count); it can refer to a C
  array, `std::vector`, `Vec`, `Array`, `InlinedArray`, or a `Matrix` row.
- `ArrayView<T>`: same, with modifiable elements.
- `Array<T>`: heap-allocated resizable array, like `std::vector<T>`.
- `InlinedArray<T, n>`: resizable array whose first `n` elements use built-in storage, which avoids heap allocation
  for small arrays.
- `GeneralArray<T, n>`: the class of which `Array<T>` (with `n == 0`) and `InlinedArray<T, n>` are aliases; code
  names it directly only when it is generic over `n`, e.g. in a function that resizes any such array.
- `CStridedArrayView<T>`, `StridedArrayView<T>`: views whose elements are separated by a stride, e.g. a column of a
  `Matrix`.

The arrays derive from the views, so an array can be passed wherever a view is expected (solid lines show
derivation, and dashed lines show aliases):
```
CArrayView<T>                   (const elements)
└─ ArrayView<T>                 (modifiable elements)
   └─ GeneralArray<T, n>
      ├-- Array<T>              (= GeneralArray<T, 0>)
      └-- InlinedArray<T, n>    (= GeneralArray<T, n>)
```
A function that only reads or modifies elements should take a `CArrayView<T>` or `ArrayView<T>`, so that it accepts
any of these containers (a `Vec` also converts to them).

## Grids (any number of dimensions)

- `SGrid<T, D1, D2, ..., Dn>`: statically sized grid with `n >= 1` dimensions (nested `Vec`s).
- `CGridView<D, T>`: view of a contiguous `D`-dimensional grid of const `T` elements.
- `GridView<D, T>`: same, with modifiable elements.
- `Grid<D, T>`: heap-allocated `D`-dimensional grid; resizing it does not preserve its elements.
- `Matrix<T>`: alias of `Grid<2, T>`, with views `CMatrixView<T>` and `MatrixView<T>`.

As for arrays:
```
CGridView<D, T>             (const elements)
└─ GridView<D, T>           (modifiable elements)
   └─ Grid<D, T>
```

## Other containers

- `Map<Key, Value>`: hash map, a wrapper around `std::unordered_map`.
- `Set<T>`: hash set, a wrapper around `std::unordered_set`.
- `InlinedSet<T, n>`: set whose first `n` elements are searched linearly in built-in storage, before moving into a
  hash `Set`.
- `Queue<T>`: FIFO queue, a wrapper around `std::deque` with `dequeue()`.
- `Stack<T>`: LIFO stack, based on `std::vector`.
- `PriorityQueue<T>`: min-priority queue of elements with `float` priorities.
- `UpdatablePriorityQueue<T>`: priority queue whose elements can also be looked up, updated, and removed, using a
  hash map.
- `Graph<T>`: set of directed edges between elements, with fast iteration over the outgoing edges of an element.
- `UnionFind<T>`: equivalence classes of elements, as pairs of elements are unified.
- `IntrusiveList`: doubly linked list of `IntrusiveListNode` members embedded within other objects.
- `Pool`: custom memory allocation pool for the objects of a class.
- `STree<T>`: ordered set, a wrapper around `std::set`.

## Geometry

- `Point`, `Vector`: a position and a translation in 3D space (both `Vec3<float>`).
- `Frame`: affine transformation of 3D points and vectors, as a 4x3 matrix applied to row vectors.
- `Bbox<T, dim>`: axis-aligned bounding box in `dim` dimensions.
- `Kdtree<T, D>`: k-d tree of elements represented by bounding boxes in `D` dimensions.
- `Bary`, `Uv`: barycentric coordinates within a triangle, and 2D texture coordinates.
- `Polygon`: array of `Point`s, with normal, area, clipping, and intersection operations.
- `Quaternion`: unit quaternion representing a 3D rotation, with `slerp()` and `squad()` interpolation.
- `PointSpatial<T>`, `ObjectSpatial`, `SpatialSearch<T>`: uniform grid over the unit cube, for finding the elements
  nearest to a query point.
- `Vector4`, `VectorF<n>`: vectors of `float` accelerated using SSE or NEON instructions.
- `Lls`, `SparseLls`: linear least-squares solvers (dense QR or SVD, and sparse conjugate gradient).

## Meshes, images, and video

- `Mesh`: vertices, faces, and edges of a polygon mesh, with their topological relations.
- `GMesh`: geometric mesh (hence the "G"), a `Mesh` with a `Point` at each vertex and string attributes on its
  elements.
- `MeshSearch`: spatial index over a `GMesh`, for closest-point queries.
- `SubMesh`: subdivides a `GMesh`, maintaining the relationship between the subdivided mesh and the base mesh.
- `PMesh`: progressive mesh, a base mesh (`AWMesh`) with a sequence of `Vsplit` records; `PMeshIter` traverses its
  levels of detail.
- `SrMesh`: selectively refinable progressive mesh, for view-dependent refinement.
- `RA3dStream`, `WA3dStream`: reading and writing streams of polygons, polylines, and points (`*.a3d`).
- `Pixel`: RGBA color with 8-bit channels (a `Vec4<uint8_t>`).
- `Image`: 2D grid of `Pixel`s (a `Matrix<Pixel>`) with file input and output.
- `Video`: 3D grid of `Pixel`s with attributes such as frame rate, bit rate, and compression type.
- `RVideo`, `WVideo`: reading or writing a video one frame at a time; `VideoNv12` stores frames in YUV 4:2:0 (NV12).
- `Audio`: 2D grid of `float` samples (channels by samples), with a sample rate and bit rate.

## Utilities

- `Stat`: accumulates statistics (count, min, max, mean, deviation) of a stream of values.
- `Timer`: measures and reports elapsed and CPU times.
- `parallel_for()`: runs a loop body in parallel over a range, using a pool of threads.
- `ParseArgs`: parses command-line options and generates their usage text.
- `RFile`, `WFile`: input and output file streams, which also accept `-` (stdin/stdout), pipe commands, and
  compressed files.
- `Random`: deterministic random-number generator, so that results are reproducible across platforms.
- `HH_STAT(S)`, `HH_SSTAT(S, v)`: macros that accumulate a `Stat`, reported at program exit.

## Code details

The include file <code>libHh/<b>RangeOp</b>.h</code> defines many functions that act on <em>ranges</em>,
which are containers or views for which `begin()` and `end()` are defined.
For example, the function call `hh::fill(ar, 1.f)` assigns the value `1.f` to all
elements in the array named `ar`,
and the function call `hh::mean(matrix)` computes the average value of all entries in the
named `matrix`.

The debugging macro <code><b>SHOW</b>(expr)</code> outputs `expr = ...` on `stderr`
and also returns `expr`.
It also accepts multiple arguments in which case it returns `void`.
For example, `SHOW(min(1, 2), "hello", 3*2)` outputs the line `min(1, 2)=1 hello 3*2=6`.
Note the special treatment of literal string values.

Unicode strings are stored using <b>UTF-8</b> encoding into ordinary `std::string` variables.
The functions `hh::utf16_from_utf8()` and `hh::utf8_from_utf16()` convert to and from the
`std::wstring` UTF-16 encodings used in `Win32` system calls.

All files use end-of-line encodings based on Unix `'\n'` LF (rather than DOS `'\r\n'` CR+LF).
All streams are opened in binary mode.  This allows text and binary to coexist in the same file.
