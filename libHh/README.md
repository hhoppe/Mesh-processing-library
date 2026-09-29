# libHh

The core library of the Mesh-processing-library: containers, geometry, meshes, images, video, and utilities.
Everything is in namespace `hh`.

## Arrays and views

| Class | Description |
| --- | --- |
| `Vec<T, n>` | Fixed-size 1D array of `n` elements, like `std::array<T, n>` but with constructors and support for `n == 0`; `Vec2<T>`, `Vec3<T>`, and `Vec4<T>` abbreviate the common sizes. |
| `CArrayView<T>` | View of a contiguous 1D range of const `T` elements (a `const T*` and a count); it can refer to a C array, `std::vector`, `Vec`, `Array`, `InlinedArray`, or a `Matrix` row. |
| `ArrayView<T>` | Same, with modifiable elements. |
| `Array<T>` | Heap-allocated resizable 1D array, like `std::vector<T>` but derived from `ArrayView<T>`. |
| `InlinedArray<T, n>` | Like `Array<T>`, but with built-in storage for its first `n` elements, which avoids heap allocation for small arrays. |

A function that only reads or modifies elements should take a `CArrayView<T>` or `ArrayView<T>`, so that it accepts
any of these containers.

## Grids

| Class | Description |
| --- | --- |
| `SGrid<T, D1, D2, ..., Dn>` | Statically sized grid with `n >= 1` dimensions (nested `Vec`s). |
| `CGridView<D, T>` | View of a contiguous `D`-dimensional grid of const `T` elements. |
| `GridView<D, T>` | Same, with modifiable elements. |
| `Grid<D, T>` | Heap-allocated `D`-dimensional grid; resizing it does not preserve its elements. |
| `Matrix<T>` | Alias of `Grid<2, T>`, with views `CMatrixView<T>` and `MatrixView<T>`. |

## Other containers

| Class | Description |
| --- | --- |
| `Map<Key, Value>` | Hash map, a wrapper around `std::unordered_map`. |
| `Set<T>` | Hash set, a wrapper around `std::unordered_set`. |
| `InlinedSet<T, n>` | Set whose first `n` elements are searched linearly in built-in storage, before moving into a hash `Set`. |
| `Queue<T>` | FIFO queue, a wrapper around `std::deque` with `dequeue()`. |
| `Stack<T>` | LIFO stack, based on `std::vector`. |
| `PriorityQueue<T>` | Min-priority queue of elements with `float` priorities. |
| `UpdatablePriorityQueue<T>` | Priority queue whose elements can also be looked up, updated, and removed, using a hash map. |
| `Graph<T>` | Set of directed edges between elements, with fast iteration over the outgoing edges of an element. |
| `UnionFind<T>` | Equivalence classes of elements, as pairs of elements are unified. |
| `EList` | Doubly linked list of `EListNode` members embedded within other objects. |
| `Pool` | Custom memory allocation pool for the objects of a class. |

## Geometry

| Class | Description |
| --- | --- |
| `Point`, `Vector` | A position and a translation in 3D space (both `Vec3<float>`). |
| `Frame` | Affine transformation of 3D points and vectors, as a 4x3 matrix applied to row vectors. |
| `Bbox<T, dim>` | Axis-aligned bounding box in `dim` dimensions. |
| `Kdtree<T, D>` | k-d tree of elements represented by bounding boxes in `D` dimensions. |

## Meshes, images, and video

| Class | Description |
| --- | --- |
| `Mesh` | Vertices, faces, and edges of a polygon mesh, with their topological relations. |
| `GMesh` | `Mesh` with a `Point` at each vertex and string attributes on its elements. |
| `Pixel` | RGBA color with 8-bit channels (a `Vec4<uint8_t>`). |
| `Image` | 2D grid of `Pixel`s (a `Matrix<Pixel>`) with file input and output. |
| `Video` | 3D grid of `Pixel`s with attributes such as frame rate, bit rate, and compression type. |

## Utilities

| Name | Description |
| --- | --- |
| `Stat` | Accumulates statistics (count, min, max, mean, deviation) of a stream of values. |
| `Timer` | Measures and reports elapsed and CPU times. |
| `parallel_for()` | Runs a loop body in parallel over a range, using a pool of threads. |
