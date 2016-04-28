
Core classes:

const Vec<T,n>          : statically sized constant array (std::array<T> with constructors and support for zero size)
Vec<T,n>                : same but modifiable data
CArrayView<T>           : view of a 1D fixed-size range of const T elements (i.e. { const T*, int n })
ArrayView<T>            : same but modifiable data
Array<T>                : heap-allocated stretchable 1D array (like std::vector but inherited from ArrayView)
PArray<T,n>             : like Array but preallocates n elements to avoid heap allocation for small arrays

const SGrid<T,D1,D2,...,Dn> : statically sized constant grid with n>=1 dimensions
SGrid<T,D1,D2,...,Dn>   : same but modifiable data
CGridView<D,T>          : view of a D-dimensional fixed-size grid of const T elements
GridView<D,T>           : same but modifiable data
Grid<D,T>               : heap-allocated D-dimensional grid (resize requires reinitialization of data)

