// -*- C++ -*-
#ifndef MESH_PROCESSING_LIBHH_MAPPEDFILE_H_
#define MESH_PROCESSING_LIBHH_MAPPEDFILE_H_

#include "libHh/Hh.h"

namespace hh {

// A read-only view of the entire contents of a file, which is memory-mapped if possible.  A file that must be read
// through a pipe (e.g., "-" or "*.gz"; see file_requires_pipe()) is instead read into memory.  The contents may be
// binary: the view holds any bytes, including '\0', and is not null-terminated.
class MappedFile : noncopyable {
 public:
  explicit MappedFile(const string& filename);
  ~MappedFile();
  [[nodiscard]] std::string_view view() const { return _view; }

 private:
  std::string_view _view;
  void* _address{nullptr};  // The start of the mapped view, if any.
  string _contents;         // The contents of a file that is not mapped.
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MAPPEDFILE_H_
