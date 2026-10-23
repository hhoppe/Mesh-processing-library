// -*- C++ -*-
#include "libHh/MappedFile.h"

#include <sstream>  // std::ostringstream

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>  // CreateFileW(), CreateFileMappingW(), MapViewOfFile(), UnmapViewOfFile()
#else
#include <fcntl.h>     // open()
#include <sys/mman.h>  // mmap(), munmap(), posix_madvise()
#include <sys/stat.h>  // fstat()
#include <unistd.h>    // close()
#endif

#include "libHh/FileIO.h"

namespace hh {

MappedFile::MappedFile(const string& filename) {
  if (file_requires_pipe(filename)) {
    RFile fi(filename);
    std::ostringstream oss;
    oss << fi().rdbuf();
    _contents = std::move(oss).str();
    _view = _contents;
    return;
  }
#if defined(_WIN32)
  HANDLE file = CreateFileW(utf16_from_utf8(filename).c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
  if (file == INVALID_HANDLE_VALUE) assertnever("Cannot open file '" + filename + "'");
  LARGE_INTEGER size;
  assertx(GetFileSizeEx(file, &size));
  if (size.QuadPart > 0) {
    HANDLE mapping = assertx(CreateFileMappingW(file, nullptr, PAGE_READONLY, 0, 0, nullptr));
    _address = assertx(MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0));
    assertx(CloseHandle(mapping));  // The view keeps the mapping alive.
    _view = std::string_view(static_cast<const char*>(_address), size_t(size.QuadPart));
  }
  assertx(CloseHandle(file));
#else
  const int fd = open(filename.c_str(), O_RDONLY);
  if (fd < 0) assertnever("Cannot open file '" + filename + "'");
  struct stat st;
  assertx(fstat(fd, &st) == 0);
  const size_t size = st.st_size;
  if (size > 0) {
    _address = mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (_address == MAP_FAILED) assertnever("Cannot map file '" + filename + "'");
    posix_madvise(_address, size, POSIX_MADV_SEQUENTIAL);
    _view = std::string_view(static_cast<const char*>(_address), size);
  }
  assertx(close(fd) == 0);
#endif
}

MappedFile::~MappedFile() {
  if (!_address) return;
#if defined(_WIN32)
  assertw(UnmapViewOfFile(_address));
#else
  assertw(munmap(_address, _view.size()) == 0);
#endif
}

}  // namespace hh
