// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/FileIO.h"

#include <fcntl.h>     // O_NOINHERIT, O_BINARY
#include <sys/stat.h>  // struct stat, struct _stat, stat()

#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>              // SetStdHandle(), STD_INPUT_HANDLE, FindFirstFile(), FindNextFile()
#include <io.h>                   // _pipe(), dup(), close(), etc.
#include <process.h>              // getpid(), _wspawnvp(), cwait()   (wspawnvp() has bad signature, at least in mingw)
#include <shellapi.h>             // SHFileOperation(), SHFILEOPSTRUCTW
#include <sys/utime.h>            // struct utimbuf, struct _utimbuf, utime(), _wutime()
HH_REFERENCE_LIB("shell32.lib");  // SHFileOperation()

#else

#include <dirent.h>  // struct dirent, opendir(), readdir(), closedir()
#include <sys/wait.h>  // wait(), waidpid()
#include <unistd.h>  // unlink(), getpid(), execvp(), etc.
#include <utime.h>  // struct utimbuf, struct _utimbuf, utime()

#endif  // defined(_WIN32)

#if defined(IO_USE_FSTREAM) || defined(IO_USE_STDIO_FILEBUF) || defined(IO_USE_CFSTREAM)
// use specified setting
#elif 0  // use to force for testing
#define IO_USE_CFSTREAM
#elif defined(_MSC_VER)
#define IO_USE_FSTREAM
#elif defined(__GNUC__) && !defined(__clang__)
#define IO_USE_STDIO_FILEBUF
#else
#define IO_USE_CFSTREAM
#endif

#if defined(IO_USE_STDIO_FILEBUF)
#include <ext/stdio_filebuf.h>  // __gnu_cxx::stdio_filebuf<char>
#endif

#include <atomic>
#include <cctype>  // std::isalnum()
#include <cstring>  // std::memmove()
#include <fstream>  // std::ifstream, std::ofstream
#include <mutex>  // mutex, lock_guard

#include "libHh/RangeOp.h"  // contains()
#include "libHh/StringOp.h"
#include "libHh/Vec.h"

// Note that RFile/WFile first construct a FILE* (which is accessible via cfile()), then a std::stream on top.
// This is quite flexible.  I use this in:
// - Image_libs.cpp so that libpng and libjpeg can work directly on FILE*; this could easily be worked around
//    because these libraries support user-defined reader/writer functions, which could access std::stream.
// - G3dio.cpp to create a RBuffer directly on the POSIX file descriptor fileno(cfile());
//   It might be possible to implement RBuffer as a custom adaptively resizable std::streambuf
//    but we would still require access to the POSIX fd to do non-blocking IO.

namespace hh {

namespace {

std::mutex s_mutex;  // for all popen(), pclose() operations in this file

FILE* my_popen(const string& scmd, const string& mode);
FILE* my_popen(CArrayView<string> scmd, const string& mode);
int my_pclose(FILE* file);

inline bool character_requires_quoting(char ch) {
  // cmd special characters: space, &()[]{}^=;!'+,`~
  return !std::isalnum(ch) && !contains(":/-@_.", ch);  // removed + and ,
}

bool string_requires_quoting(const string& s) {
  for_int(i, narrow_cast<int>(s.size())) {
    if (character_requires_quoting(s[i])) return true;
  }
  return false;
}

string portable_simple_quote(const string& s) { return string_requires_quoting(s) ? '"' + s + '"' : s; }

// ***

#if defined(IO_USE_CFSTREAM)

// Open fstream on (C stdio) FILE* or POSIX fd:
// http://stackoverflow.com/questions/2746168/how-to-construct-a-c-fstream-from-a-posix-file-descriptor

// related: http://stackoverflow.com/questions/10667543/creating-fstream-object-from-a-file-pointer

// related: http://stackoverflow.com/questions/14734091/how-to-open-custom-i-o-streams-from-within-a-c-program
//  solutions by Andy Prowl (unbuffered), James Kanze (buffered)

// related: http://stackoverflow.com/questions/4151504/wrapping-file-with-custom-stdostream

// http://www.josuttis.com/cppcode/fdstream.hpp   -- see ~/git/hh_src/_other/fdstream.h

// Also http://ilab.usc.edu/rjpeters/groovx/stdiobuf_8cc-source.html and
//   http://ilab.usc.edu/rjpeters/groovx/stdiobuf_8h-source.html

int seekdir_to_origin(std::ios_base::seekdir dir) {
  switch (dir) {
    case std::ios_base::beg: return SEEK_SET;
    case std::ios_base::end: return SEEK_END;
    case std::ios_base::cur: return SEEK_CUR;
    default: assertnever("");
  }
}

// An implementation of streambuf that reads from a (C stdio) FILE* input stream using a small buffer.
// Limitations: the FILE* cannot be simultaneously used for writing; the input data is buffered twice!
class icfstreambuf : public std::streambuf {
 public:
  icfstreambuf(FILE* file) : _file(file), _buffer0(buffer + putback_size) {
    setg(_buffer0, _buffer0, _buffer0);  // set empty: eback() == beg, gptr() == cur, egptr() == end are all the same
  }

 private:
  FILE* _file;
  static constexpr int putback_size = 4;
  static constexpr int buffer_size = 4096;
  char buffer[putback_size + buffer_size];
  char* const _buffer0;
  // base showmanyc() returns 0 to indicate we don't know how many characters are available
  virtual int_type underflow() override {  // add some characters to the buffer if empty
    if (gptr() < egptr()) return traits_type::to_int_type(*gptr());
    // copy up to putback_size previously read characters into putback buffer
    int putback_num = 0;
    if (1) {
      putback_num = min(int(gptr() - eback()), possible_cast<int>(putback_size));
      ASSERTX(putback_num >= 0 && putback_num <= putback_size);
      std::memmove(_buffer0 - putback_num, gptr() - putback_num, putback_num);  // ranges may overlap
    }
    size_t num = fread(_buffer0, sizeof(char), buffer_size, _file);
    if (!num) return EOF;  // on failure we retain gptr() == egptr()
    setg(_buffer0 - putback_num, _buffer0, _buffer0 + num);
    return traits_type::to_int_type(*gptr());
  }
  virtual pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override {
    Warning("untested");
    assertx(!(which & std::ios_base::out));
    if (which & std::ios_base::in) {
      if (assertw(!fseek(_file, assert_narrow_cast<long>(off), seekdir_to_origin(dir))) &&
          !(dir == std::ios_base::cur && off == 0))  // don't clear buffer if called from tellg()
        setg(_buffer0, _buffer0, _buffer0);          // only clear if successful
    }
    return ftell(_file);
  }
  virtual pos_type seekpos(pos_type pos, std::ios_base::openmode which) override {
    Warning("untested");
    assertx(!(which & std::ios_base::out));
    if (which & std::ios_base::in) {
      if (assertw(!fseek(_file, assert_narrow_cast<long>(pos), SEEK_SET))) setg(_buffer0, _buffer0, _buffer0);
    }
    return ftell(_file);
  }
};

// Create a std::fstream wrapper around a (C stdio) FILE* input stream (which is not closed upon destruction).
class icfstream : public std::istream {
 public:
  icfstream(FILE* file) : std::istream(nullptr), _buf(file) { rdbuf(&_buf); }

 private:
  icfstreambuf _buf;
};

// An implementation of streambuf that writes to a (C stdio) FILE* output stream.
// Limitations: the FILE* cannot be simultaneously used for reading.
class ocfstreambuf : public std::streambuf {
 public:
  ocfstreambuf(FILE* file) : _file(file) {}

 private:
  FILE* _file;
  virtual int_type overflow(int_type ch) override {  // write one character
    // setp() would set pbase() == beg, pptr() == cur, epptr() == end
    return ch == EOF ? ch : fputc(ch, _file);
  }
  virtual std::streamsize xsputn(const char* s, std::streamsize count) override {  // write multiple characters
    return fwrite(s, sizeof(char), assert_narrow_cast<size_t>(count), _file);
  }
  virtual pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override {
    Warning("untested");
    assertx(!(which & std::ios_base::in));
    if (which & std::ios_base::out) {
      assertw(!fseek(_file, assert_narrow_cast<long>(off), seekdir_to_origin(dir)));
    }
    return ftell(_file);
  }
  virtual pos_type seekpos(pos_type pos, std::ios_base::openmode which) override {
    Warning("untested");
    assertx(!(which & std::ios_base::in));
    if (which & std::ios_base::out) {
      assertw(!fseek(_file, assert_narrow_cast<long>(pos), SEEK_SET));
    }
    return ftell(_file);
  }
  virtual int sync() override { return fflush(_file); }
};

// Create a std::fstream wrapper around a (C stdio) FILE* output stream (which is not closed upon destruction).
class ocfstream : public std::ostream {
 public:
  ocfstream(FILE* file) : std::ostream(nullptr), _buf(file) { rdbuf(&_buf); }

 private:
  ocfstreambuf _buf;
};

#endif  // defined(IO_USE_CFSTREAM)

}  // namespace

// *** RFile::Implementation and WFile::Implementation

#if defined(IO_USE_FSTREAM)

class RFile::Implementation {
 public:
  explicit Implementation(FILE* file) : _ifstream(file) {}  // non-standard extension in VS
  std::istream* get_stream() { return &_ifstream; }

 private:
  std::ifstream _ifstream;
};

class WFile::Implementation {
 public:
  explicit Implementation(FILE* file) : _ofstream(file) {}  // non-standard extension in VS
  std::ostream* get_stream() { return &_ofstream; }

 private:
  std::ofstream _ofstream;
};

#elif defined(IO_USE_STDIO_FILEBUF)

class RFile::Implementation {
 public:
  Implementation(FILE* file) : _filebuf(file, std::ios_base::in), _istream(&_filebuf) {}
  std::istream* get_stream() { return &_istream; }

 private:
  __gnu_cxx::stdio_filebuf<char> _filebuf;  // gcc-specific
  std::istream _istream;
};

class WFile::Implementation {
 public:
  Implementation(FILE* file) : _filebuf(file, std::ios_base::out), _ostream(&_filebuf) {}
  std::ostream* get_stream() { return &_ostream; }

 private:
  __gnu_cxx::stdio_filebuf<char> _filebuf;  // gcc-specific
  std::ostream _ostream;
};

#elif defined(IO_USE_CFSTREAM)

class RFile::Implementation {
 public:
  Implementation(FILE* file) : _icfstream(file) {}
  std::istream* get_stream() { return &_icfstream; }

 private:
  icfstream _icfstream;  // cross-platform but less efficient due to extra buffering
};

class WFile::Implementation {
 public:
  Implementation(FILE* file) : _ocfstream(file) {}
  std::ostream* get_stream() { return &_ocfstream; }

 private:
  ocfstream _ocfstream;  // cross-platform but slightly less efficient due to extra API layer
};

#else
#error
#endif

// *** RFile

RFile::RFile(const string& filename) {
  std::lock_guard<std::mutex> lock(s_mutex);  // for popen(), and just to be safe, for fopen() as well
  string sfor = get_canonical_path(filename);
  const string mode = "r";
  if (ends_with(filename, "|")) {
    _file_ispipe = true;
    _file = my_popen(filename.substr(0, filename.size() - 1), mode);  // no quoting at all
  } else if (ends_with(filename, ".gz") || ends_with(filename, ".Z")) {
    _file_ispipe = true;
    _file = my_popen(V<string>("gzip", "-d", "-c", sfor), mode);  // gzip supports .Z (replacement for zcat)
  } else if (filename == "-") {
    // assertw(!HH_POSIX(isatty)(0));
    _file = stdin;
    _is = &std::cin;
  } else if (file_exists(sfor)) {
    if (!assertw(!file_exists(sfor + ".Z")) || !assertw(!file_exists(sfor + ".gz")))
      showdf("** Using uncompressed version of '%s'\n", sfor.c_str());
#if defined(_WIN32)
    _file = _wfopen(utf16_from_utf8(sfor).c_str(), L"rb");
#else
    _file = fopen(sfor.c_str(), "rb");
#endif
  } else if (file_exists(sfor + ".gz")) {
    _file_ispipe = true;
    _file = my_popen(V<string>("gzip", "-d", "-c", sfor + ".gz"), mode);
  } else if (file_exists(sfor + ".Z")) {
    _file_ispipe = true;
    _file = my_popen(V<string>("gzip", "-d", "-c", sfor + ".Z"), mode);
  }
  if (_file && !_is) {
    _impl = make_unique<Implementation>(_file);
    _is = _impl->get_stream();
  }
  if (!_is) throw std::runtime_error("Could not open file '" + filename + "' for reading");
}

RFile::~RFile() {
  std::lock_guard<std::mutex> lock(s_mutex);  // for pclose(), and just to be safe, for fclose() as well
  if (_file_ispipe) {
#if defined(_WIN32)
    // Avoids the "Broken pipe" error message, but takes too long for huge streams!
    if (0) _is->ignore(std::numeric_limits<int>::max());
#endif
  }
  _impl = nullptr;
  if (_file) {
    if (_file_ispipe) {
      int ret = my_pclose(_file);
      // PIPE signal may cause source process to return signal information, so ignore return value here.
      dummy_use(ret);
      if (0) assertw(!ret);
    } else {
      if (_file != stdin) assertw(!fclose(_file));
    }
  }
}

// *** WFile

WFile::WFile(const string& filename) {
  std::lock_guard<std::mutex> lock(s_mutex);  // for popen(), and just to be safe, for fopen() as well
  assertx(filename != "");
  string sfor = get_canonical_path(filename);
  const string mode = "w";
  if (filename[0] == '|') {
    _file_ispipe = true;
    _file = my_popen(filename.substr(1), mode);  // no quoting at all
  } else if (ends_with(filename, ".Z")) {
    _file_ispipe = true;
    _file = my_popen(("compress >" + portable_simple_quote(sfor)), mode);
  } else if (ends_with(filename, ".gz")) {
    _file_ispipe = true;
    _file = my_popen(("gzip >" + portable_simple_quote(sfor)), mode);
  } else if (filename == "-") {
    _file = stdout;
    _os = &std::cout;
  } else {
#if defined(_WIN32)
    _file = _wfopen(utf16_from_utf8(sfor).c_str(), L"wb");
#else
    _file = fopen(sfor.c_str(), "wb");
#endif
  }
  if (_file && !_os) {
    _impl = make_unique<Implementation>(_file);
    _os = _impl->get_stream();
  }
  if (!(_os && *_os)) throw std::runtime_error("Could not open file '" + filename + "' for writing");
}

WFile::~WFile() {
  std::lock_guard<std::mutex> lock(s_mutex);  // for pclose(), and just to be safe, for fclose() as well
  if (_os) _os->flush();
  _impl = nullptr;
  if (_file) {
    fflush(_file);
    if (_file_ispipe) {
      int ret = my_pclose(_file);
      assertw(!ret);
    } else {
      if (_file != stdout) assertw(!fclose(_file));
    }
  }
}

// *** Misc

bool file_exists(const string& name) {
#if defined(_WIN32)
  struct _stat fstat;
  if (_wstat(utf16_from_utf8(name).c_str(), &fstat)) return false;
#else
  struct stat fstat;
  if (stat(name.c_str(), &fstat)) return false;
#endif
  if (fstat.st_mode & S_IFDIR) {
    if (0) Warning("Found directory when expecting a file");
    if (0) SHOW("found directory '" + name + "' when expecting a file");
    return false;
  }
  return true;
}

bool directory_exists(const string& name) {
#if defined(_WIN32)
  struct _stat fstat;
  if (_wstat(utf16_from_utf8(name).c_str(), &fstat)) return false;
#else
  struct stat fstat;
  if (stat(name.c_str(), &fstat)) return false;
#endif
  if (fstat.st_mode & S_IFDIR) return true;  // directory
  return false;
}

bool file_requires_pipe(const string& name) {
  return (name == "-" || ends_with(name, ".Z") || ends_with(name, ".gz") || ends_with(name, "|") ||
          begins_with(name, "|"));
}

// ret: 0 if error
uint64_t get_path_modification_time(const string& name) {
#if defined(_WIN32)
  // https://msdn.microsoft.com/en-us/library/14h5k7ff.aspx
  struct _stat fstat;  // __time32_t st_mtime;
  if (_wstat(utf16_from_utf8(name).c_str(), &fstat)) return 0;
  return fstat.st_mtime;
#else
  struct stat fstat;
  if (stat(name.c_str(), &fstat)) return 0;
  return fstat.st_mtime;
#endif
}

// ret: success
bool set_path_modification_time(const string& name, uint64_t time) {
  // https://msdn.microsoft.com/en-us/library/4wacf567.aspx
#if defined(_WIN32)
  struct _utimbuf ut;
  ut.actime = time;
  ut.modtime = time;
  return !_wutime(utf16_from_utf8(name).c_str(), &ut);
#else
  struct utimbuf ut;
  ut.actime = time;
  ut.modtime = time;
  return !utime(name.c_str(), &ut);
#endif
}

namespace {
enum class EType { files, directories };
Array<string> get_in_directory(const string& directory, EType type) {
  // http://stackoverflow.com/questions/306533/how-do-i-get-a-list-of-files-in-a-directory-in-c
  // also http://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
  Array<string> ar_filenames;
#if defined(_WIN32)
  WIN32_FIND_DATAW file_data;
  HANDLE dir = FindFirstFileW(utf16_from_utf8(directory + "/*").c_str(), &file_data);
  {
    if (dir == INVALID_HANDLE_VALUE) return {};  // No files found
    do {
      string file_name = utf8_from_utf16(file_data.cFileName);
      const bool is_directory = !!(file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY);
      if ((type == EType::files && is_directory) || (type == EType::directories && !is_directory)) continue;
      if (is_directory && (file_name == "." || file_name == "..")) continue;
      ar_filenames.push(std::move(file_name));
    } while (FindNextFileW(dir, &file_data));
  }
  assertx(FindClose(dir));
#else
  DIR* dir = opendir(directory.c_str());
  {
    while (struct dirent* ent = readdir(dir)) {
      string file_name = ent->d_name;
      string path_name = directory + "/" + file_name;
      struct stat st;
      if (stat(path_name.c_str(), &st) == -1) continue;
      const bool is_directory = (st.st_mode & S_IFDIR) != 0;
      if ((type == EType::files && is_directory) || (type == EType::directories && !is_directory)) continue;
      if (is_directory && (file_name == "." || file_name == "..")) continue;
      ar_filenames.push(std::move(file_name));
    }
  }
  assertx(!closedir(dir));
#endif
  return ar_filenames;
}
}  // namespace

Array<string> get_files_in_directory(const string& directory) { return get_in_directory(directory, EType::files); }

Array<string> get_directories_in_directory(const string& directory) {
  return get_in_directory(directory, EType::directories);
}

bool command_exists_in_path(const string& name) {
  string s = getenv_string("PATH");
  const char pathsep = contains(s, ';') || contains(s, '\\') ? ';' : ':';
  string::size_type i = 0;
  for (;;) {
    auto j = s.find_first_of(pathsep, i);  // may equal string::npos
    string dir = s.substr(i, j - i);
    if (file_exists(dir + '/' + name) || file_exists(dir + '/' + name + ".bat") ||
        file_exists(dir + '/' + name + ".exe"))
      return true;
    if (j == string::npos) break;
    i = j + 1;
  }
  return false;
}

bool remove_file(const string& name) { return !HH_POSIX(unlink)(name.c_str()); }

bool recycle_path(const string& pathname) {
  // http://www.hardcoded.net/articles/send-files-to-trash-on-all-platforms.htm
#if defined(_WIN32)
  // https://msdn.microsoft.com/en-us/library/windows/desktop/bb762164%28v=vs.85%29.aspx
  // typedef struct _SHFILEOPSTRUCT {
  //   HWND         hwnd;
  //   UINT         wFunc;
  //   PCZZTSTR     pFrom;
  //   PCZZTSTR     pTo;
  //   FILEOP_FLAGS fFlags;
  //   BOOL         fAnyOperationsAborted;
  //   LPVOID       hNameMappings;
  //   PCTSTR       lpszProgressTitle;
  // } SHFILEOPSTRUCT, *LPSHFILEOPSTRUCT;
  assertx(is_path_absolute(pathname));
  std::wstring wfilenames;
  {
    string name = pathname;
    for_int(i, narrow_cast<int>(name.size())) {
      if (name[i] == '/') name[i] = '\\';
    }
    wfilenames = utf16_from_utf8(name);
    wfilenames.push_back(0);  // for double-null termination
  }
  SHFILEOPSTRUCTW op = {};
  op.hwnd = nullptr;     // hopefully this handle is not used when I specify FOF_NO_UI
  op.wFunc = FO_DELETE;  // use Recycle Bin if possible
  op.pFrom = wfilenames.data();
  op.fFlags = FOF_ALLOWUNDO;
  // FOF_NO_UI equivalent to FOF_SILENT | FOF_NOCONFIRMATION | FOF_NOERRORUI | FOF_NOCONFIRMMKDIR but not on clang
  op.fFlags |= FOF_SILENT | FOF_NOCONFIRMATION | FOF_NOERRORUI | FOF_NOCONFIRMMKDIR;
  int ret = SHFileOperationW(&op);
  if (1 && ret) SHOW("SHFileOperation failed", pathname, ret);
  if (!ret) assertx(!op.fAnyOperationsAborted);
  return !ret;
#else
  return remove_file(pathname);
#endif
}

TmpFile::TmpFile(const string& suffix) {
  static std::atomic<int> s_count{0};
  for_int(i, 10000) {
    int lcount = ++s_count;
    _filename = sform("TmpFile.%d.%d%s%s", HH_POSIX(getpid)(), lcount, (suffix == "" ? "" : "."), suffix.c_str());
    if (!file_exists(_filename)) return;
  }
  assertnever("");
}

TmpFile::~TmpFile() {
  if (!getenv_bool("TMPFILE_KEEP")) assertx(remove_file(_filename));
}

// *** Quoting

// Notes on cygwin double-quote problem:
//
// It appears that the prefix (c:) of the command name affects the parsing of the arguments.
//  (here sh == /cygwin/bin/sh)
// sh -c ~/git/hh_src/bin/win/HTest\ -showargs\ \\\"ab
//  # Arg00='\ab'
// sh -c c:/hh/git/hh_src/bin/win/HTest\ -showargs\ \\\"ab
//  # Arg00='\ab'
// sh -c /hh/git/hh_src/bin/win/HTest\ -showargs\ \\\"ab
//  # Arg00='"ab'
// sh -c /cygdrive/c/hh/git/hh_src/bin/win/HTest\ -showargs\ \\\"ab\ c:/dummy
//  # Arg00='"ab'
//  # Arg01='c:/dummy'
// (set path=(~/git/hh_src/bin/win $path:q); sh -c HTest\ -showargs\ \\\"ab\ c:/dummy)
//  # Arg00='"ab'
//  # Arg01='c:/dummy'

// When grep is invoked from Emacs, it appears as "c:\cygwin64\bin\grep -nH ..."

// Backslash all non-ordinary characters.
string quote_arg_for_sh(const string& s) {
  std::ostringstream oss;
  for_int(i, narrow_cast<int>(s.size())) {
    char ch = s[i];
    if (character_requires_quoting(s[i])) oss << '\\';
    oss << ch;
  }
  return assertx(oss).str();
}

string quote_arg_for_shell(const string& s) {
  // Double quotes is not as general but should work in many cases for all shells.
  if (1) return portable_simple_quote(s);
  return quote_arg_for_sh(s);  // correct for csh and sh
}

// On Win32, used to quote each argument of a spawn() command.
static string windows_spawn_quote(const string& s) {
  if (!string_requires_quoting(s)) return s;
  std::ostringstream oss;
  oss << '"';
  for_int(i, narrow_cast<int>(s.size())) {
    char ch = s[i];
    if (ch == '"' || ch == '\\') {  // for both " and \ , move outside double-quotes and backslash it
      oss << '"' << '\\' << ch << '"';
    } else {
      oss << ch;
    }
  }
  oss << '"';
  if (0) SHOW("windows_spawn_quote", assertx(oss).str());
  return assertx(oss).str();
}

// On cygwin, used to quote each argument of a spawn() command.
static string cygwin_spawn_quote(const string& s) {
  if (!string_requires_quoting(s)) return s;
  std::ostringstream oss;
  oss << '"';
  for_int(i, narrow_cast<int>(s.size())) {
    char ch = s[i];
    if (ch == '"' || ch == '\\') {  // for both " and \ , backslash it (without moving it outside double-quotes)
      oss << '\\' << ch;
    } else {
      oss << ch;
    }
  }
  oss << '"';
  if (0) SHOW("cygwin_spawn_quote", assertx(oss).str());
  return assertx(oss).str();
}

// Under Windows, must quote the arguments in spawn call so that they are properly parsed by client CRT.
// Note that we do not know ahead of time if client uses cygwin or not.
// I assume that "sh" is always a cygwin process, and launch everything else from within "sh -c".
static string spawn_quote(const string& s, bool b_client_uses_cygwin) {
  return b_client_uses_cygwin ? cygwin_spawn_quote(s) : windows_spawn_quote(s);
}

// Returns: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_spawn(CArrayView<string> sargv, bool wait) {
  dummy_use(spawn_quote);
  assertx(sargv.num());
  assertx(sargv[0] != "");
  assertw(!contains(getenv_string("CYGWIN"), "noglob"));
#if defined(_WIN32)
  {
    const int mode = wait ? P_WAIT : P_NOWAIT;
    Array<std::wstring> nargv(sargv.num());
    const bool b_client_uses_cygwin = sargv[0] == "sh";  // special processing for cygwin crt parsing
    const bool b_client_cmd = sargv[0] == "cmd";
    if (b_client_uses_cygwin) {
      // 2015-11-20:
      // "Cygwin uses UTF-8 by default. To use a different character set, you need to set
      //   the LC_ALL, LC_CTYPE or LANG environment variables."
      // Not very understandable: https://cygwin.com/cygwin-ug-net/setup-locale.html
      // This next line seems to enable the launch of ffmpeg on ~/data/video/braille_*HDbrink8h.mp4
      if (1) my_setenv("LC_ALL", "en_US.CP437");
    }
    if (!b_client_cmd && begins_with(sargv[0], "cmd")) Warning("Unexpected cmd command");
    if (b_client_cmd && !(sargv.num() == 3 && sargv[1] == "/s/c")) {
      SHOW(sargv);
      Warning("Unexpected cmd args");
    }
    for_int(i, sargv.num()) {
      const char dq = '"';
      // Special flag "/s" in "cmd /s/c command" always expects outer quotes around command.
      string str =
          (b_client_cmd ? (i < 2 ? sargv[i] : (dq + sargv[i] + dq)) : spawn_quote(sargv[i], b_client_uses_cygwin));
      if (0) SHOW(i, str);
      nargv[i] = utf16_from_utf8(str);
    }
    Array<const wchar_t*> argv(sargv.num() + 1);
    for_int(i, sargv.num()) argv[i] = nargv[i].c_str();
    argv.last() = nullptr;
    // Adapted from crt/system.c; env == nullptr means inherit environment
    // One problem is that there is no way to hide the resulting console window
    //  (created if the current process does not already have a console window)
    //  because CreateProcess() call is hidden within CRT/dospawn.c and its StartupInfo structure
    //  does not specify ".wShowWindow = SW_HIDE" (http://stackoverflow.com/questions/4743559/).
    return _wspawnvp(mode, utf16_from_utf8(sargv[0]).c_str(), argv.data());
  }
#else  // Unix (or cygwin)
  {
    Array<const char*> argv(sargv.num() + 1);
    argv.last() = nullptr;
    for_int(i, sargv.num()) argv[i] = sargv[i].c_str();
    if (wait) {
      pid_t pid = fork();
      assertx(pid >= 0);                      // assert that fork() succeeded
      if (!pid) {                             // child process
        if (0) assertx(!HH_POSIX(close)(0));  // no need to read from stdin?
        execvp(argv[0], const_cast<char**>(argv.data()));
        exit(127);  // exec() failed; report same exit code as failed system()
      }
      int status;
      assertx(waitpid(pid, &status, 0) == pid);
      // TODO: determine which headers to use: sys/types.h  sys/wait.h?
      if (!WIFEXITED(status)) return -1;          // could have been signal
      if (WEXITSTATUS(status) == 127) return -1;  // exit code generated above for failed exec()
      return WEXITSTATUS(status);
    } else {
      // Async spawn is tricky; note that system("(command &)") does not report whether command was found.
      // Enhanced from http://lubutu.com/code/spawning-in-unix
      int fd[2];
      assertx(pipe(fd) != -1);
      pid_t pid = fork();
      assertx(pid >= 0);                   // assert that fork() succeeded  (else close(fd[0]), close(fd[1]))
      if (!pid) {                          // child process
        assertx(!HH_POSIX(close)(fd[0]));  // no need to read from parent process
        pid = fork();
        if (pid > 0) {
          // grandchild pid back to parent
          int64_t t = pid;
          assertx(HH_POSIX(write)(fd[1], &t, sizeof(t)) == sizeof(t));
          exit(0);
        }
        if (!pid) {                                    // grandchild process
          if (fcntl(fd[1], F_SETFD, FD_CLOEXEC) == 0)  // (close pipe if exec succeeds)
            execvp(argv[0], const_cast<char**>(argv.data()));
        }
        // something failed: the most recent fork(), the fcntl(), or the exec().
        assertx(errno > 0);
        int64_t t = -errno;
        assertx(write(fd[1], &t, sizeof(t)) == sizeof(t));
        exit(1);  // this exit code is not accessed by parent
      }
      ::wait(nullptr);                   // (I don't understand the reason/need for this)
      assertx(!HH_POSIX(close)(fd[1]));  // no need to write to child process
      pid = -1;                          // expect to read back a process id from child
      for (;;) {                         // outputs from child or grandchild may come in any order
        int64_t t;
        int nread = HH_POSIX(read)(fd[0], &t, sizeof(t));
        assertx(nread >= 0);
        SHOW(t);  // ?
        if (!nread) break;
        if (t < 0)
          errno = narrow_cast<int>(-t);
        else
          pid = narrow_cast<pid_t>(t);
      }
      assertx(!HH_POSIX(close)(fd[0]));
      return pid;
    }
  }
#endif
}

// Returns: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_sh(const string& scmd, bool wait) {
  assertx(scmd != "");
  const bool debug = getenv_bool("MY_SH_DEBUG");
  intptr_t ret = -1;
  if (debug) SHOW(scmd, getenv_string("PATH"));
  // SH
  if (ret < 0) ret = my_spawn(V<string>("sh", "-c", scmd), wait);
  if (ret < 0 && debug) Warning("Shell 'sh' not found");
  // CSH
  if (ret < 0) ret = my_spawn(V<string>("csh", "-c", scmd), wait);
  if (ret < 0 && debug) Warning("Shell 'csh' not found");
  if (ret < 0) Warning("Neither 'sh' nor 'csh' shells were found; resorting to 'cmd'");
  // CMD
  if (0 && ret < 0) SHOW(scmd);
#if !defined(_WIN32)
  if (ret < 0) Warning("Failed to find sh/csh (outside WIN32); highly odd");
#endif
  if (ret < 0) ret = my_spawn(V<string>("cmd", "/s/c", scmd), wait);  // my_spawn() adds double-quotes for /s option
  if (ret < 0 && debug) Warning("Shell 'cmd' not found");
  if (ret < 0) Warning("Could not spawn shell command (sh/csh/cmd)");
  return ret;
}

intptr_t my_sh(CArrayView<string> sargv, bool wait) {
  assertx(sargv.num() > 0);
  string scmd;
  for_int(i, sargv.num()) {
    if (i) scmd += ' ';
    scmd += quote_arg_for_sh(sargv[i]);
  }
  if (0) return my_sh(scmd, wait);  // inferior path because would not adapt quoting to cmd
  // Adapt the quoting of the arguments depending on which shell is invoked.
  const bool debug = getenv_bool("MY_SH_DEBUG");
  intptr_t ret = -1;
  if (debug) SHOW(sargv, scmd, getenv_string("PATH"));
  // SH
  if (ret < 0) {
    ret = my_spawn(V<string>("sh", "-c", scmd), wait);
    if (ret < 0 && debug) Warning("Shell 'sh' not found");
  }
  // CSH
  if (ret < 0) {
    ret = my_spawn(V<string>("csh", "-c", scmd), wait);
    if (ret < 0 && debug) Warning("Shell 'csh' not found");
  }
  // CMD
  // (cd ~/tmp; cp -p ~/bin/sys/gzip.exe .; set path=(. ~/git/mesh_processing/bin c:/windows/system32 c:/windows); Filtermesh ~/data/mesh/"complex file name.m" -stat)
  if (ret < 0) {
    if (1) Warning("Neither 'sh' nor 'csh' shells were found; resorting to 'cmd'");
    scmd = "";
    for_int(i, sargv.num()) {
      if (i) scmd += ' ';
      scmd += windows_spawn_quote(sargv[i]);  // unsure about this
    }
    if (0) SHOW(scmd);
    ret = my_spawn(V<string>("cmd", "/s/c", scmd), wait);  // my_spawn() adds double-quotes for /s option
    if (ret < 0 && debug) Warning("Shell 'cmd' not found");
  }
  if (ret < 0) Warning("Could not spawn shell command (sh/csh/cmd)");
  return ret;
}

namespace {

template <typename Ch, typename Traits = std::char_traits<Ch>>
struct basic_nullbuf : std::basic_streambuf<Ch, Traits> {
  using base_type = std::basic_streambuf<Ch, Traits>;
  using int_type = typename base_type::int_type;
  using traits_type = typename base_type::traits_type;
  int_type overflow(int_type c) override { return traits_type::not_eof(c); }
};

using nullbuf = basic_nullbuf<char>;
nullbuf null_obj;

}  // namespace

std::ostream cnull{&null_obj};

namespace {

// *** my_popen(), my_pclose()

#if !defined(_WIN32)

FILE* my_popen(const string& scmd, const string& mode) {
  // Note that "g++ -std=c++XX ..." sets -D__STRICT_ANSI__ which hides from the header files the functions
  //  popen(), pclose(), usleep(), posix_memalign(), setenv(), unsetenv(), gethostname().
  // Therefore we must compile with "g++ -std=c++XX -U__STRICT_ANSI__ ...".
  return popen(scmd.c_str(), mode.c_str());
}

FILE* my_popen(CArrayView<string> sargv, const string& mode) {
  string scmd;
  for_int(i, sargv.num()) {
    if (i) scmd += ' ';
    scmd += quote_arg_for_sh(sargv[i]);
  }
  return my_popen(scmd, mode);
}

int my_pclose(FILE* file) { return pclose(file); }

#else  // defined(_WIN32)

// Adapt popen() to use sh/csh if possible (rather than cmd) so that
// - it works within UNC directories
// - it uses '#!' convention for sh/bash/csh/perl scripts
// Inspired from:
// - MSDN "_pipe() example BeepFilter.Cpp"
// - popen.c in vc98/crt/src
// - MSDN "Creating a Child Process with Redirected Input and Output"

// One cannot use any of the following characters in a file name:  \ / ? : * " > < |

// From osfinfo.c, defined in io.h:
//  intptr_t __cdecl _get_osfhandle(int fh);
//  extern "C" int __cdecl _set_osfhnd(int fh, intptr_t value);

const int tot_fd = 512;  // max of _NHANDLE_ in internal.h
intptr_t* popen_pid = nullptr;

template <typename Tcmd>  // const string& or CArrayView<string>
FILE* my_popen_internal(const Tcmd& tcmd, const string& mode) {
  assertx(mode == "rb" || mode == "wb");
  bool is_read = mode == "rb";
  if (0) SHOW(getenv_string("PATH"));
  fflush(stdout);
  fflush(stderr);
  std::cout.flush();
  std::cerr.flush();
  if (!popen_pid) popen_pid = new intptr_t[tot_fd];  // never deleted
  int stdhdl = is_read ? 1 : 0;
  const int bufsize = 1024;  // tried larger size for faster mp4 read in FF_RVideo_Implementation, but no effect
  Vec<int, 2> pipes;
  assertx(!_pipe(pipes.data(), bufsize, O_BINARY | O_NOINHERIT));
  assertx(pipes[0] > 2 && pipes[0] < tot_fd && pipes[1] > 2 && pipes[1] < tot_fd);
  int new_stdhdl = pipes[is_read ? 1 : 0];
  int pipehdl = pipes[is_read ? 0 : 1];
  int bu_stdhdl = HH_POSIX(dup)(stdhdl);
  assertx(bu_stdhdl > 2);
  if (0) {
    SHOW(stdhdl, pipehdl, new_stdhdl, bu_stdhdl);
    SHOW(_get_osfhandle(0), _get_osfhandle(1), _get_osfhandle(2));
    SHOW(_get_osfhandle(pipes[0]), _get_osfhandle(pipes[1]));
    SHOW(_get_osfhandle(new_stdhdl), _get_osfhandle(bu_stdhdl));
  }
  assertx(HH_POSIX(dup2)(new_stdhdl, stdhdl) >= 0);  // success is either 0 or stdhdl depending on POSIX or Windows
  // SetStdHandle(STD_INPUT_HANDLE & STD_OUTPUT_HANDLE) done automatically
  //  by _set_osfhnd() called from dup2(), only for _CONSOLE_APP !
  if (stdhdl == 0) assertx(SetStdHandle(STD_INPUT_HANDLE, HANDLE(_get_osfhandle(stdhdl))));
  if (stdhdl == 1) assertx(SetStdHandle(STD_OUTPUT_HANDLE, HANDLE(_get_osfhandle(stdhdl))));
  assertx(!HH_POSIX(close)(new_stdhdl));
  if (0) {
    SHOW(_get_osfhandle(0), _get_osfhandle(1), _get_osfhandle(2));
    SHOW(_get_osfhandle(bu_stdhdl), _get_osfhandle(pipehdl));
  }
  // Ideally, child should not inherit bu_stdhdl;
  //  no easy way to do that without directly using DuplicateHandle().
  //  (pipehdl is not inherited by child process)
  intptr_t pid = my_sh(tcmd, false);
  if (pid < 0) {
    SHOW("Could not launch shell (not in path?)");
    return nullptr;
  }
  popen_pid[pipehdl] = pid;
  assertx(HH_POSIX(dup2)(bu_stdhdl, stdhdl) >= 0);  // success is either 0 or stdhdl depending on posix or iso
  // if (stdhdl == 0) assertx(SetStdHandle(STD_INPUT_HANDLE,  HANDLE(_get_osfhandle(stdhdl))));
  // if (stdhdl == 1) assertx(SetStdHandle(STD_OUTPUT_HANDLE, HANDLE(_get_osfhandle(stdhdl))));
  assertx(!HH_POSIX(close)(bu_stdhdl));
  return assertx(HH_POSIX(fdopen)(pipehdl, mode.c_str()));  // closed by fclose() in my_pclose_internal() below
}

int my_pclose_internal(FILE* file) {
  int fd = HH_POSIX(fileno)(file);
  assertx(!fclose(file));
  assertx(popen_pid);
  intptr_t proc_handle = popen_pid[fd];
  int termstat;
  intptr_t handle = HH_POSIX(cwait)(&termstat, proc_handle, WAIT_CHILD);
  if (handle != proc_handle) return -1;
  return termstat;
}

FILE* my_popen(const string& scmd, const string& mode) {
  string mode2 = mode + "b";  // always binary format
  return my_popen_internal(scmd, mode2);
}

FILE* my_popen(CArrayView<string> scmd, const string& mode) {
  string mode2 = mode + "b";  // always binary format
  return my_popen_internal(scmd, mode2);
}

int my_pclose(FILE* file) { return my_pclose_internal(file); }

#endif  // defined(_WIN32)

}  // namespace

}  // namespace hh
