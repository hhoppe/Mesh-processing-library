// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FILEIO_H_
#define MESH_PROCESSING_LIBHH_FILEIO_H_

#include "libHh/Array.h"  // CArrayView

namespace hh {

// Create a read stream from a file (FILE and/or istream); supports file decompression and input pipe commands.
class RFile : noncopyable {
 public:
  // supports "-", ".Z", ".gz", "command args... |"  (in most cases, try to close stdin within "command |")
  explicit RFile(const string& filename);
  ~RFile();
  std::istream& operator()() const { return *_is; }
  FILE* cfile() { return _file; }

 private:
  bool _file_ispipe{false};
  FILE* _file{nullptr};
  class Implementation;
  unique_ptr<Implementation> _impl;
  std::istream* _is{nullptr};
};

// Create a write stream to a file (FILE and/or ostream); supports file compression and output pipe commands.
class WFile : noncopyable {
 public:
  // supports "-", ".Z", ".gz", "| command args..."
  explicit WFile(const string& filename);
  ~WFile();
  std::ostream& operator()() const { return *_os; }
  FILE* cfile() { return _file; }

 private:
  bool _file_ispipe{false};
  FILE* _file{nullptr};
  class Implementation;
  unique_ptr<Implementation> _impl;
  std::ostream* _os{nullptr};
};

// Return true if we have read to the end-of-file.
inline bool reached_eof(std::istream& is) {
  char ch;
  is.get(ch);
  return !is;
}

// Checks if a file already exists.
bool file_exists(const string& name);

// Checks if a folder already exists.
bool directory_exists(const string& name);

// Checks if filename would be read using a pipe stream ("-", ".Z", ".gz", "| command", "command |").
bool file_requires_pipe(const string& name);

// Retrieve modification time of file or directory.
uint64_t get_path_modification_time(const string& name);

// Set modification time of file or directory; return: success.
bool set_path_modification_time(const string& name, uint64_t time);

// Returns an unsorted list of files in a directory (prefix each by "directory + '/'" to get absolute path).
Array<string> get_files_in_directory(const string& directory);

// Returns an unsorted list of directories in a directory (prefix each by "directory + '/'" to get absolute path).
Array<string> get_directories_in_directory(const string& directory);

// Checks if my_sh() would find this command in the current PATH.
bool command_exists_in_path(const string& name);

// Returns success.
bool remove_file(const string& name);

// Deletes a file/directory to the Recycle Bin / Trash if possible.  Ret: success.
bool recycle_path(const string& pathname);

// Creates the name for a temporary file, and deletes the file when going out of scope.
class TmpFile : noncopyable {
 public:
  TmpFile(const string& suffix = "");
  ~TmpFile();
  string filename() const { return _filename; }

 private:
  string _filename;
};

// For sh/csh argument.
string quote_arg_for_sh(const string& s);

// For sh/csh/cmd argument.
string quote_arg_for_shell(const string& s);

// Returns: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_spawn(CArrayView<string> sargv, bool wait);

// Run command s (already properly quoted) using shell sh or csh or cmd, in that order.
// (The quoting in s may be fragile if we must resort to shell cmd.)
// Returns: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_sh(const string& scmd, bool wait = true);

// Run command words sargv (after quoting them) using shell sh or csh or cmd, in that order.
// Returns: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_sh(CArrayView<string> sargv, bool wait = true);

// Null output stream (which silently gobbles up all output).
extern std::ostream cnull;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FILEIO_H_
