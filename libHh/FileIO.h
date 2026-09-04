// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FILEIO_H_
#define MESH_PROCESSING_LIBHH_FILEIO_H_

#include "libHh/Array.h"  // CArrayView

namespace hh {

// Create a read stream (FILE and istream) from a file, compressed file, URL, or input pipe command.
// Supports "-" (std::cin), ".Z", ".gz", "https://...", and "command args... |".
// Note that it is best to close stdin within an input pipe: "command </dev/null ... |"!
class RFile : noncopyable {
 public:
  explicit RFile(string filename);
  ~RFile();
  [[nodiscard]] std::istream& operator()() const { return *_is; }
  [[nodiscard]] FILE* cfile() { return _file; }

 private:
  bool _file_isstd{false};
  bool _file_ispipe{false};
  FILE* _file{nullptr};
  class Implementation;
  unique_ptr<Implementation> _impl;
  std::istream* _is{nullptr};
};

// Create a write stream (FILE and ostream) to a file, compressed file, or output pipe command.
// Supports "-" (std::cout), ".Z", ".gz", and "| command args...".
class WFile : noncopyable {
 public:
  explicit WFile(string filename);
  ~WFile();
  [[nodiscard]] std::ostream& operator()() const { return *_os; }
  [[nodiscard]] FILE* cfile() { return _file; }

 private:
  bool _file_isstd{false};
  bool _file_ispipe{false};
  FILE* _file{nullptr};
  class Implementation;
  unique_ptr<Implementation> _impl;
  std::ostream* _os{nullptr};
};

// Assert that we have read to the end-of-file.
inline void assert_reached_eof(std::istream& is) {
  char ch;
  is.get(ch);
  assertx(!is);
}

// Check if a file already exists.
[[nodiscard]] bool file_exists(const string& name);

// Check if a folder already exists.
[[nodiscard]] bool directory_exists(const string& name);

// Check if filename refers to a pipe command ("command |" or "| command").
[[nodiscard]] bool is_pipe(const string& name);

// Check if filename is a URL ("https://" or "http://").
[[nodiscard]] bool is_url(const string& name);

// Check if filename would be read using a pipe stream ("-", ".Z", ".gz", "| command", "command |", "https://").
[[nodiscard]] bool file_requires_pipe(const string& name);

// Retrieve modification time of file or directory.
[[nodiscard]] uint64_t get_path_modification_time(const string& name);

// Set modification time of file or directory; return: success.
bool set_path_modification_time(const string& name, uint64_t time);

// Return an unsorted list of files in a directory (prefix each by "directory + '/'" to get absolute path).
[[nodiscard]] Array<string> get_files_in_directory(const string& directory);

// Return an unsorted list of directories in a directory (prefix each by "directory + '/'" to get absolute path).
[[nodiscard]] Array<string> get_directories_in_directory(const string& directory);

// Check if my_sh() would find this command in the current PATH.
[[nodiscard]] bool command_exists_in_path(const string& name);

// Return: success.
bool remove_file(const string& name);

// Delete a file or directory by moving it to the Recycle Bin / Trash if possible.  Return: success.
bool recycle_path(const string& pathname);

// Create the name for a temporary file, and delete the file when going out of scope.
class TmpFile : noncopyable {
 public:
  TmpFile(const string& suffix = "");
  TmpFile(const string& suffix, std::istream& is);  // Create the temporary file with the contents of the input stream.
  ~TmpFile();
  [[nodiscard]] string filename() const { return _filename; }
  void write_to(std::ostream& os) const;  // Write the temporary file into the output stream.
 private:
  string _filename;
};

// For sh/bash/csh/tcsh argument.
[[nodiscard]] string quote_arg_for_sh(const string& s);

// For sh/bash/csh/tcsh/cmd argument.
[[nodiscard]] string quote_arg_for_shell(const string& s);

// Return: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_spawn(CArrayView<string> sargv, bool wait);

// Run command s (already properly quoted) using shell sh or csh or cmd, in that order.
// (The quoting in s may be fragile if we must resort to shell cmd.)
// Return: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_sh(const string& command, bool wait = true);

// Run command words sargv (after quoting them) using shell sh or csh or cmd, in that order.
// Return: -1 if spawn error, else exit_code (for wait == true) or pid (for wait == false).
intptr_t my_sh(CArrayView<string> sargv, bool wait = true);

// Null output stream (which silently gobbles up all output).
extern std::ostream cnull;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FILEIO_H_
