// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STRINGOP_H_
#define MESH_PROCESSING_LIBHH_STRINGOP_H_

#include "libHh/Hh.h"

namespace hh {

// Does a string contain a character?  (Subsumed by the general contains() function in RangeOp.h.)
// inline bool contains(const string& str, char ch) {
//     return str.find(ch) != string::npos;
// }

// If prefix is not at the start of s, returns false; else erases prefix from s and returns true.
inline bool remove_at_start(string& s, std::string_view prefix) {
  if (!s.starts_with(prefix)) return false;
  s.erase(0, prefix.size());
  return true;
}

// If suffix is not at the end of s, returns false; else erases suffix from s and returns true.
inline bool remove_at_end(string& s, std::string_view suffix) {
  if (!s.ends_with(suffix)) return false;
  s.erase(s.size() - suffix.size());
  return true;
}

// Replace all instances of substring with the replacement substring.
[[nodiscard]] static inline string replace_all(std::string_view str, std::string_view substring,
                                               std::string_view sreplacement) {
  string result;
  std::string_view::size_type i = 0;
  for (;;) {
    const auto j = str.find(substring, i);
    result += str.substr(i, j - i);
    if (j == std::string_view::npos) break;
    result += sreplacement;
    i = j + substring.size();
  }
  return result;
}

// Convert a string to lowercase.
[[nodiscard]] inline string to_lower(string s) {
  // for (char& ch : s) if (std::isupper(ch)) ch += 'a' - 'A';
  std::use_facet<std::ctype<char>>(std::locale()).tolower(s.data(), s.data() + s.size());
  return s;
}

// Convert a string to uppercase.
[[nodiscard]] inline string to_upper(string s) {
  // for (char& ch : s) if (std::islower(ch)) ch += 'A' - 'a';
  std::use_facet<std::ctype<char>>(std::locale()).toupper(s.data(), s.data() + s.size());
  return s;
}

// Returns the directory of a file path, like csh "$file:h" or bash "${file%/*}".
[[nodiscard]] inline string get_path_head(const string& s) {
  auto i = s.find_last_of("/\\");
  return i == string::npos ? s : s.substr(0, i);
}

// Returns the local filename of a file path, like csh "$file:t" or bash "${file##*/}".
[[nodiscard]] inline string get_path_tail(const string& s) {
  auto i = s.find_last_of("/\\");
  return i == string::npos ? s : s.substr(i + 1);
}

// Returns the root name of a file path, like csh "$file:r" or bash "${file%.*}".
[[nodiscard]] inline string get_path_root(const string& s) {
  auto i = s.rfind('.');
  return i == string::npos ? s : s.substr(0, i);
}

// Returns the file extension of a file path, without the period, like csh "$file:e" or bash "${file##*.}", except
// that it returns "" if the path has no period.
[[nodiscard]] inline string get_path_extension(const string& s) {
  auto i = s.rfind('.');
  return i == string::npos ? "" : s.substr(i + 1);
}

// Change directory separator characters '\\' to '/'.
[[nodiscard]] inline string get_canonical_path(const string& s) {
  string s2 = replace_all(s, "\\", "/");
  if (s2[0] && s2[1] == ':' && s2[2] == '/' && s2[0] >= 'A' && s2[0] <= 'Z') s2[0] += 'a' - 'A';
  return s2;
}

[[nodiscard]] inline bool is_path_absolute(const string& s) {
  assertx(!s.empty());
  return ((s[0] == '/' || s[0] == '\\') || (((s[0] >= 'a' && s[0] <= 'z') || (s[0] >= 'A' && s[0] <= 'Z')) &&
                                            s[1] == ':' && (s[2] == '/' || s[2] == '\\')));
}

// Convert a relative path to an absolute one if not already.
[[nodiscard]] inline string get_path_absolute(const string& s) {
  if (is_path_absolute(s)) return s;
  return get_current_directory() + "/" + s;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_STRINGOP_H_
