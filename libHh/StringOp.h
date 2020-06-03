// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STRINGOP_H_
#define MESH_PROCESSING_LIBHH_STRINGOP_H_

#include "Hh.h"

namespace hh {

// Does string contain a character? (subsumed by general contains() function in RangeOp.h)
// inline bool contains(const string& str, char ch) {
//     return str.find(ch) != string::npos;
// }

// Does string contain a substring?
inline bool contains(const string& str, const string& substr) { return str.find(substr) != string::npos; }

// Does string have the specified prefix string?
inline bool begins_with(const string& s, const string& se) {
  assertx(!se.empty());
  return !s.compare(0, se.size(), se);
}

// Does string have the specified suffix string?
inline bool ends_with(const string& s, const string& se) {
  if (s.size() < se.size()) return false;
  if (s.compare(s.size() - se.size(), se.size(), se)) return false;
  return true;
}

// If se is not at beg of s, return false; else erase se from s and return true.
inline bool remove_at_beginning(string& s, const string& se) {
  if (!begins_with(s, se)) return false;
  s.erase(0, se.size());
  return true;
}

// If se is not at end of s, return false; else erase se from s and return true.
inline bool remove_at_end(string& s, const string& se) {
  if (!ends_with(s, se)) return false;
  s.erase(s.size() - se.size());
  return true;
}

// Replace all pattern substrings with specified replacement substring.
static inline string replace_all(const string& str, const string& spattern, const string& sreplacement) {
  string sres;
  string::size_type i = 0;
  for (;;) {
    auto j = str.find(spattern, i);
    sres += str.substr(i, j - i);
    if (j == string::npos) break;
    sres += sreplacement;
    i = j + spattern.size();
  }
  return sres;
}

// Convert a string to lowercase.
inline string to_lower(string s) {
  // for (char& ch : s) if (std::isupper(ch)) ch += 'a' - 'A';
  std::use_facet<std::ctype<char>>(std::locale()).tolower(&s[0], &s[0] + s.size());  // fails with s.data()
  return s;
}

// Convert a string to uppercase.
inline string to_upper(string s) {
  // for (char& ch : s) if (std::islower(ch)) ch += 'A' - 'a';
  std::use_facet<std::ctype<char>>(std::locale()).toupper(&s[0], &s[0] + s.size());  // fails with s.data()
  return s;
}

// Return the directory of file path, like csh $file:h .
inline string get_path_head(const string& s) {
  auto i = s.find_last_of("/\\");
  return i == string::npos ? s : s.substr(0, i);
}

// Return the local filename of file path, like csh $file:t .
inline string get_path_tail(const string& s) {
  auto i = s.find_last_of("/\\");
  return i == string::npos ? s : s.substr(i + 1);
}

// Return the root name of file path, like csh $file:r .
inline string get_path_root(const string& s) {
  auto i = s.rfind(".");
  return i == string::npos ? s : s.substr(0, i);
}

// Return the file extension of file path, like csh $file:e .
inline string get_path_extension(const string& s) {
  auto i = s.rfind(".");
  return i == string::npos ? "" : s.substr(i + 1);
}

// Change directory separator characters '\\' to '/'.
inline string get_canonical_path(const string& s) {
  string s2 = replace_all(s, "\\", "/");
  if (s2[0] && s2[1] == ':' && s2[2] == '/' && s2[0] >= 'A' && s2[0] <= 'Z') s2[0] += 'a' - 'A';
  return s2;
}

inline bool is_path_absolute(const string& s) {
  assertx(!s.empty());
  return ((s[0] == '/' || s[0] == '\\') || (((s[0] >= 'a' && s[0] <= 'z') || (s[0] >= 'A' && s[0] <= 'Z')) &&
                                            s[1] == ':' && (s[2] == '/' || s[2] == '\\')));
}

// Convert a relative path to an absolute one if not already.
inline string get_path_absolute(const string& s) {
  if (is_path_absolute(s)) return s;
  return get_current_directory() + "/" + s;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_STRINGOP_H_
