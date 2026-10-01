// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/StringOp.h"
using namespace hh;

int main() {
  {
    string s = "prefix.body.suffix";
    assertx(!remove_at_start(s, "body"));
    assertx(remove_at_start(s, "prefix."));
    SHOW(s);
    assertx(!remove_at_end(s, "body"));
    assertx(remove_at_end(s, ".suffix"));
    SHOW(s);
    assertx(remove_at_start(s, "") && remove_at_end(s, "") && s == "body");
    assertx(remove_at_start(s, "body") && s == "");
  }
  {
    SHOW(replace_all("a.b.c", ".", "::"));
    SHOW(replace_all("aaaa", "aa", "b"));  // Non-overlapping matches, from left to right.
    SHOW(replace_all("abc", "x", "y"));
    SHOW(replace_all("abc", "abc", ""));
    SHOW(replace_all("", "a", "b"));
    SHOW(replace_all("a\\b\\c", "\\", "/"));
  }
  {
    SHOW(to_lower("Hello, World! 123"));
    SHOW(to_upper("Hello, World! 123"));
    assertx(to_lower("") == "" && to_upper("") == "");
  }
  {
    for (const string s : {"dir/sub/file.ext", "file.ext", "dir\\file.tar.gz", "dir/file", "/file", "file"}) {
      SHOW(s, get_path_head(s), get_path_tail(s), get_path_root(s), get_path_extension(s));
    }
    // Only a period in the last path component starts an extension.
    for (const string s : {"dir.v2/file", "dir.v2/file.txt", "dir.v2\\file"}) {
      SHOW(s, get_path_root(s), get_path_extension(s));
    }
  }
  {
    SHOW(get_canonical_path("C:\\Users\\name\\file.txt"));
    SHOW(get_canonical_path("c:/already/canonical"));
    SHOW(get_canonical_path("relative\\path"));
    SHOW(get_canonical_path("Z:"));
    SHOW(get_canonical_path(""));
  }
  {
    for (const string s : {"/usr/bin", "\\server", "C:/dir", "c:\\dir", "dir/file", "file", "C:", "C:file"}) {
      SHOW(s, is_path_absolute(s));
    }
    assertx(get_path_absolute("/usr/bin") == "/usr/bin");
    const string s = get_path_absolute("some_file");
    assertx(is_path_absolute(s));
    assertx(s.ends_with("/some_file"));
  }
}
