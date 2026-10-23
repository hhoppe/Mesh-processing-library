// -*- C++ -*-
#include "libHh/MappedFile.h"

#include "libHh/FileIO.h"
using namespace hh;

namespace {

// Writes the bytes to a file (compressed if its name ends in ".gz"), and verifies that MappedFile reads them back.
void test_contents(const string& filename, const string& contents) {
  {
    WFile fi(filename);
    fi().write(contents.data(), contents.size());
  }
  {
    const MappedFile mapped_file(filename);
    const std::string_view view = mapped_file.view();
    SHOW(filename, view.size());
    assertx(view == contents);
  }
  assertx(remove_file(filename));
}

}  // namespace

int main() {
  test_contents("MappedFile_test.empty.tmp", "");
  {  // The contents may be binary, including '\0' bytes and no terminating newline.
    string binary;
    for_int(i, 256) binary.push_back(char(i));
    binary += string(3, '\0');
    test_contents("MappedFile_test.binary.tmp", binary);
    // A compressed file is read through a pipe rather than mapped.
    test_contents("MappedFile_test.binary.tmp.gz", binary);
  }
  {  // A larger file spans many pages.
    string text;
    for_int(i, 200'000) text += "Vertex " + std::to_string(i + 1) + "  0 1 2\n";
    test_contents("MappedFile_test.text.tmp", text);
  }
}
