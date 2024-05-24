// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/FileIO.h"

#include "libHh/Image.h"
using namespace hh;

int main() {
  {
    const string url = "https://github.com/hhoppe/data/raw/main/LICENSE";
    RFile fi(url);
    string line;
    assertx(my_getline(fi(), line, true));
    assertx(line == "CC0 1.0 Universal");
    assertx(my_getline(fi(), line, true));
    assertx(line == "");
    assertx(my_getline(fi(), line, true));
    assertx(line == "Statement of Purpose");
  }
  {
    const string url = "https://github.com/hhoppe/data/raw/main/image.png";
    RFile fi(url);
    string filename;
    {
      TmpFile tmp_file("png", fi());
      filename = tmp_file.filename();
      assertx(file_exists(filename));
      Image image{filename};
      assertx(image.dims() == V(128, 128));
    }
    assertx(!file_exists(filename));
  }
  {
    const string url = "https://github.com/hhoppe/data/raw/main/image.png";
    TmpFile tmp_file("png", RFile{url}());
    Image image{tmp_file.filename()};
    assertx(image.dims() == V(128, 128));
  }
  {
    RFile fi("echo a b |");
    string line;
    assertx(my_getline(fi(), line, true));
    assertx(line == "a b");
  }
}
