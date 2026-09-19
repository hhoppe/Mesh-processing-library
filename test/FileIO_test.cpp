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
      const TmpFile tmp_file("png", fi());
      filename = tmp_file.filename();
      assertx(file_exists(filename));
      const Image image{filename};
      assertx(image.dims() == V(128, 128));
    }
    assertx(!file_exists(filename));
  }
  {
    const string url = "https://github.com/hhoppe/data/raw/main/image.png";
    const TmpFile tmp_file("png", RFile{url}());
    const Image image{tmp_file.filename()};
    assertx(image.dims() == V(128, 128));
  }
  {
    RFile fi("echo a b |");
    string line;
    assertx(my_getline(fi(), line, true));
    assertx(line == "a b");
  }
  {
    const auto verify_reversed_lines = [](const string& content) {
      Array<string> expected;  // The lines of content, in reverse order.
      for (size_t i = 0; i < content.size();) {
        const size_t j = min(content.find('\n', i), content.size());
        string line = content.substr(i, j - i);
        if (line.ends_with('\r')) line.pop_back();
        expected.push(std::move(line));
        i = j + 1;
      }
      reverse(expected);
      const TmpFile tmp_file("txt");
      {
        WFile fo(tmp_file.filename());
        fo() << content;
      }
      ReversedLinesReader reader(tmp_file.filename());
      int i = 0;
      for (string line; reader.getline(line);) assertx(i < expected.num() && line == expected[i++]);
      assertx(i == expected.num());
    };
    for (const string& content : V<string>("", "\n", "\n\n", "a", "a\n", "a\nbc", "a\n\nbc\n", "a\r\nbc\r\n", "\r\n"))
      verify_reversed_lines(content);
    string content;  // Spans several 1 MiB chunks and includes a line that is longer than a chunk.
    for (const int i : range(100'000)) content += string(i % 37, char('a' + i % 26)) + '\n';
    content += string(1'500'000, 'x') + '\n';
    for (const int i : range(50'000)) content += string(i % 11, char('A' + i % 26)) + '\n';
    verify_reversed_lines(content);
  }
}
