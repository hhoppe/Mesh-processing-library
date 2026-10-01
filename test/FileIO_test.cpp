// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/FileIO.h"

#include <filesystem>

#include "libHh/Image.h"
#include "libHh/RangeOp.h"  // reverse(), sort()
using namespace hh;

namespace {

string read_all(const string& filename) {
  RFile fi(filename);
  std::ostringstream oss;
  oss << fi().rdbuf();
  return oss.str();
}

}  // namespace

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
  {
    // Classification of filenames.
    struct Case {
      string name;
      bool is_pipe, is_url, requires_pipe;
    };
    const Case cases[] = {
        {"file.txt", false, false, false},    {"-", false, false, true},
        {"file.gz", false, false, true},      {"file.Z", false, false, true},
        {"file.z", false, false, false},      {"file.gz.txt", false, false, false},
        {"echo a |", true, false, true},      {"| cat", true, false, true},
        {"https://a.b/c", false, true, true}, {"http://a.b/c", false, true, true},
        {"ftp://a.b/c", false, false, false}, {"xhttps://a.b", false, false, false},
    };
    for (const Case& c : cases) {
      if (is_pipe(c.name) != c.is_pipe || is_url(c.name) != c.is_url || file_requires_pipe(c.name) != c.requires_pipe)
        assertnever(SSHOW(c.name));
    }
  }
  {
    // Write and read back a plain file, including its modification time.
    const string filename = "FileIO_test.tmp.txt";
    assertx(!file_exists(filename));
    assertx(get_path_modification_time(filename) == 0);
    {
      WFile fo(filename);
      fo() << "line1\n\nline 3\n";
    }
    assertx(file_exists(filename) && !directory_exists(filename));
    assertx(read_all(filename) == "line1\n\nline 3\n");
    {
      RFile fi(filename);
      string line;
      for (const string expected : {"line1", "", "line 3"}) assertx(my_getline(fi(), line) && line == expected);
      assert_reached_eof(fi());
    }
    const uint64_t time = 1'000'000'000;  // 2001-09-09.
    assertx(set_path_modification_time(filename, time));
    assertx(get_path_modification_time(filename) == time);
    assertx(remove_file(filename));
    assertx(!file_exists(filename));
    assertx(!remove_file(filename));
    assertx(!set_path_modification_time(filename, time));
  }
  {
    // Write and read back a compressed file, either explicitly or by the name without its ".gz" suffix.
    const string filename = "FileIO_test.tmp";
    {
      WFile fo(filename + ".gz");
      for_int(i, 1000) fo() << i << '\n';
    }
    assertx(file_exists(filename + ".gz") && !file_exists(filename));
    string expected;
    for_int(i, 1000) expected += std::to_string(i) + '\n';
    assertx(read_all(filename + ".gz") == expected);
    assertx(read_all(filename) == expected);
    assertx(remove_file(filename + ".gz"));
  }
  {
    // Write to and read from pipe commands.
    const string filename = "FileIO_test.tmp.pipe";
    {
      WFile fo("| cat >" + filename);
      fo() << "piped\n";
    }
    assertx(read_all(filename) == "piped\n");
    assertx(read_all("cat " + filename + " |") == "piped\n");
    assertx(remove_file(filename));
  }
  {
    // Failure to open a file throws an exception.
    const auto throws = [](auto func) {
      try {
        func();
      } catch (const std::runtime_error&) {
        return true;
      }
      return false;
    };
    assertx(throws([] { RFile fi("FileIO_test.nonexistent"); }));
    assertx(throws([] { WFile fo("FileIO_test.nonexistent_dir/file.txt"); }));
  }
  {
    // A temporary file is deleted when going out of scope.
    string filename1, filename2;
    {
      std::istringstream iss("contents\n");
      const TmpFile tmp_file1("txt", iss);
      const TmpFile tmp_file2;
      filename1 = tmp_file1.filename(), filename2 = tmp_file2.filename();
      assertx(filename1 != filename2 && filename1.ends_with(".txt") && filename2.starts_with("TmpFile."));
      assertx(file_exists(filename1) && !file_exists(filename2));
      std::ostringstream oss;
      tmp_file1.write_to(oss);
      assertx(oss.str() == "contents\n");
      WFile fo(filename2);  // TmpFile requires that the file exist upon its destruction.
    }
    assertx(!file_exists(filename1) && !file_exists(filename2));
  }
  {
    // Listing the files and subdirectories of a directory.
    const string dir = "FileIO_test.tmp.dir";
    assertx(std::filesystem::create_directories(dir + "/sub1"));
    assertx(std::filesystem::create_directory(dir + "/sub2"));
    {
      WFile fo(dir + "/file1.txt");
    }
    {
      WFile fo(dir + "/file2");
    }
    assertx(directory_exists(dir) && !file_exists(dir) && directory_exists(dir + "/sub1"));
    SHOW(sort(get_files_in_directory(dir)));
    SHOW(sort(get_directories_in_directory(dir)));
    assertx(get_files_in_directory(dir + "/sub1").num() == 0);
    assertx(std::filesystem::remove_all(dir) == 5u);
    assertx(!directory_exists(dir));
  }
  {
    // Quoting of shell arguments.
    SHOW(quote_arg_for_sh("simple-name_1.txt"), quote_arg_for_shell("simple-name_1.txt"));
    SHOW(quote_arg_for_sh("a b'c"), quote_arg_for_shell("a b'c"));
    SHOW(quote_arg_for_sh("$x;y&"));
  }
  {
    // Exit codes of spawned commands.
    assertx(command_exists_in_path("sh") && !command_exists_in_path("FileIO_test_nonexistent_command"));
    SHOW(my_sh("exit 3"));
    SHOW(my_sh(V<string>("sh", "-c", "exit 4")));
    SHOW(my_spawn(V<string>("sh", "-c", "exit 5"), true));
    // Arguments are passed literally, despite special characters.
    const string filename = "FileIO_test.tmp.args";
    const string arg = "a  b*'\"$x;|";
    assertx(my_sh(V<string>("sh", "-c", "echo \"$1\" >" + filename, "unused_arg0", arg)) == 0);
    assertx(read_all(filename) == arg + "\n");
    assertx(remove_file(filename));
  }
  {
    // The null stream discards all output.
    cnull << "discarded" << 123 << std::flush;
    assertx(cnull);
  }
}
