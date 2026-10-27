// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "libHh/Histogram.h"

using namespace hh;

namespace {

const string k_filename = "Histogram_test.tmp.txt";

// Show the histogram file written by the Histogram destructor (optionally omitting the bucket centers), and return
// the sum of its bucket counts.
int show_histogram_file(bool show_centers = true) {
  int sum = 0;
  {
    RFile fi(k_filename);
    string line;
    while (my_getline(fi(), line)) {
      float center;
      int count;
      assertx(sscanf(line.c_str(), "%g %d", &center, &count) == 2);
      if (show_centers) {
        showf("  %s\n", line.c_str());
      } else {
        showf("  %d\n", count);
      }
      sum += count;
    }
  }
  assertx(remove_file(k_filename));
  return sum;
}

}  // namespace

int main() {
  {
    // Integer values 0..16 in 4 buckets; the range is widened by 1.5 buckets on each side to [-6, 22].
    {
      Histogram hist(k_filename, 4);
      for_int(i, 17) hist.add(float(i));
    }
    SHOW(show_histogram_file());
  }
  {
    // Values added before clear() are discarded.
    {
      Histogram hist(k_filename, 4);
      for_int(i, 10) hist.add(1000.f);
      hist.clear();
      for (const float f : {1.f, 2.f, 2.f, 3.f, 3.f, 3.f, 4.f}) hist.add(f);
    }
    SHOW(show_histogram_file());
  }
  {
    // An outlier beyond 3 standard deviations narrows the range (with a warning) and falls into the last bucket.
    {
      Histogram hist(k_filename, 5);
      for_int(i, 40) hist.add(float(i % 4));
      hist.add(1000.f);
    }
    assertx(show_histogram_file(false) == 41);
  }
  {
    // An empty filename disables the output.
    {
      Histogram hist("", 5);
      hist.add(1.f);
    }
    assertx(!file_exists(k_filename));
  }
}
