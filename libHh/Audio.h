// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_AUDIO_H_
#define MESH_PROCESSING_LIBHH_AUDIO_H_

#include "libHh/Grid.h"

#if 0
{
  // 400 Hz tone for 3 s at 48 kHz sampling in stereo
  const double freq = 400., duration = 3., samplerate = 48'000.;
  const int nchannels = 2;
  const int nsamples = int(duration * samplerate + .5);
  Audio audio(V(nchannels, nsamples));
  audio.attrib().samplerate = samplerate;
  for_int(i, audio.nsamples()) for_int(ch, audio.nchannels()) {
    float t = i / samplerate;  // time in seconds
    audio(ch, i) = std::sin(t * freq * TAU);
  }
  audio.attrib().bitrate = 256'000;  // 256 kbps
  audio.write_file("file.mp3");
  // Audio read/write is performed using ffmpeg.
}
#endif

namespace hh {

// Audio contains a sequence of audio samples for one or more channels; values should be in the range [-1.f, +1.f].
class Audio : public Grid<2, float> {
  // Corresponds to pcm_f32be or pcm_f32le depending on native byte ordering.
  using base = Grid<2, float>;
  friend void swap(Audio& l, Audio& r) noexcept;

 public:
  using value_type = float;
  struct Attrib;
  explicit Audio(const Vec2<int>& dims = V(0, 0)) { init(dims); }  // nchannels, nsamples
  explicit Audio(const Audio&) = default;
  explicit Audio(const string& filename) { read_file(filename); }
  Audio(Audio&& v) noexcept { swap(*this, v); }
  Audio(base&& v) noexcept { swap(implicit_cast<base&>(*this), v); }
  ~Audio() {}
  Audio& operator=(Audio&& v) noexcept {
    clear();
    swap(*this, v);
    return *this;
  }
  void operator=(base&& v) { clear(), swap(implicit_cast<base&>(*this), v); }
  Audio& operator=(const Audio&) = default;
  void operator=(CGridView<2, float> audio) { base::assign(audio); }
  void init(const Vec2<int>& dims) { base::init(dims); }
  void clear() { init(twice(0)); }
  int nchannels() const { return base::dim(0); }
  int nsamples() const { return base::dim(1); }
  const Attrib& attrib() const { return _attrib; }
  Attrib& attrib() { return _attrib; }
  void read_file(const string& filename);         // filename may be "-" for std::cin;  may throw std::runtime_error
  void write_file(const string& filename) const;  // filename may be "-" for std::cout; may throw std::runtime_error
  string diagnostic_string() const;

  // Misc:
  struct Attrib {
    string suffix;          // e.g. "wav"; "" if unknown; to identify format of read_file("-") and write_file("-")
    double samplerate{0.};  // samples / sec (Hz)
    int bitrate{0};         // bits / sec
  };

 private:
  Attrib _attrib;
};

// Whether filename suffix identifies it as audio.
bool filename_is_audio(const string& filename);

// Return predicted audio suffix given first byte of file, or "" if unrecognized.
string audio_suffix_for_magic_byte(uchar c);

//----------------------------------------------------------------------------

// Shared for implementation in Video.cpp
bool ffmpeg_command_exists();

inline void swap(Audio& l, Audio& r) noexcept {
  using std::swap;
  swap(implicit_cast<Audio::base&>(l), implicit_cast<Audio::base&>(r));
  swap(l._attrib, r._attrib);
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_AUDIO_H_
