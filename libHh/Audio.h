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

 public:
  using value_type = float;
  struct Attrib;
  explicit Audio(const Vec2<int>& dims = V(0, 0)) { init(dims); }  // nchannels, nsamples
  explicit Audio(const Audio&) = default;
  explicit Audio(const string& filename) { read_file(filename); }
  Audio(Audio&& v) noexcept { swap(*this, v); }
  Audio(base&& v) noexcept { swap(implicit_cast<base&>(*this), v); }
  Audio& operator=(Audio&& v) noexcept { return clear(), swap(*this, v), *this; }
  void operator=(base&& v) { clear(), swap(implicit_cast<base&>(*this), v); }
  Audio& operator=(const Audio&) = default;
  void operator=(CGridView<2, float> audio) { base::assign(audio); }
  void init(const Vec2<int>& dims) { base::init(dims); }
  void clear() { init(twice(0)); }
  [[nodiscard]] int nchannels() const { return base::dim(0); }
  [[nodiscard]] int nsamples() const { return base::dim(1); }
  [[nodiscard]] auto& attrib(this auto&& self) { return self._attrib; }
  void read_file(const string& filename);         // filename may be "-" for std::cin;  may throw std::runtime_error
  void write_file(const string& filename) const;  // filename may be "-" for std::cout; may throw std::runtime_error
  [[nodiscard]] string diagnostic_string() const;

  // Misc:
  struct Attrib {
    string suffix;          // e.g. "wav"; "" if unknown; to identify format of read_file("-") and write_file("-")
    double samplerate{0.};  // samples / sec (Hz)
    int bitrate{0};         // bits / sec
  };
  friend void swap(Audio& l, Audio& r) noexcept;

 private:
  Attrib _attrib;
};

// Whether filename suffix identifies it as audio.
[[nodiscard]] bool filename_is_audio(const string& filename);

// Return predicted audio suffix given first byte of file, or "" if unrecognized.
[[nodiscard]] string audio_suffix_for_magic_byte(uchar c);

//----------------------------------------------------------------------------

// Shared for implementation in Video.cpp
[[nodiscard]] bool ffmpeg_command_exists();

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_AUDIO_H_
