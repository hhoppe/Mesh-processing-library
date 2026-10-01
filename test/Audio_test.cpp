// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Audio.h"

#include "libHh/FileIO.h"
#include "libHh/GridOp.h"  // crop()
#include "libHh/Stat.h"
using namespace hh;

int main() {
  if (0) my_setenv("AUDIO_DEBUG", "1");
  if (1) my_setenv("AUDIO_TEST_CODEC", "1");  // Avoid dependency on external ffmpeg program.
  if (1) {
    // 400 Hz tone for 3 s at 48 kHz sampling in stereo.
    const double freq = 400., duration = 3., samplerate = 48'000.;
    const int nchannels = 2;
    const int nsamples = int(duration * samplerate + .5);
    Audio audio1(V(nchannels, nsamples));
    audio1.attrib().samplerate = samplerate;
    audio1.attrib().bitrate = 256'000;  // 256 kbps
    for_int(i, audio1.nsamples()) for_int(ch, audio1.nchannels()) {
      const double t = i / samplerate;  // Time in seconds.
      float v;
      if (1) {
        v = std::sin(float(t * freq * TAU));  // This one compresses well using *.mp3.
      } else if (0) {
        const double mod_freq = 5.;  // Add a modulation frequency of 5 Hz.
        const double freq2 = freq * (1. + .3 * std::sin(t * mod_freq * TAU));
        v = std::sin(float(t * freq2 * TAU));
      } else {
        const double mod_freq = 5.;  // Add a modulation frequency of 5 Hz.
        const double t2 = t + .5 * (1. / mod_freq) * pow(std::sin(t * mod_freq * TAU), .5);
        v = std::sin(float(t2 * freq * TAU));
      }
      audio1[ch, i] = v;
    }
    SHOW(audio1.nsamples());
    if (1) {
      if (0) audio1.write_file("Audio_test.mp3");
      if (1) audio1.write_file("Audio_test.wav");
      SHOW(audio1.diagnostic_string());
    }
    if (1) {
      Audio audio2(0 ? "Audio_test.mp3" : "Audio_test.wav");
      SHOW(audio2.nsamples());
      SHOW(audio2.diagnostic_string());
      SHOW(audio2.dims());
      assertx(audio2.nchannels() == audio1.nchannels());
      assertx(audio2.attrib().samplerate == audio1.attrib().samplerate);
      HH_RSTAT(Schan_diff, audio2[1] - audio2[0]);
      // A lossy encoding (*.mp3) may introduce extra samples, whereas *.wav must preserve their number.
      assertx(audio2.nsamples() >= audio1.nsamples());
      if (audio2.attrib().suffix == "wav") assertx(audio2.nsamples() == audio1.nsamples());
      Audio audio2c(audio2);
      audio2c = crop(audio2c, twice(0), audio2.dims() - audio1.dims());
      HH_RSTAT(Senc_diff, audio2c - audio1);
      const float thresh = audio2.attrib().suffix == "wav" ? 0.f : 1e-2f;  // The *.wav float encoding is lossless.
      SHOW(audio1[0].head(5));
      SHOW(audio2[1].head(5));
      for_int(i, audio2.nsamples()) for_int(ch, audio2.nchannels()) {
        if (i >= audio1.nsamples()) continue;
        const float diff = audio2[ch, i] - audio1[ch, i];
        if (abs(diff) > thresh) assertnever(SSHOW(i, diff));
      }
    }
    assertx(remove_file("Audio_test.wav"));
  }
  {
    // A small audio with three channels and a non-default sampling rate also round-trips exactly.
    Audio audio(V(3, 10));
    audio.attrib().samplerate = 44'100.;
    audio.attrib().bitrate = 999;
    for_int(ch, 3) for_int(i, 10) audio[ch, i] = (ch + 1) * .1f * float(i - 5);
    audio.write_file("Audio_test2.wav");
    SHOW(audio.diagnostic_string());  // The written suffix is retained in the attributes.
    const Audio audio2("Audio_test2.wav");
    SHOW(audio2.diagnostic_string());
    assertx(audio2.dims() == audio.dims() && ranges::equal(audio2, audio));
    SHOW(audio2[2]);
    assertx(remove_file("Audio_test2.wav"));
  }
  {
    // Reading a missing file throws.
    bool threw = false;
    try {
      const Audio audio("Audio_test_nonexistent.wav");
    } catch (const std::runtime_error&) {
      threw = true;
    }
    assertx(threw);
  }
  {
    // The formatting of the diagnostic string.
    Audio audio(V(1, 5));
    SHOW(audio.diagnostic_string(), audio.nchannels(), audio.nsamples());
    audio.attrib().samplerate = 8'000.;
    audio.attrib().bitrate = 2'500'000;
    audio.attrib().suffix = "mp3";
    SHOW(audio.diagnostic_string());
    audio.attrib().bitrate = 500;
    SHOW(audio.diagnostic_string());
  }
  {
    // Construction, assignment, and swap.
    Audio audio1(V(2, 4));
    fill(audio1, .5f);
    audio1.attrib().samplerate = 100.;
    const Audio audio2(audio1);  // The copy includes the attributes.
    assertx(audio2.attrib().samplerate == 100. && ranges::equal(audio2, audio1));
    Audio audio3(std::move(audio1));
    assertx(audio3.dims() == V(2, 4) && audio1.size() == 0);  // NOLINT(bugprone-use-after-move)
    Grid<2, float> grid(V(1, 3), -.25f);
    Audio audio4(std::move(grid));  // Move of a grid transfers its allocation.
    assertx(audio4.nchannels() == 1 && audio4.nsamples() == 3 && audio4[0, 2] == -.25f);
    assertx(audio4.attrib().samplerate == 0.);
    audio4 = Grid<2, float>(V(2, 2), 1.f);
    assertx(audio4.dims() == V(2, 2));
    audio4 = CGridView<2, float>(Grid<2, float>(V(2, 2), 3.f));  // Elementwise assignment requires the same size.
    assertx(sum(audio4) == 12.f);
    swap(audio4, audio3);
    assertx(audio4.dims() == V(2, 4) && audio4.attrib().samplerate == 100. && audio3.dims() == V(2, 2));
    audio4.clear();
    assertx(audio4.size() == 0);
  }
  {
    // Filename and magic-byte recognition.
    for (const string filename : {"a.wav", "a.WAV", "a.mp3", "a.mp4", "a.pcm", "a"})
      SHOW(filename, filename_is_audio(filename));
    SHOW(audio_suffix_for_magic_byte('R'), audio_suffix_for_magic_byte('I'), audio_suffix_for_magic_byte(uchar{255}));
  }
}
