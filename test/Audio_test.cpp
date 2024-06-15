// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Audio.h"

#include "libHh/FileIO.h"
#include "libHh/GridOp.h"  // crop()
#include "libHh/Stat.h"
using namespace hh;

int main() {
  if (0) my_setenv("AUDIO_DEBUG", "1");
  if (1) my_setenv("AUDIO_TEST_CODEC", "1");  // avoid dependency on external ffmpeg program
  if (1) {
    // 400 Hz tone for 3 s at 48 kHz sampling in stereo
    const double freq = 400., duration = 3., samplerate = 48'000.;
    const int nchannels = 2;
    const int nsamples = int(duration * samplerate + .5);
    Audio audio1(V(nchannels, nsamples));
    audio1.attrib().samplerate = samplerate;
    audio1.attrib().bitrate = 256'000;  // 256 kbps
    for_int(i, audio1.nsamples()) for_int(ch, audio1.nchannels()) {
      double t = i / samplerate;  // time in seconds
      float v;
      if (1) {
        v = std::sin(float(t * freq * TAU));  // this one compresses well using *.mp3
      } else if (0) {
        double mod_freq = 5.;  // add a modulation frequency of 5 Hz
        double freq2 = freq * (1. + .3 * std::sin(t * mod_freq * TAU));
        v = std::sin(float(t * freq2 * TAU));
      } else {
        double mod_freq = 5.;  // add a modulation frequency of 5 Hz
        double t2 = t + .5 * (1. / mod_freq) * pow(std::sin(t * mod_freq * TAU), .5);
        v = std::sin(float(t2 * freq * TAU));
      }
      audio1(ch, i) = v;
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
      HH_RSTAT(Schan_diff, audio2[1] - audio2[0]);
      assertx(audio2.nsamples() >= audio1.nsamples());
      Audio audio2c(audio2);
      audio2c = crop(audio2c, twice(0), audio2.dims() - audio1.dims());
      HH_RSTAT(Senc_diff, audio2c - audio1);
      const float thresh = audio2.attrib().suffix == "wav" ? 1e-7f : 1e-2f;
      SHOW(audio1[0].head(5));
      SHOW(audio2[1].head(5));
      for_int(i, audio2.nsamples()) for_int(ch, audio2.nchannels()) {
        if (i >= audio1.nsamples()) continue;
        float diff = audio2(ch, i) - audio1(ch, i);
        if (abs(diff) > thresh) assertnever(SSHOW(i, diff));
      }
    }
    assertx(remove_file("Audio_test.wav"));
  }
}
