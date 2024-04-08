// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Audio.h"

#include "libHh/StringOp.h"  // get_path_extension()

namespace hh {

Audio Audio::read(const string& filename) {
  Audio audio;
  audio.read_file(filename);
  return audio;
}

string Audio::diagnostic_string() const {
  string s = sform("nsamples=%d nchannels=%d (%g Hz)", nsamples(), nchannels(), attrib().samplerate);
  if (attrib().suffix != "") s += " (" + attrib().suffix + ")";
  int brate = attrib().bitrate;
  if (brate)
    s += (brate > 1000000 ? sform(" (%.2fMi bps)", brate / 1000000.f)
          : brate > 1000  ? sform(" (%.2fKi bps)", brate / 1000.f)
                          : sform(" (%d bps)", brate));
  return s;
}

bool filename_is_audio(const string& filename) {
  static const auto& k_extensions = *new Array<string>{
      "wav", "mp3",
      // "pcm",
  };
  return k_extensions.index(to_lower(get_path_extension(filename))) >= 0;
}

string audio_suffix_for_magic_byte(uchar c) {
  // see also image_suffix_for_magic_byte() and video_suffix_for_magic_byte()
  // Documentation on prefixes for various image containers:
  // *.wav: "RIFF"
  // *.mp3: "ID3\003\000", "\377\373\220D"   (ID3 is a metadata container often used in conjunction with MP3)
  switch (c) {
      // u'\xFF' for "mp3" would be ambiguous with "jpg"
    case 'R': return "wav";
    case 'I': return "mp3";
    default: return "";
  }
}

}  // namespace hh
