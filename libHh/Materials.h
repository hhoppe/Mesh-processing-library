// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MATERIALS_H_
#define MESH_PROCESSING_LIBHH_MATERIALS_H_

#include "libHh/Array.h"
#include "libHh/StringOp.h"

namespace hh {

// An array of material strings, indexed by an integer material identifier (matid).
// Some strings may be null "".
// Strings are formatted like GMesh strings.
//  usually contain 'matid=%d'.
//  may contain 'simatid=%d'     (Softimage material identifier)
//  may contain 'mat="%s"'       (Material name)
//  may contain 'rgb=(%g %g %g)' (Diffuse color of material)
//  may contain 'groups="%s"'    (the model to which this face belongs)
class Materials {
 public:
  void read(std::istream& is) {  // must be empty
    assertx(!_matstrings.num());
    int nmaterials;
    {
      string line;
      assertx(my_getline(is, line));
      assertx(remove_at_start(line, "nmaterials="));
      nmaterials = to_int(line);
    }
    _matstrings.init(nmaterials);
    for_int(matid, nmaterials) assertx(my_getline(is, _matstrings[matid]));
  }
  void write(std::ostream& os) const {
    os << "nmaterials=" << _matstrings.num() << '\n';
    for_int(matid, _matstrings.num()) os << _matstrings[matid] << '\n';
  }
  void set(int matid, string matstring) {
    _matstrings.access(matid);
    _matstrings[matid] = std::move(matstring);
  }
  const string& get(int matid) const {
    assertx(_matstrings.ok(matid));
    assertx(_matstrings[matid] != "");
    return _matstrings[matid];
  }
  int num() const { return _matstrings.num(); }
  size_t size() const { return _matstrings.size(); }
  bool ok(int i) const { return _matstrings.ok(i); }

 private:
  Array<string> _matstrings;
  // Default operator=() and copy_constructor are safe.
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MATERIALS_H_
