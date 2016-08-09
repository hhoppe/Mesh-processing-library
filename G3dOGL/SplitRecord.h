// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "SimplicialComplex.h"

namespace hh {

struct AreaData {
    int dim;
    int id;
    float area;
};

class SplitRecord {
 public:
    SplitRecord()                               { reset(); }
    SplitRecord(const SplitRecord&)             = delete;
    SplitRecord& operator=(const SplitRecord&)  = delete;
    SplitRecord(SplitRecord&&)                  = default;
    SplitRecord& operator=(SplitRecord&&)       = default;
    bool read(std::istream& is);
    void write(std::ostream& os) const;
    void reset();
    int getNextOutcome() { return _outcome[_outcome_index++]; }
    void applySplit(SimplicialComplex& K);

    // apply split and capture info for geomorphs
    void applyGMSplit(SimplicialComplex& K);

    // apply split and capture info for compression
    void applyCmpSplit(SimplicialComplex& K);

    void applyUnify(SimplicialComplex& K) const;

    // access
    const Array<AreaData>& getAreas();
    int getVs() const { return _vsid; }
    int getVt() const { return _vtid; }
    int vsp() const { return _pos_bit; }
    const Point& getDeltap() const { return _deltap; }

    // meaningful after applyGMSplit
    const std::vector<Simplex>& getNewFacets() const { return new_facets; }
    // meaningful after applyCmpSplit
    const std::vector<Simplex>& getNewSimplices() const { return new_simplices; }

    // Note that "Array<int> _outcome" below takes the union of the following enum values.
    enum { V_NOEDGE, V_EDGE };        // vertex outcomes
    enum { E_VS, E_VT, E_VSVT, E_F }; // edge outcomes
    enum { F_VS, F_VT, F_VSVT };      // face outcomes

    struct MaterialData {
        int dim;
        int id;
        int matid;
    };

 private:
    int _vsid;
    int _vtid;
    Array<int> _outcome;
    int _pos_bit;
    Point _deltap;
    Array<MaterialData> _material;
    Array<AreaData> _area;

    std::vector<Simplex> new_facets;
    std::vector<Simplex> new_simplices;

    // auxiliary variables
    int _outcome_index;
    int _material_index;
};

inline void SplitRecord::reset() {
    _outcome_index = 0;
    _material_index = 0;
}

inline const Array<AreaData>& SplitRecord::getAreas() {
    return _area;
}

} //namespace hh
