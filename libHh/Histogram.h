// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HISTOGRAM_H_
#define MESH_PROCESSING_LIBHH_HISTOGRAM_H_

#include "Stat.h"
#include "Array.h"
#include "FileIO.h"
#include "RangeOp.h"

namespace hh {

// Accumulate a set of values, and upon destruction, write a histogram of these value into a text file.
class Histogram : noncopyable {
 public:
    explicit Histogram(string filename, int nbuckets) : _filename(std::move(filename)), _nbuckets(nbuckets) { }
    void clear()                                { _ar_val.clear(); }
    void add(float val)                         { _ar_val.push(val); }
    ~Histogram() {
        if (_filename=="") return;
        Stat stat(_ar_val);
        assertx(_nbuckets>0);
        float min = stat.min(), max = stat.max(), avg = stat.avg(), sdv = stat.sdv();
        assertw(sdv>0.f);
        const float max_sdv = 3.f;
        {
            float range1 = max-min;
            // Go slightly beyond min and max.
            min -= range1/_nbuckets*1.5f;
            max += range1/_nbuckets*1.5f;
        }
        if (!assertw(min>avg-sdv*max_sdv)) min = avg-sdv*max_sdv;
        if (!assertw(max<avg+sdv*max_sdv)) max = avg+sdv*max_sdv;
        float bucket_size = (max-min)/_nbuckets;
        if (!assertw(bucket_size>0.f)) bucket_size = 1.f;
        float recip_bucket_size = 1.f/bucket_size;
        Array<int> buckets(_nbuckets, 0);
        for (float f : _ar_val) {
            int bi = clamp(static_cast<int>((f-min)*recip_bucket_size), 0, _nbuckets-1);
            buckets[bi]++;
        }
        showdf("Hist(%s): %s\n", _filename.c_str(), stat.short_string().c_str());
        WFile fi(_filename);
        for_int(bi, _nbuckets) {
            fi() << sform("%g %d\n", min+(bi+0.5f)*bucket_size, buckets[bi]);
        }
    }
 private:
    string _filename;
    int _nbuckets;
    Array<float> _ar_val;
};

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_HISTOGRAM_H_
