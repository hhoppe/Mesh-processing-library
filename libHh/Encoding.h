// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ENCODING_H_
#define MESH_PROCESSING_LIBHH_ENCODING_H_

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/Map.h"
#include "libHh/Pqueue.h"
#include "libHh/RangeOp.h"  // sort()

namespace hh {

// Keep an ordered list of elements, and move any referenced element to the front.
template <typename T> class MoveToFront : noncopyable {
 public:
  int enter(const T& e) {
    int ifound = _list.index(e);
    if (ifound < 0) {
      ifound = _list.num();
      _list.push(e);
    }
    //  (An alternative faster implementation would be to randomly swap with another element closer to front,
    //   or to rotate k elements instead of the whole prefix set of elements.)
    rotate(_list, _list[ifound]);  // Rotate to place element [ifound] to at element [0].
    return ifound;
  }

 private:
  Array<T> _list;
};

// Record a set of events T with float probabilities.  Then, can compute entropy, Huffman cost, etc.
template <typename T> class Encoding : noncopyable {
 public:
  Encoding() {
    const bool movetofront = getenv_bool("MOVE_TO_FRONT");
    assertx(!movetofront);
    // No longer supported because it would only work if _map was Map<int, float>, i.e. for T == int.
    // Instead, we separate MovetoFront into its own class.
  }

  void add(const T& e, float prob) {
    // if (_movetofront) i = _movetofront->enter(e);
    bool is_new;
    float& oprob = _map.enter(e, prob, is_new);
    if (!is_new) oprob += prob;
  }

  void print() const {
    struct P {
      float prob;
      string str;
    };
    Array<P> ar;
    ar.reserve(_map.num());
    for (auto& [e, prob] : _map) ar.push(P{prob, make_string(e)});
    sort(ar, [](const P& p1, const P& p2) { return p1.prob < p2.prob || (p1.prob == p2.prob && p1.str < p2.str); });
    showdf("Encoding {\n");
    for (const P& p : ar) showdf(" %s %g\n", p.str.c_str(), p.prob);
    showdf("}\n");
  }

  // Return sum of prob to encode binary tree.
  float huffman_cost() const {
    Pqueue<int> pq;
    pq.reserve(_map.num());
    for (float prob : _map.values()) pq.enter_unsorted(0, prob);
    pq.sort();
    assertw(pq.num() >= 2);
    float sum = 0.f;
    for (;;) {
      if (pq.num() < 2) break;
      float prob1 = pq.min_priority();
      pq.remove_min();
      float prob2 = pq.min_priority();
      pq.remove_min();
      float prob = prob1 + prob2;
      pq.enter(0, prob);
      sum += prob;
    }
    return sum;
  }

  float entropy() const {
    double tot_prob = sum(_map.values());
    if (!assertw(tot_prob)) tot_prob = 1.;
    if (_map.num() <= 1) return 0.f;
    double sum = 0.;
    for (float prob : _map.values()) sum += prob * (-std::log2(prob / tot_prob));
    return float(sum);
  }

  // Return normalized entropy (entropy() / tot_prob).
  float norm_entropy() const {
    double tot_prob = sum(_map.values());
    if (!assertw(tot_prob)) tot_prob = 1.;
    if (_map.num() <= 1) return 0.f;
    double sum = 0.;
    for (float prob : _map.values()) sum += prob * (-std::log2(prob / tot_prob));
    return float(sum / tot_prob);
  }

  // include probability table
  float worst_entropy() const {
    assertnever("not implemented");
    // Sturling's approximation: n! =~ sqrt(TAU * n) * n^n * e^-n
    //   log_2(n!) =~ (0.5 * log(TAU) + (n + 0.5) * log(n) - n) / log(2)
    // For arithmetic coding with large probability distributions, the distribution itself is very predictable.
    // What requires encoding is the permutation of the symbols in the sorted decreasing list of symbols.
    // This permutation requires approximately m * log(m) bits if there are m  symbols.
    // Note: Unix has: double lgamma(double), which implements log(n!).
    // Include coding of probability table.
  }

  template <typename Func = string(const T&)>
  void print_top_entries(const string& name, int ntop, Func cb_entry_name) const {
    float tot_prob = sum<float>(_map.values());
    showdf("Encoding: %s (nunique=%d, tot_prob=%g) {\n", name.c_str(), _map.num(), tot_prob);
    if (0) {
      Pqueue<T> pq;
      pq.reserve(_map.num());
      float max_prob = 0.f;
      for (float prob : _map.values()) max_prob = max(max_prob, prob);
      for (auto& [e, prob] : _map) pq.enter_unsorted(e, max_prob - prob);
      pq.sort();
      if (!assertw(tot_prob)) tot_prob = 1.f;
      float cumu_prob = 0.f;
      for_int(i, ntop) {
        if (!pq.num()) break;
        float prob = max_prob - pq.min_priority();
        cumu_prob += prob;
        T e = pq.remove_min();
        string ename = cb_entry_name(e);
        showdf("  %20s  %.5f  cumu %.5f\n", ename.c_str(), prob / tot_prob, cumu_prob / tot_prob);
      }
    } else {
      struct P {
        float prob;
        string str;
      };
      Array<P> ar;
      ar.reserve(_map.num());
      for (auto& [e, prob] : _map) ar.push(P{prob, cb_entry_name(e)});
      // Sort in descending order by prob, then if equal, ascending by name.
      sort(ar, [](const P& p1, const P& p2) { return p1.prob > p2.prob || (p1.prob == p2.prob && p1.str < p2.str); });
      float cumu_prob = 0.f;
      for_int(i, min(ntop, ar.num())) {
        const P& p = ar[i];
        float prob = p.prob;
        string ename = p.str;
        cumu_prob += prob;
        showdf("  %20s  %.5f  cumu %.5f\n", ename.c_str(), prob / tot_prob, cumu_prob / tot_prob);
      }
    }
    showdf("} normalized_entropy=%f\n", entropy() / tot_prob);
    // showdf("} normalized_entropy=%f  (worst=%.2f)\n", entropy() / tot_prob, worst_entropy() / tot_prob);
  }

 private:
  Map<T, float> _map;
};

// Record a set of quantized delta values and report the resulting entropy.
// Also supports a set of float.
class DeltaEncoding {
 public:
  // Enter some number of data bits.
  void enter_bits(int nbits) {
    _num++;
    _enc_nbits.add(nbits, 1.f);
    _delta_bits += nbits;
  }

  // Enter some sign value (0 or 1), which is predicted from prior sign.
  void enter_sign(int vsign) {
    assertx(vsign == 0 || vsign == 1);
    _enc_sign[_prev_sign].add(vsign, 1.f);
    _prev_sign = vsign;
  }

  // Return total bits based on arithmetic coding of nbits and sign.
  int analyze(const string& s) const {
    int total_bits = int(ceil(total_entropy()));
    float enc_signs = _enc_sign[0].entropy() + _enc_sign[1].entropy();
    if (!s.empty())
      showdf("DE %s: n=%d  nbits=%.1f  sign=%.1f  bdelta=%.1f  total=%.1f  (%d)\n",  //
             s.c_str(), _num, _enc_nbits.entropy() / _num, enc_signs / _num, float(_delta_bits) / _num,
             float(total_bits) / _num, total_bits);
    return total_bits;
  }

  static int val_bits(float v) {
    float a = abs(v);
    if (a < 1.f) return 0;
    // int vbits = int(ceil(std::log2(a)));  // Old method.
    int vbits = int(floor(std::log2(a + 1.0001f)));  // New method 1997-06-09.
    // v : 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20
    // vb: -  1  1  2  2  2  2  3  3  3  3  3  3  3  3  4  4  4  4  4  4
    return vbits;
  }

  static int val_sign(float v) { return v >= 0.f; }
  // Enter some number of floating-point values.
  void enter_coords(CArrayView<float> ar) {
    for_int(c, ar.num()) {
      int nbits = val_bits(ar[c]);
      enter_bits(nbits);
      if (nbits) enter_sign(val_sign(ar[c]));
    }
  }

  void enter_vector(CArrayView<float> ar) {
    const int n = ar.num();
    static const int g_nbits_together = getenv_int("DELTA_ENCODING_NBITS_TOGETHER");
    if (g_nbits_together) {
      Warning("nbits_together tends to be a bad idea: poor encoding");
      int code = 0;
      assertx(n < narrow_cast<int>(sizeof(int) * 8 / 5));
      for_int(c, n) {
        int nbits = val_bits(ar[c]);
        _delta_bits += nbits;
        if (nbits) enter_sign(val_sign(ar[c]));
        assertx(nbits < 32);
        code = code * 32 + nbits;
      }
      _num++;
      _enc_nbits.add(code, 1.f);
      return;
    }
    int max_nbits = 0;
    for_int(c, n) {
      int nb = val_bits(ar[c]);
      if (nb > max_nbits) max_nbits = nb;
    }
    enter_bits(max_nbits * n);
    if (max_nbits) {
      for_int(c, n) enter_sign(val_sign(ar[c]));
    }
  }

  float total_entropy() const {
    return _delta_bits + _enc_nbits.entropy() + _enc_sign[0].entropy() + _enc_sign[1].entropy();
  }

 private:
  int _num{0};
  int _delta_bits{0};
  Encoding<int> _enc_nbits;
  int _prev_sign{0};
  Vec2<Encoding<int>> _enc_sign;  // Based on the previous sign.
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ENCODING_H_
