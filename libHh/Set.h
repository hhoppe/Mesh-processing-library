// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SET_H_
#define MESH_PROCESSING_LIBHH_SET_H_

#include <unordered_set>
#include "Random.h"

#if 0
{
  Set<Edge> sete;
  for (Edge e : set) consider(e);
  //
  struct mypair {
    unsigned _v1, _v2;
  };
  struct hash_mypair {
    size_t operator()(const mypair& e) const { return hash_combine(my_hash(e._v1), e._v2); }
  };
  struct equal_mypair {
    bool operator()(const mypair& e1, const mypair& e2) const { return e1._v1 == e2._v1 && e1._v2 == e2._v2; }
  };
  Set<mypair, hash_mypair, equal_mypair> setpairs;
  //
  struct hash_edge {
    ... const GMesh& _mesh;
  }  // from MeshOp.cpp
  hash_edge he(mesh);
  Set<Edge, hash_edge> sete(he);
}
#endif

namespace hh {

// My wrapper around std::unordered_set<>.  (typename Equal also goes by name Pred in C++ standard library).
template<typename T, typename Hash = std::hash<T>, typename Equal = std::equal_to<T> > class Set {
    using type = Set<T, Hash, Equal>;
    using base = std::unordered_set<T, Hash, Equal>;
 public:
    using Hashf = typename base::hasher;
    using Equalf = typename base::key_equal;
    using value_type = T;
    using iterator = typename base::iterator;
    using const_iterator = typename base::const_iterator;
    Set()                                       = default;
    explicit Set(Hashf hashf)                   : _s(0, hashf) { }
    explicit Set(Hashf hashf, Equalf equalf)    : _s(0, hashf, equalf) { }
    void clear()                                { _s.clear(); }
    void enter(const T& e)                      { auto p = _s.insert(e); ASSERTX(p.second); } // e must be new
    void enter(T&& e)                           { auto p = _s.insert(std::move(e)); ASSERTX(p.second); } // must be new
    const T& enter(const T& e, bool& is_new)     { auto p = _s.insert(e); is_new = p.second; return *p.first; }
    // omit "const T& enter(T&& e, bool& is_new)" because e could be lost if !is_new
    bool add(const T& e)                        { auto p = _s.insert(e); return p.second; } // ret: is_new
    // omit "bool add(T&& e)" because e could be lost if !is_new
    bool remove(const T& e)                     { return remove_i(e); } // ret: was_found
    bool contains(const T& e) const             { return _s.find(e)!=end(); }
    int num() const                             { return narrow_cast<int>(_s.size()); }
    size_t size() const                         { return _s.size(); }
    bool empty() const                          { return _s.empty(); }
    const T& retrieve(const T& e, bool& present) const { return retrieve_i(e, present); }
    const T& retrieve(const T& e) const         { auto i = _s.find(e); return i!=end() ? *i : def(); }
    const T& get(const T& e) const              { auto i = _s.find(e); ASSERTXX(i!=end()); return *i; }
    const T& get_one() const                    { ASSERTXX(!empty()); return *begin(); }
    const T& get_random(Random& r) const        { auto i = crand(r); return *i; }
    T remove_one()              { ASSERTXX(!empty()); T e = std::move(*begin()); _s.erase(begin()); return e; }
    T remove_random(Random& r)                  { auto i = crand(r); T e = std::move(*i); _s.erase(i); return e; }
    iterator begin()                            { return _s.begin(); }
    const_iterator begin() const                { return _s.begin(); }
    iterator end()                              { return _s.end(); }
    const_iterator end() const                  { return _s.end(); }
 private:
    base _s;
    static const T& def()                       { static const T k_default = T{}; return k_default; }
    const T& retrieve_i(const T& e, bool& present) const {
        auto i = _s.find(e); present = i!=end(); return present ? *i : def();
    }
    bool remove_i(const T& e) {
        if (_s.erase(e)==0) return false;
        if (1 && _s.size()<_s.bucket_count()/16) _s.rehash(0);
        return true;
    }
    const_iterator crand(Random& r) const { // see also similar code in Map
        assertx(!empty());
        if (0) {
            return std::next(begin(), r.get_size_t()%_s.size()); // likely slow; no improvement
        } else {
            size_t nbuckets = _s.bucket_count();
            size_t bn = r.get_size_t()%nbuckets;
            size_t ne = _s.bucket_size(bn);
            size_t nskip = r.get_size_t()%(20+ne);
            while (nskip>=_s.bucket_size(bn)) {
                nskip -= _s.bucket_size(bn);
                bn++; if (bn==nbuckets) bn = 0;
            }
            auto li = _s.begin(bn);
            while (nskip--) { ASSERTXX(li!=_s.end(bn)); ++li; }
            ASSERTXX(li!=_s.end(bn));
            // convert from const_local_iterator to const_iterator
            auto i = _s.find(*li); ASSERTXX(i!=_s.end());
            return i;
        }
    }
    // Default operator=() and copy_constructor are safe.
};

template<typename T> HH_DECLARE_OSTREAM_RANGE(Set<T>);
template<typename T> HH_DECLARE_OSTREAM_EOL(Set<T>);

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_SET_H_
