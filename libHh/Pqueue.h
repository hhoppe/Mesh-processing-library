// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PQUEUE_H_
#define MESH_PROCESSING_LIBHH_PQUEUE_H_

#include "Array.h"
#include "Map.h"

namespace hh {

namespace details {
namespace PQ {
template<typename T> struct Node {
    Node()                                  = default;
    explicit Node(const T& e, float pri)    : _e(e), _pri(pri) { }
    explicit Node(T&& e, float pri)         : _e(std::move(e)), _pri(pri) { }
    Node& operator=(Node&& n) noexcept      { _e = std::move(n._e); _pri = n._pri; return *this; }
    T _e;
    float _pri;
};
} // namespace PQ
} // namespace details

// Self-resizing priority queue.  Note: much code duplicated in HPqueue!
template<typename T> class Pqueue : noncopyable {
 public:
    void clear()                                { _ar.clear(); }
    void enter(const T& e, float pri)           { ASSERTX(pri>=0), enter_i(e, pri); }
    void enter(T&& e, float pri)                { ASSERTX(pri>=0), enter_i(std::move(e), pri); }
    void reserve(int size)                      { _ar.reserve(size); }
    int num() const                             { return _ar.num(); }
    size_t size() const                         { return _ar.size(); }
    bool empty() const                          { return !num(); }
    const T& min() const                        { return (ASSERTXX(!empty()), _ar[0]._e); }
    float min_priority() const                  { return (ASSERTXX(!empty()), _ar[0]._pri); }
    T remove_min()                              { return (ASSERTXX(!empty()), remove_min_i()); }
    void enter_unsorted(const T& e, float pri)  { return (ASSERTX(pri>=0), _ar.push(Node(e, pri))); }
    void enter_unsorted(T&& e, float pri)       { ASSERTX(pri>=0), _ar.push(Node(std::move(e), pri)); }
    void sort()                                 { sort_i(); }
 private:
    using Node = details::PQ::Node<T>;
    Array<Node> _ar;
    void nmove(int n1, int n2) {
        _ar[n1]._e = std::move(_ar[n2]._e);
        _ar[n1]._pri = _ar[n2]._pri;
    }
    int adjust_up(int n, const float cp) {
        for (;;) {
            if (!n) break;
            int pn = (n-1)/2;   // parent node
            if (cp<_ar[pn]._pri) { nmove(n, pn); n = pn; continue; }
            break;
        }
        return n;
    }
    int adjust_down(int n, const float cp) {
        for (;;) {
            int ln = n*2+1;       // left child node
            if (ln>=num()) break; // no children
            float lp = _ar[ln]._pri;
            int rn = n*2+2;     // right child node
            if (rn>=num()) {    // no right child
                if (cp>lp) { nmove(n, ln); n = ln; continue; }
                break;
            }
            float rp = _ar[rn]._pri;
            if (cp>lp) {
                if (lp<rp) {
                    nmove(n, ln); n = ln; continue;
                } else {
                    nmove(n, rn); n = rn; continue;
                }
            }
            if (cp>rp) { nmove(n, rn); n = rn; continue; }
            break;
        }
        return n;
    }
    void enter_i(const T& e, float pri) {
        _ar.add(1);             // leave this new node uninitialized
        int j = adjust_up(num()-1, pri);
        _ar[j]._e = e;
        _ar[j]._pri = pri;
    }
    void enter_i(T&& e, float pri) {
        _ar.add(1);             // leave this new node uninitialized
        int j = adjust_up(num()-1, pri);
        _ar[j]._e = std::move(e);
        _ar[j]._pri = pri;
    }
    T remove_min_i() {
        T e = std::move(_ar[0]._e);
        if (num()==1) { _ar.sub(1); return e; }
        T e0 = std::move(_ar[num()-1]._e); float pri = _ar[num()-1]._pri;
        _ar.sub(1);
        int j = adjust_down(0, pri);
        _ar[j]._e = std::move(e0);
        _ar[j]._pri = pri;
        return e;
    }
    void sort_i() {
        for (int i = (num()-2)/2; i>=0; --i) {
            T e = std::move(_ar[i]._e); float pri = _ar[i]._pri;
            int j = adjust_down(i, pri);
            // It would be nice to have faster case for j==i.
            _ar[j]._e = std::move(e);
            _ar[j]._pri = pri;
        }
    }
};

// Hashed priority queue allowing insertion/deletion/update.  Note: much code duplicated in Pqueue!
template<typename T, typename Hash = std::hash<T>, typename Equal = std::equal_to<T>> class HPqueue : noncopyable {
 public:
    void clear()                                { _ar.clear(), _m.clear(); }
    void enter(const T& e, float pri)           { ASSERTX(pri>=0), enter_i(e, pri); }
    void reserve(int size)                      { _ar.reserve(size); }
    int num() const                             { return _ar.num(); }
    size_t size() const                         { return _ar.size(); }
    bool empty() const                          { return !num(); }
    const T& min() const                        { return (ASSERTXX(!empty()), _ar[0]._e); }
    float min_priority() const                  { return (ASSERTXX(!empty()), _ar[0]._pri); }
    T remove_min()                              { return (ASSERTXX(!empty()), remove_min_i()); }
    void enter_unsorted(const T& e, float pri)  { ASSERTX(pri>=0), _m.enter(e, num()), _ar.push(Node(e, pri)); }
    void sort()                                 { return sort_i(); }
    bool contains(const T& e) const             { return _m.contains(e); }
    float retrieve(const T& e) const            { bool b; int i = _m.retrieve(e, b); return b ? _ar[i]._pri : -1.f; }
    float remove(const T& e)                    { return remove_i(e); }                       // ret pri or <0
    float update(const T& e, float pri)         { return (ASSERTX(pri>=0), update_i(e, pri)); } // ret prevpri or <0
    float enter_update(const T& e, float pri)   { return (ASSERTX(pri>=0), enter_update_i(e, pri)); } // prevpri or <0
    bool enter_update_if_smaller(const T& e, float pri) { return (ASSERTX(pri>=0), enter_update_if_smaller_i(e, pri)); }
    bool enter_update_if_greater(const T& e, float pri) { return (ASSERTX(pri>=0), enter_update_if_greater_i(e, pri)); }
 private:
    using Node = details::PQ::Node<T>;
    Array<Node> _ar;
    Map<T, int, Hash, Equal> _m;        // element -> index in array
    void consider_shrink() {
        if (0 && num()<_ar.capacity()*.4f && _ar.capacity()>100) reserve(_ar.capacity()/2);
    }
    void nmove(int n1, int n2) {
        _ar[n1] = std::move(_ar[n2]);
        int on2 = _m.replace(_ar[n1]._e, n1); ASSERTX(on2==n2);
    }
    // (cp is the priority of the current node n, which may not be up-to-date in _ar[n])
    // After this call returns index j, if j!=n, elements have been shifted,
    //  and the old element at n should be moved into its new location at j.
    int adjust(int n, const float cp, bool up, bool down) {
        int orig_n = n;
        if (up) {
            for (;;) {
                if (!n) break;
                int pn = (n-1)/2;
                if (cp<_ar[pn]._pri) { nmove(n, pn); n = pn; continue; }
                break;
            }
            if (n!=orig_n) return n;
        }
        if (down) {
            for (;;) {
                int ln = n*2+1;       // left child
                if (ln>=num()) break; // no children
                float lp = _ar[ln]._pri;
                int rn = n*2+2;  // right child
                if (rn>=num()) { // no right child
                    if (cp>lp) { nmove(n, ln); n = ln; continue; }
                    break;
                }
                float rp = _ar[rn]._pri;
                if (cp>lp) {
                    if (lp<rp) {
                        nmove(n, ln); n = ln; continue;
                    } else {
                        nmove(n, rn); n = rn; continue;
                    }
                }
                if (cp>rp) { nmove(n, rn); n = rn; continue; }
                break;
            }
        }
        return n;
    }
    void enter_i(const T& e, float pri) {
        _ar.add(1);             // leave this new node uninitialized
        int j = adjust(num()-1, pri, true, false);
        _ar[j]._e = e; _ar[j]._pri = pri;
        _m.enter(e, j);
    }
    void sort_i() {
        for (int i = (num()-2)/2; i>=0; --i) {
            T e = _ar[i]._e; float pri = _ar[i]._pri;
            int j = adjust(i, pri, false, true);
            if (j!=i) {
                _ar[j]._e = e; _ar[j]._pri = pri;
                int oi = _m.replace(e, j); ASSERTX(oi==i);
            }
        }
    }
    T remove_min_i() {
        T e = _ar[0]._e;
        if (num()==1) { _ar.sub(1); int j = _m.remove(e); ASSERTX(j==0); return e; }
        T e0 = _ar[num()-1]._e; float pri = _ar[num()-1]._pri;
        _ar.sub(1);
        { int j = _m.remove(e); ASSERTX(j==0); }
        int j = adjust(0, pri, false, true);
        _ar[j]._e = e0;
        _ar[j]._pri = pri;
        int oi = _m.replace(e0, j); ASSERTX(oi==num());
        consider_shrink();
        return e;
    }
    float remove_i(const T& e) {
        bool present; int i = _m.retrieve(e, present);
        if (!present) return -1.f;
        float ppri = _ar[i]._pri;
        T e0 = _ar[num()-1]._e; float pri = _ar[num()-1]._pri;
        _ar.sub(1);
        { int j = _m.remove(e); ASSERTX(j==i); }
        if (i<num()) {          // if num() was 1, we have i==0, num()==0
            int j = adjust(i, pri, true, true);
            _ar[j]._e = e0;
            _ar[j]._pri = pri;
            int oi = _m.replace(e0, j); ASSERTX(oi==num());
        }
        consider_shrink();
        return ppri;
    }
    float update_i(const T& e, float pri) {
        bool present; int i = _m.retrieve(e, present);
        if (!present) return -1.f;
        float oldpri = _ar[i]._pri;
        int j = adjust(i, pri, true, true);
        _ar[j]._pri = pri;
        if (j!=i) {
            _ar[j]._e = e;
            int oi = _m.replace(e, j); ASSERTX(oi==i);
        }
        return oldpri;
    }
    float enter_update_i(const T& e, float pri) {
        bool present; int i = _m.retrieve(e, present);
        if (!present) {
            enter(e, pri);
            return -1.f;
        } else {
            float oldpri = _ar[i]._pri;
            int j = adjust(i, pri, true, true);
            _ar[j]._pri = pri;
            if (j!=i) {
                _ar[j]._e = e;
                int oi = _m.replace(e, j); ASSERTX(oi==i);
            }
            return oldpri;
        }
    }
    bool enter_update_if_smaller_i(const T& e, float pri) {
        bool present; int i = _m.retrieve(e, present);
        if (!present) {
            enter(e, pri);
            return true;
        } else if (pri<_ar[i]._pri) {
            int j = adjust(i, pri, true, false);
            _ar[j]._pri = pri;
            if (j!=i) {
                _ar[j]._e = e;
                int oi = _m.replace(e, j); ASSERTX(oi==i);
            }
            return true;
        }
        return false;
    }
    bool enter_update_if_greater_i(const T& e, float pri) {
        bool present; int i = _m.retrieve(e, present);
        if (!present) {
            enter(e, pri);
            return true;
        } else if (pri>_ar[i]._pri) {
            int j = adjust(i, pri, false, true);
            _ar[j]._pri = pri;
            if (j!=i) {
                _ar[j]._e = e;
                int oi = _m.replace(e, j); ASSERTX(oi==i);
            }
            return true;
        }
        return false;
    }
};

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_PQUEUE_H_
