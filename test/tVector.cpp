// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <list>
#include <deque>

#include "Stack.h"              // vec_contains()
using namespace hh;

namespace {

// *** adapted from tStack.cpp

void test_stack() {
    {
        struct ST {
            ST(int i) : _i(i) { }
            int _i;
        };
        std::vector<const ST*> s;
        assertx(s.empty());
        s.push_back(new ST(1)); // never deleted
        s.push_back(new ST(2));
        s.push_back(new ST(3));
        assertw(vec_pop(s)->_i==3);
        assertw(vec_pop(s)->_i==2);
        assertw(vec_pop(s)->_i==1);
    }
    {
        std::vector<int> s;
        s.push_back(0);
        s.push_back(1);
        s.push_back(2);
        int i = 0;
        for (int j : s) {
            assertw(j==i++);
        }
        assertx(i==3);
        assertw(vec_pop(s)==2);
        assertw(vec_pop(s)==1);
        assertw(vec_pop(s)==0);
        assertx(s.empty());
    }
    {
        std::vector<float> s;
        s.push_back(1);
        s.push_back(4);
        s.push_back(9);
        int i = 0;
        for (float v : s) {
            assertw(v==square(i+1)); i++;
        }
        assertx(i==3);
        assertw(vec_pop(s)==9);
        assertw(vec_pop(s)==4);
        assertw(vec_pop(s)==1);
        assertx(s.empty());
    }
    {
        std::vector<int> s;
        assertx(s.empty());
        for (int i : s) { dummy_use(i); if (1) assertnever(""); }
        for_int(i, 4) { s.push_back(i); }
        assertw(s.size()==4);
        assertw(!s.empty());
        assertw(vec_contains(s, 2));
        assertw(s.back()==3);
        assertw(vec_pop(s)==3);
        assertw(vec_pop(s)==2);
        {
            int i = 0;
            for (int j : s) {
                assertw(j==i++);
            }
        }
        assertw(!vec_contains(s, 2));
        assertw(vec_pop(s)==1);
        assertw(vec_pop(s)==0);
        assertw(s.empty());
    }
}

// *** from tQueue.cpp

template<typename T> bool queue_contains(const std::deque<T>& queue, const T& e) {
    for (auto& ee : queue) { if (e==ee) return true; }
    return false;
}

template<typename T> T queue_pop(std::deque<T>& queue) {
    T e = std::move(queue.front()); queue.pop_front(); return e;
}

void test_queue() {
    std::deque<int> q;
    assertx(q.empty());
    for (int i : q) { dummy_use(i); if (1) assertnever(""); }
    for_int(i, 4) { q.push_back(i); }
    assertw(q.size()==4);
    assertw(!q.empty());
    assertw(queue_contains(q, 1));
    assertw(q.front()==0);
    assertw(queue_pop(q)==0);
    assertw(queue_pop(q)==1);
    {
        int i = 0;
        for (int j : q) {
            assertw(j==2+i++);
        }
    }
    assertw(!queue_contains(q, 1));
    q.push_front(5);
    q.push_front(4);
    assertw(queue_pop(q)==4);
    assertw(queue_pop(q)==5);
    assertw(queue_pop(q)==2);
    assertw(queue_pop(q)==3);
    assertw(q.empty());
}

// *** from tHList

template<typename T> bool list_contains(const std::list<T>& l, const T& e) {
    for (const auto& ee : l) { if (e==ee) return true; }
    return false;
}

void test_list() {
    std::list<int> l;
    assertx(l.empty());
    for (int i : l) { dummy_use(i); if (1) assertnever(""); }
    l.push_front(1);
    l.push_front(0);
    l.push_back(2);
    l.push_back(3);
    assertw(l.size()==4);
    assertw(!l.empty());
    for_intL(i, -2, 6) { assertw(list_contains(l, i)==(i>=0 && i<4)); }
    assertw(l.front()==0);
    assertw(l.back()==3);
    assertw(l.front()==0); l.pop_front();
    assertw(l.front()==1); l.pop_front();
    assertw(l.back()==3); l.pop_back();
    assertw(l.back()==2); l.pop_back();
    assertw(l.empty());
    assertw(l.size()==0);
    l.push_back(1);
    l.push_back(2);
    // l.erase(remove(l.begin(), l.end(), 77), l.end()); // remove element 77
    // l.insert_before(1, 0);
    l.insert(find(l.begin(), l.end(), 1), 0);
    // l.insert_after(2, 3);
    l.insert(++find(l.begin(), l.end(), 2), 3);
    {
        int i = 0;
        for (int j : l) { assertw(j==i++); }
    }
    // assertw(l.before(2)==1);
    // assertw(l.after(2)==3);
    // assertw(l.after(3)==0);
    // assertw(l.before(0)==0);
    l.clear();
}

} // namespace

int main() {
    test_stack();
    test_queue();
    test_list();
}
