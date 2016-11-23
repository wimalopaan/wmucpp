/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

//#include "../tests/algorithm.h"
//#include "../tests/percent.h"
//#include "../tests/duration.h"
//#include "../tests/physical.h"
//#include "../tests/types.h"
//#include "../tests/atomic.h"

//int main(){
//}

//void assertFunction(bool b, const char *function, const char *file, unsigned int line) {
//    if (!b) {
//        std::cout << "Assertion failed in: " << function << " file: " << file << " line: " << line << std::endl;
//    }
//}

#include <memory>
#include <set>
#include <functional>

struct A {
    int value;
    A(int _value) : value(_value) {}
};
bool operator<(const A& lhs, const A& rhs) {
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    return lhs.value < rhs.value;
}

namespace std {
template<>
struct less<std::shared_ptr<A>> {
      bool operator()(const std::shared_ptr<A> &lh, const std::shared_ptr<A> &rh) {
          std::cout << __PRETTY_FUNCTION__ << std::endl;
          return *lh < *rh;
      }
}; 
}

struct B {
    typedef std::shared_ptr<A> ptr_t;
    typedef std::set<ptr_t> set_t;
    
    int num;

    set_t data;
    
    B(int _num) : num(_num) {}
};
bool operator<(const B& lhs, const B& rhs) {
    return lhs.num< rhs.num;
}

namespace std {
template<>
struct less<std::shared_ptr<B>> {
      bool operator()(const std::shared_ptr<B> &lh, const std::shared_ptr<B> &rh) {
          std::cout << __PRETTY_FUNCTION__ << std::endl;
          return *lh < *rh;
      }
}; 
}

struct C {
    typedef std::shared_ptr<B> ptr_t;
    typedef std::set<ptr_t> set_t;
    set_t data;
};


int main() {
  B::ptr_t a1(new A(1));
  B::ptr_t a2(new A(2));

  C::ptr_t b(new B(1));

  b->data.insert(a1);
  b->data.insert(a2);

  C c;

  c.data.insert(b);
  c.data.insert(b);

}