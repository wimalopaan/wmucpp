/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include <stdint.h>
#include <stdlib.h>
#include "std/array.h"
#include "std/initializer_list.h"

namespace std {
    template<typename T>
    class vector {
    public:
        vector(const std::initializer_list<T>& ilist) : mData(static_cast<T*>(malloc(sizeof(T) * ilist.size()))), mSize(ilist.size()){
            T* d = mData;
            for(const auto& v : ilist) {
                *d++ = v;
            }
        }
        T* begin() const {
            return mData;
        }
        T* end() const {
            return mData + mSize;
        }
    private:
        T* mData = nullptr;
        uint16_t mSize = 0;
    };
}

volatile uint8_t x;

// Aufbau vtable:
// 1) Implementierungsinformation (RTTI) (geht auch nicht: -frtti -> undefined reference
// 2) Zeiger auf dtor
// 3) Zeiger auf f()
// 4) Zeiger auf weitere Funktionen

struct IX {
    //    virtual ~IX() {}; // <> benötigt delete, man kann also nur Zeiger auf bestehende Objekte verwenden -> undefined reference
    virtual void f() const = 0;
};

class A : public IX {
public:
    virtual void f() const override {
        x = v;        
    }
private:
    uint8_t v{0};
};
class B : public IX {
public:
    virtual void f() const override {
        x = v;        
    }
private:
    uint8_t v{0};
};

A a;
B b;

int main() {
//    std::array<IX*, 2> objects{&a, &b};
    
//    for(const auto o: objects) {
//        if(o) {
//            o->f();
//        }
//    }
    
//    std::vector<IX*> v = {&a, &b, nullptr};
//    for(const auto o: v) {
//        if(o) {
//            o->f();
//        }
//    }

    while(true);
}
