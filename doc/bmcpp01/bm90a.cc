/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <cstddef>
#include <array>
#include <initializer_list>
#include <stdlib.h>

void * operator new([[maybe_unused]] size_t size) noexcept
{
    asm(";new");
    return nullptr;
}

void operator delete([[maybe_unused]] void * ptr)
{
    asm(";delete1");
}
void operator delete([[maybe_unused]] void * ptr,[[maybe_unused]]  unsigned int)
{
    asm(";delete2");
}

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
// 1) Implementierungsinformation (0) (RTTI) (geht auch nicht: -frtti -> undefined reference
// 2) Implementierungsinformation (0)
// -- 2a) (bei virtuellem dtor) dtor1 (ptr, size)
// -- 2b) (bei virtuellem dtor) dtor2 (ptr, size)
// 3) Zeiger auf f()
// 4) Zeiger auf weitere Funktionen

struct IIX {
    virtual ~IIX() {
//        asm(";d1");
    }
//    virtual void f() const = 0; // benötigt __cx_pure_virtual
    virtual void f() const {}
};

struct IX : public IIX {
    virtual ~IX() {
        asm(";d2");
    } // <> benötigt delete, man kann also nur Zeiger auf bestehende Objekte verwenden -> undefined reference
//    virtual void f() const = 0;
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
    std::array<IX*, 2> objects{&a, &b};
    
    for(const auto o: objects) {
        if(o) {
            o->f();
        }
    }
    
//    IX* a1 = &a;
    
//    auto z = dynamic_cast<A*>(a1);
    
    
//    std::vector<IX*> v = {&a, &b, nullptr};
//    for(const auto o: v) {
//        if(o) {
//            o->f();
//        }
//    }

    while(true);
}
