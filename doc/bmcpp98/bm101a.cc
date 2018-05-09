#define NDEBUG

#include <cstdint>
#include <cstddef>
#include <array>

#include "container/pgmarray.h"

#include "util/meta.h"

struct IF {
    virtual bool hasChildren() const = 0;
    
    virtual IF* const* begin() const = 0;
    virtual IF* const* end() const = 0;
      
};
template<auto N = 8>
struct A : IF {
    template<typename... TP>
    A(TP&... cc) : mChildren{&cc...} {}
    
    bool hasChildren() const {
        return N != 0;
    }
    
    IF* const* begin() const {
        if constexpr(N > 0) {
            return &mChildren[0];
        }
        else {
            return nullptr;
        }
    }
    IF* const* end() const {
        if constexpr(N > 0) {
            return &mChildren[N - 1] + 1;
        }
        else {
            return nullptr;
        }
    }
    const std::array<IF*, N> mChildren;
};

template<auto N = 8>
struct B : IF {
    template<typename... TP>
    B(const TP&... cc) : mChildren{&cc...} {}

    bool hasChildren() const {
        return N != 0;
    }

    IF* const* begin() const {
        if constexpr(N > 0) {
            return &mChildren[0];
        }
        else {
            return nullptr;
        }
    }
    IF* const * end() const {
        if constexpr(N > 0) {
            return &mChildren[N - 1] + 1;
        }
        else {
            return nullptr;
        }
    }
    const std::array<IF*, N> mChildren;
};


A<0> l1;
A<0> l2;
A<1> a1(l1);
A<2> a2(l1, l2);
B<0> b;

IF* ta1 = &a2;

int main() {
    uint8_t y = 0;
    
    for(auto it = ta1->begin(); it != ta1->end(); ++it) {
        auto cc = *it;
        for(auto it2 = cc->begin(); it2 != cc->end(); ++it2) {
            ++y;
        }
    }
    return y;
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    while(true) {}
}
#endif
