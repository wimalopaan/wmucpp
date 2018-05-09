#define NDEBUG

#include <cstdint>
#include <cstddef>
#include <array>

#include "util/meta.h"
#include "util/static_interface.h"


#include "console.h"
#include "simavr/simavrdebugconsole.h"
using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

template<typename... Impl>
struct Interface : Static::InterfaceBase<Impl...> {
    using base = Static::InterfaceBase<Impl...>;
    using implementors = typename base::implementors;
    using ptype = typename base::ptype;

    inline static const ptype* begin(const ptype& n) {
        const ptype* r = nullptr;
        Meta::visitAt<implementors>(n.id(), [&](auto i){
            r = n.get(i)->begin();
        });
        return r;
    }
    
    inline static const ptype* end(const ptype& n) {
        const ptype* r = nullptr;
        Meta::visitAt<implementors>(n.id(), [&](auto i){
            r = n.get(i)->end();
        });
        return r;
    }
    inline static bool hasChildren(const ptype& n) {
        bool r = false;
        Meta::visitAt<implementors>(n.id, [&](auto i){
            r = n.get(i)->hasChildren();
        });
        return r;
    }
};

template<auto N> struct A;
template<auto N> struct B;

using If = Interface<A<0>, A<1>, A<2>, B<0>>;

template<auto N = 8>
struct A : If {
    template<typename... TP>
    A(const TP&... cc) : mChildren{cc...} {}
    
    bool hasChildren() const {
        return N != 0;
    }
    
    const ptype* begin() const {
        if constexpr(N > 0) {
            return &mChildren[0];
        }
        else {
            return nullptr;
        }
    }
    const ptype* end() const {
        if constexpr(N > 0) {
            return &mChildren[N - 1] + 1;
        }
        else {
            return nullptr;
        }
    }
    std::array<ptype, N> mChildren;
};

template<auto N = 8>
struct B : If {
    template<typename... TP>
    B(const TP&... cc) : mChildren{cc...} {}

    bool hasChildren() const {
        return N != 0;
    }

    const ptype* begin() const {
        if constexpr(N > 0) {
            return &mChildren[0];
        }
        else {
            return nullptr;
        }
    }
    const ptype* end() const {
        if constexpr(N > 0) {
            return &mChildren[N - 1] + 1;
        }
        else {
            return nullptr;
        }
    }
    std::array<ptype, N> mChildren;
};

A<0> l1;
A<0> l2;
A<1> a1(If::make_pointer(l1));
A<2> a2(If::make_pointer(l1), If::make_pointer(l2));
B<0> b;

auto ta1 = If::make_pointer(a2);

int main() {
    uint8_t y = 0;
    
    std::outl<terminal>("E: "_pgm, RAMEND);
    std::outl<terminal>("S: "_pgm, RAMSTART);
    
    std::outl<terminal>("T: "_pgm, If::typeBits);
    std::outl<terminal>("A: "_pgm, If::adrBits);

    std::outl<terminal>("L: "_pgm, If::size);
    
    
    for(auto it = If::begin(ta1); it != If::end(ta1); ++it) {
        auto cc = *it;
        ++y;
        for(auto it2 = If::begin(cc); it2 != If::end(cc); ++it2) {
        }
    }
    std::outl<terminal>(y);
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    while(true) {}
}
#endif
