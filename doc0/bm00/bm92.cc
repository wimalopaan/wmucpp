#include <utility>
#include <array>
#include <variant>
#include <type_traits>

struct MCU {
    struct P1 {
        static inline volatile std::byte r1;
        static inline volatile std::byte r2;
    };
    struct P2 {
        static inline volatile std::byte r1;
        static inline volatile std::byte r2;
    };
};


struct A {
//    A(std::byte v = 0_B) : value(v){
//        asm(";Ac");
//    }
    inline constexpr std::byte get() const {
        return value;
    }
    inline void set(std::byte v) const {
        MCU::P1::r1 = v;
    }
    inline void toggle(std::byte v) const {
        MCU::P1::r1 |= v;
    }
    inline void store() {
        value = MCU::P1::r2;
    }
    std::byte value{};
//    ~A() {
//        asm(";Ad");
//    }
};
struct B {
//    B(std::byte v = 0_B) : value(v){
//        asm(";Bc");
//    }
    inline constexpr std::byte get() const {
        return value;
    }
    inline void set(std::byte v) const {
        MCU::P2::r1 = v;
    }
    inline void toggle(std::byte v) const {
        MCU::P2::r1 |= v;
    }
    inline void store() {
        value = MCU::P2::r2;
    }
    std::byte value{};
//    ~B() {
//        asm(";Bd");
//    }
};

namespace  {
    A x1{32_B};
    B x2{42_B};
    A x3{52_B};   
    B x4{62_B};   
}

int main() {
    std::array<std::variant<A, B, nullptr_t>, 4> p{x1, x2, x3, nullptr};
//    p[0] = x4;
    while(true) {
        std::byte x{};
        for(auto&& i : p) {
            i.visit([&]<typename P>(P& v){
                        if constexpr(!std::is_same_v<P, nullptr_t>) {
                            v.store();
                            x |= v.get();
                        }
            });
        }
        x1.set(x);
//        p[0].visit([&]<typename V>(V& v){
//                       if constexpr(!std::is_same_v<V, nullptr_t>) {
            
//                           v.set(x);
//                       }
//        });
    }
}
