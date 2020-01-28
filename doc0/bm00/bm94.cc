#include <utility>
#include <variant>
#include <type_traits>
#include <etl/vector.h>

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
};

namespace  {
    //    A x1{32_B};
    //    B x2{42_B};
    //    A x3{52_B};   
    //    B x4{62_B};   
}

int main() {
    //    etl::Vector<std::variant<B, A>, 4> p{A{32_B}, B{42_B}, B{52_B}};
    //    etl::Vector<std::variant<B, A>, 4> p{x1, x2, x3, x4};
    etl::Vector<std::variant<B, A>, 4> p;
    p.push_back(A{32_B});
    p.push_back(B{42_B});
    p.push_back(A{52_B});
    p.push_back(B{62_B});
    
    //    p.push_back(x1);
    //    p.push_back(x2);
    //    p.push_back(x3);
    //    p.push_back(x4);
    while(true) {
        std::byte x{};
        for(auto&& i : p) {
            i.visit([&]<typename P>(P& v){
                        v.store();
                        x |= v.get();
                    });
        }
        //        x1.set(x);
        p[0].visit([&]<typename V>(V& v){
                       if constexpr(!std::is_same_v<V, nullptr_t>) {
                           v.set(x);
                       }
                   });
    }
}

