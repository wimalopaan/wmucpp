#include <avr/io.h>
#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif
#include <array>

volatile uint8_t x;
volatile uint8_t n;

static constexpr uint8_t n_mask{0x0f};
//static constexpr uint8_t n_mask{0x1f}; // führt zu UB (s.u.)

namespace Pgm {
    template<template<auto> typename G, typename S>
    struct Array;
    
    template<template<auto> typename G, auto... II>
    struct Array<G, std::index_sequence<II...>> {
        using generator = G<sizeof...(II)>;
        using value_type = typename generator::value_type;
        using index_type = typename generator::index_type;
        static inline value_type value(const index_type index) {
            if constexpr(std::is_same_v<value_type, uint8_t>) {
                return {pgm_read_byte((uint8_t*)&data[index])};
            }
        }
    private:
        inline static constexpr auto mData = generator{}();    
        inline static constexpr const value_type data[] PROGMEM = {mData[II] ...}; 
    };
}

template<auto Size>
struct Generator {
    using value_type = uint8_t;
    using index_type = uint8_t;
    constexpr auto operator()() const {
        std::array<value_type, Size> d;
        for(uint8_t i{0}; auto& v : d) {
            v = 0xffu >> i++; // falls UB vorhanden, wird es aufgedeckt
        }
        return d;
    }
};

using Lut = Pgm::Array<Generator, std::make_index_sequence<(n_mask + 1)>>;

void test1() {
    uint8_t m{0xff};
    for(uint8_t i{}; i < (n & n_mask); ++i) {
        m >>= 1; // 8-Bit Shift
    }
    x = m;    
}

void test2() {
    uint8_t m2{0xff};
    x = m2 >> (n & n_mask);  // no UB: Int-Promo 
}

void test3() {
    static constexpr auto lut = Generator<(n_mask + 1)>{}();
    x = lut[n & n_mask];
}

void test5() {
    x = Lut::value(n & n_mask);
}

void test4() {
    switch(n) {
    case 0:
        x = 0xff;
        break;
    case 1:
        x = 0x7f;
        break;
    case 2:
        x = 0x3f;
        break;
    case 3:
        x = 0x1f;
        break;
    case 4:
        x = 0x0f;
        break;
    case 5:
        x = 0x07;
        break;
    case 6:
        x = 0x03;
        break;
    case 7:
        x = 0x01;
        break;
    default:
        x = 0;
        break;
    }
}

int main() {
}
