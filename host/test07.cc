#include <iostream>
#include <cstdint>
#include <cstddef>
#include <cassert>

#include "local.h"
#include "local_types.h"

consteval inline int to_int(char c) {
    int result = 0;
    
    if (c >= 'A' && c <= 'F') {
        result = static_cast<int>(c) - static_cast<int>('A') + 10;
    }
    else if (c >= 'a' && c <= 'f') {
        result = static_cast<int>(c) - static_cast<int>('a') + 10;
    }
    else {
        result = static_cast<int>(c) - static_cast<int>('0');
    }
    
    return result;
}
template<std::size_t N>
consteval inline long long parse(const char (&arr)[N]) {
    long long base = 10;
    std::size_t offset = 0;
    
    if (N > 2) {
        bool starts_with_zero = arr[0] == '0';
        bool is_hex = starts_with_zero && arr[1] == 'x';
        bool is_binary = starts_with_zero && arr[1] == 'b';
        
        if (is_hex) {
            //0xDEADBEEF (hexadecimal)
            base = 16;
            offset = 2;
        }
        else if (is_binary) {
            //0b101011101 (binary)
            base = 2;
            offset = 2;
        }
        else if (starts_with_zero) {
            //012345 (octal)
            base = 8;
            offset = 1;
        }
    }
    
    long long number = 0;
    long long multiplier = 1;
    
    for (std::size_t i = 0; i < N - offset; ++i) {
        char c = arr[N - 1 - i];
        if (c != '\'') { // skip digit separators
            number += to_int(c) * multiplier;
            multiplier *= base;
        }
    }
    
    return number;
}
template <char... CC>
consteval auto operator"" _c() {
    return std::integral_constant<size_t, parse<sizeof...(CC)>({CC...})>{};
}

void bad() {
    for(uint8_t i{}; i < 33; i++) {
        uint32_t mask = 1 << i;
    }
}

consteval void better_0() {
    for(uint8_t i{}; i < 32; i++) {
        uint32_t mask = uint32_t{1} << i;
    }
}
void better_1() {
    for(uint8_t i{}; i < 32; i++) {
        assert(i < (sizeof(uint32_t) * 8));
        uint32_t mask = uint32_t{1} << i;
    }
}
void better_2() {
    for(uint8_t i{}; i < (sizeof(uint32_t) * 8); i++) {
        uint32_t mask = uint32_t{1} << i;
    }
}

template<size_t Bits>
struct mask_t {
private:
    struct Shift {
        using value_type = etl::typeForValue_t<Bits>;
        constexpr inline void operator++() {
            assert(*this);
            ++value;            
        }
        constexpr inline explicit operator bool() const {
            return value != Bits;
        }
        constexpr inline operator value_type() const {
            return value;
        }
    private:
        value_type value{};
    };
public:
    using value_type = etl::typeForBits_t<Bits>;
    using shift_type = Shift;
    constexpr inline explicit mask_t(const value_type v = value_type{}): value{v} {}

    constexpr inline bool any() const {
        return value != 0;
    }
        
    constexpr inline value_type toInt() const {
        return value;    
    }
    
    constexpr inline mask_t operator<<(const shift_type d) const {
        assert(d);
        return mask_t{value << d};        
    }
    template<size_t N>
    constexpr inline mask_t operator<<(const std::integral_constant<size_t, N>) const {
        static_assert(N < Bits);
        return mask_t{value << N};        
    }
    template<size_t N>
    constexpr inline void operator<<=(const std::integral_constant<size_t, N>) {
        static_assert(N < Bits);
        value <<= N;
    }
    private:
    value_type value{};
};

using m_t = mask_t<32>;

volatile uint32_t r;

namespace std {
    template<size_t N>
    constexpr inline auto operator-(std::integral_constant<size_t, N>) {
        return std::integral_constant<ssize_t, -N>{};    
    }
}

void better_10() {
    for(m_t::shift_type i; i; ++i) {
        m_t mask = m_t{0x0001} << i;
        r = mask.toInt();
        std::cout << "i: " << (int)i << " : " << mask.toInt() << '\n';
    }
    m_t m{0b0000'00001};
//    m <<= 33_c;
    
    while(m.any()) {
        m <<= 1_c;
    }
}

struct A {
    bool end() const {
        return value > 9;
    }
    void operator++() {
        ++value;
    }
private:
    int value{};
};

template<typename T>
bool end(T v) {
    return v.end();
}
template<>
bool end<int>(int v) {
    return v > 9;
}

template<typename T>
void bar() {
    T v; // default-initialization: non-class type -> indeterminate value
    T v1{}; // value-initialization: non-class type -> zero-initialization
//    T v2{0}; // direct-initialization: non-class type -> no narrowing; class-type -> need ctor callable with one argument
//    T v3 = 0; // copy-initialization: non-class-type -> narrowing; class-type -> need conversion-ctor
    T v4(); // function declaration
    while(!end(v)) {
        ++v;
    }
}

int main() {
//    bad();
//    better_0();
//    better_1();
//    better_2();
//    better_10();
    bar<int>();
    bar<A>();
}
