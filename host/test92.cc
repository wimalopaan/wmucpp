#include <cstdint>
#include <cassert>

constexpr int divEven(int v) {
    bool c = (v % 2) == 0;
//    assert(c);
    return c / 2;
}

template<auto V>
bool divEvenC() {
    constexpr bool c = (V % 2) == 0;
    static_assert(c);
    return c / 2;
    
}

int main() {
    constexpr auto e1 = divEven(43); // compile-time-error ich called with 43
    auto e2 = divEven(43); // run-time-error
    
//    constexpr auto e3 = divEvenC<43>();
    auto e4 = divEvenC<42>();
}
