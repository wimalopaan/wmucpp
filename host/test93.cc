#include <cstdint>
#include <cstddef>
#include <compare> 
#include <vector>
#include <array>
#include <iostream>

//struct uint {
//    uint8_t value;
//};

template<auto V>
struct A {};

struct B {
    constexpr B(int a) : value{a} {}
//    inline constexpr auto operator<=>(const B& rhs) const = default;
//private:
    uint8_t value;
};

int main() {
    A<B{3}> t;        
}

