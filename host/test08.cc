#include <cstdint>
#include <cstddef>

//consteval bool bad0() {
//    for(uint8_t i = 0; i < 33; i++) {
//        uint32_t mask = 1 << i;
//    }
//    return true;
//}

//template<typename T>
constexpr bool bad1() {
    for(uint32_t i{0}; i < 33; i++) {
        uint32_t mask = 1 << i;
    }
    return true;
}

//constexpr auto test1 = bad1();

int main() {
//    bad0();
//    bad1<int>();
}
