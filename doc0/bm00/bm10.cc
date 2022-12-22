#include <cstdint>
#include <limits>
#include <array>
#include <cmath>

int main() {    
//    uint8_t a[10] = {};
    
//    for(uint8_t i = 0; i < 10; ++i) {
//        a[i] = 42 + i;
//    }
    
//    return a[8];
    
//    constexpr auto b = []{
//        int8_t x = std::numeric_limits<int8_t>::max();
////        ++x;
//        return (x > std::numeric_limits<int8_t>::max());
//    }();

//    auto p = "1" + 2;

//    int a1[100] {};
    
    auto a = []{
        std::array<uint8_t, 100> a;
        for(size_t i{}; auto& e: a) {
            e = sin(2 * M_PI / a.size());
        }
        return a;
    }();
    
//    constexpr auto b2 = []{
//        uint8_t a[10];
//        uint8_t b[10];
        
//        auto p1 = &a[0];
////        auto p2 = &b[9];
//        auto p2 = &a[9];
        
//        return p1 < p2;
//    }();
}
