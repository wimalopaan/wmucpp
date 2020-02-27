#include <cstdint>
#include <utility>
#include <cmath>
#include <array>

#include <etl/algorithm.h>

template<typename T>
struct A {
    T m;
};

//using tt = A<uint32_t>;
//using tt = uint32_t;
using tt = float;

tt f(tt x);
tt call(tt x) {
    return f(x);
}


int main() {
    std::array<int, 10> a;
    std::array<int, 10> b;
    constexpr auto s = std::size(a);
    
    etl::copy(a, b);
    
}
//int main() {
//}

tt f(tt x) {
    return x;
}



